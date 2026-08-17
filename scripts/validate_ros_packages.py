#!/usr/bin/env python3
"""Statically validate the repository's ROS package metadata and resources."""

from __future__ import annotations

import ast
import configparser
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = REPOSITORY_ROOT / 'ws' / 'src'

EXPECTED_ROS_PACKAGES = {
    'esp_bridge',
    'esp_fusion_pkg',
    'fusion_pkg',
    'gap_follow',
    'lidar_logger_pkg',
    'tof_processor_pkg',
    'ultrasonic_pkg',
    'unitree_lidar_ros2',
    'vfh_pkg',
    'vision_uart_pkg',
}
EXPECTED_SOURCE_PACKAGES = EXPECTED_ROS_PACKAGES | {'unitree_lidar_sdk'}

PYTHON_IMPORT_DEPENDENCIES = {
    'geometry_msgs': 'geometry_msgs',
    'nav_msgs': 'nav_msgs',
    'rclpy': 'rclpy',
    'sensor_msgs': 'sensor_msgs',
    'serial': 'python3-serial',
    'std_msgs': 'std_msgs',
    'tf2_ros': 'tf2_ros',
}

MANIFEST_DEPENDENCY_TAGS = {
    'build_depend',
    'build_export_depend',
    'buildtool_depend',
    'depend',
    'exec_depend',
}

errors: list[str] = []


def require(condition: bool, message: str) -> None:
    if not condition:
        errors.append(message)


def parse_python(path: Path) -> ast.Module | None:
    try:
        return ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    except (OSError, SyntaxError) as exc:
        errors.append(f'{path.relative_to(REPOSITORY_ROOT)}: Python parse failed: {exc}')
        return None


def manifest_dependencies(root: ET.Element) -> set[str]:
    return {
        element.text.strip()
        for element in root
        if element.tag in MANIFEST_DEPENDENCY_TAGS and element.text
    }


def imported_roots(tree: ast.Module) -> set[str]:
    imports: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports.update(alias.name.split('.', 1)[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imports.add(node.module.split('.', 1)[0])
    return imports


def setup_call(tree: ast.Module, path: Path) -> ast.Call | None:
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == 'setup':
            return node
    errors.append(f'{path.relative_to(REPOSITORY_ROOT)}: no setup(...) call found')
    return None


def validate_console_scripts(package_dir: Path, package_name: str, tree: ast.Module) -> None:
    call = setup_call(tree, package_dir / 'setup.py')
    if call is None:
        return

    entry_points_node = next(
        (keyword.value for keyword in call.keywords if keyword.arg == 'entry_points'),
        None,
    )
    if entry_points_node is None:
        errors.append(f'{package_name}: setup.py has no entry_points')
        return

    try:
        entry_points = ast.literal_eval(entry_points_node)
    except (ValueError, TypeError) as exc:
        errors.append(f'{package_name}: cannot statically read entry_points: {exc}')
        return

    console_scripts = entry_points.get('console_scripts', [])
    require(bool(console_scripts), f'{package_name}: no console_scripts are declared')

    for entry in console_scripts:
        if '=' not in entry or ':' not in entry:
            errors.append(f'{package_name}: invalid console entry point {entry!r}')
            continue
        _, target = (part.strip() for part in entry.split('=', 1))
        module_name, function_name = (part.strip() for part in target.split(':', 1))
        module_path = package_dir / (module_name.replace('.', '/') + '.py')
        require(module_path.is_file(), f'{package_name}: entry-point module is missing: {module_name}')
        if not module_path.is_file():
            continue
        module_tree = parse_python(module_path)
        if module_tree is None:
            continue
        functions = {
            node.name
            for node in module_tree.body
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        }
        require(
            function_name in functions,
            f'{package_name}: entry-point function is missing: {target}',
        )


def validate_ament_python(package_dir: Path, package_name: str, dependencies: set[str]) -> None:
    module_dir = package_dir / package_name
    resource_marker = package_dir / 'resource' / package_name
    setup_path = package_dir / 'setup.py'
    setup_cfg_path = package_dir / 'setup.cfg'

    require(
        'ament_python' in dependencies,
        f'{package_name}: ament_python buildtool dependency is missing',
    )
    require(module_dir.is_dir(), f'{package_name}: Python module directory is missing')
    require((module_dir / '__init__.py').is_file(), f'{package_name}: __init__.py is missing')
    require(resource_marker.is_file(), f'{package_name}: ament resource marker is missing')
    require(setup_path.is_file(), f'{package_name}: setup.py is missing')
    require(setup_cfg_path.is_file(), f'{package_name}: setup.cfg is missing')

    if setup_path.is_file():
        setup_text = setup_path.read_text(encoding='utf-8')
        require(
            'share/ament_index/resource_index/packages' in setup_text,
            f'{package_name}: setup.py does not install the ament resource marker',
        )
        require(
            "['package.xml']" in setup_text,
            f'{package_name}: setup.py does not install package.xml',
        )
        tree = parse_python(setup_path)
        if tree is not None:
            assigned_name = next(
                (
                    node.value.value
                    for node in tree.body
                    if isinstance(node, ast.Assign)
                    and any(isinstance(target, ast.Name) and target.id == 'package_name' for target in node.targets)
                    and isinstance(node.value, ast.Constant)
                    and isinstance(node.value.value, str)
                ),
                None,
            )
            require(
                assigned_name == package_name,
                f'{package_name}: setup.py package_name is {assigned_name!r}',
            )
            validate_console_scripts(package_dir, package_name, tree)

    if setup_cfg_path.is_file():
        config = configparser.ConfigParser(interpolation=None)
        try:
            config.read(setup_cfg_path, encoding='utf-8')
            expected_path = f'$base/lib/{package_name}'
            require(
                config.get('develop', 'script_dir', fallback=None) == expected_path,
                f'{package_name}: setup.cfg develop.script_dir must be {expected_path}',
            )
            require(
                config.get('install', 'install_scripts', fallback=None) == expected_path,
                f'{package_name}: setup.cfg install.install_scripts must be {expected_path}',
            )
        except configparser.Error as exc:
            errors.append(f'{package_name}: setup.cfg parse failed: {exc}')

    for source_path in sorted(module_dir.rglob('*.py')):
        tree = parse_python(source_path)
        if tree is None:
            continue
        for imported_root in sorted(imported_roots(tree)):
            dependency = PYTHON_IMPORT_DEPENDENCIES.get(imported_root)
            if dependency:
                require(
                    dependency in dependencies,
                    f'{package_name}: import {imported_root!r} requires manifest dependency {dependency!r}',
                )


def validate_unitree_package(
    package_dir: Path,
    manifest_root: ET.Element,
    dependencies: set[str],
) -> None:
    cmake_path = package_dir / 'CMakeLists.txt'
    launch_path = package_dir / 'launch' / 'launch.py'
    rviz_path = package_dir / 'rviz' / 'view.rviz'
    cmake = cmake_path.read_text(encoding='utf-8') if cmake_path.is_file() else ''

    required_dependencies = {
        'ament_index_python',
        'geometry_msgs',
        'launch',
        'launch_ros',
        'libpcl-all-dev',
        'pcl_conversions',
        'rclcpp',
        'rviz2',
        'sensor_msgs',
        'std_msgs',
        'tf2',
        'tf2_ros',
    }
    for dependency in sorted(required_dependencies):
        require(dependency in dependencies, f'unitree_lidar_ros2: missing dependency {dependency!r}')

    require(
        manifest_root.findtext('buildtool_depend') == 'ament_cmake',
        'unitree_lidar_ros2: ament_cmake buildtool dependency is missing',
    )
    require(cmake_path.is_file(), 'unitree_lidar_ros2: CMakeLists.txt is missing')
    require('../../../unitree_lidar_sdk' not in cmake, 'unitree_lidar_ros2: fragile SDK path remains')
    for token in (
        'UNITREE_SDK_ROOT',
        'UNITREE_SDK_SUPPORTED_ARCHITECTURES aarch64 x86_64',
        'CMAKE_SYSTEM_NAME',
        'CMAKE_SYSTEM_PROCESSOR',
        'libunilidar_sdk2.a',
        'unitree_sdk_library.txt',
        'message(FATAL_ERROR',
        'install(DIRECTORY launch',
        'install(DIRECTORY config',
        'install(DIRECTORY rviz',
    ):
        require(token in cmake, f'unitree_lidar_ros2: CMake token is missing: {token}')

    for cmake_dependency in (
        'PCL',
        'Threads',
        'ament_cmake',
        'geometry_msgs',
        'pcl_conversions',
        'rclcpp',
        'sensor_msgs',
        'std_msgs',
        'tf2',
        'tf2_ros',
    ):
        require(
            f'find_package({cmake_dependency} REQUIRED)' in cmake,
            f'unitree_lidar_ros2: find_package({cmake_dependency} REQUIRED) is missing',
        )

    require(launch_path.is_file(), 'unitree_lidar_ros2: launch/launch.py is missing')
    require(rviz_path.is_file(), 'unitree_lidar_ros2: rviz/view.rviz is missing')
    if launch_path.is_file():
        launch_text = launch_path.read_text(encoding='utf-8')
        parse_python(launch_path)
        require(
            'get_package_share_directory' in launch_text,
            'unitree_lidar_ros2: launch file does not use the ament package share directory',
        )
        require(
            "'config', 'driver.defaults.yaml'" in launch_text,
            'unitree_lidar_ros2: launch file does not use the installed driver config path',
        )
        require(
            "'rviz', 'view.rviz'" in launch_text,
            'unitree_lidar_ros2: launch file does not use the conventional RViz install path',
        )
        require(
            'subprocess' not in launch_text,
            'unitree_lidar_ros2: launch resource lookup still shells out to ros2',
        )

    sdk_root = SOURCE_ROOT / 'unitree_lidar_sdk'
    archives = {
        path.parent.name: path
        for path in sorted((sdk_root / 'lib').glob('*/libunilidar_sdk2.a'))
    }
    require(
        set(archives) == {'aarch64', 'x86_64'},
        f'unitree_lidar_sdk: expected Linux archives for aarch64 and x86_64, found {sorted(archives)}',
    )
    for architecture, archive in archives.items():
        try:
            archive_header = archive.read_bytes()[:8]
        except OSError as exc:
            errors.append(f'unitree_lidar_sdk: cannot read {architecture} archive: {exc}')
            continue
        require(
            archive_header == b'!<arch>\n',
            f'unitree_lidar_sdk: {architecture} library is not an ar archive',
        )

    sdk_cmake_path = sdk_root / 'CMakeLists.txt'
    require(sdk_cmake_path.is_file(), 'unitree_lidar_sdk: CMakeLists.txt is missing')
    if sdk_cmake_path.is_file():
        sdk_cmake = sdk_cmake_path.read_text(encoding='utf-8')
        for token in (
            'UNITREE_SDK_SUPPORTED_ARCHITECTURES aarch64 x86_64',
            'CMAKE_SYSTEM_NAME',
            'CMAKE_SYSTEM_PROCESSOR',
            'find_package(Threads REQUIRED)',
            'install(TARGETS ${UNITREE_SDK_EXAMPLE_TARGETS}',
            'install(DIRECTORY include/',
            'install(FILES "${UNITREE_SDK_LIBRARY}"',
            'message(FATAL_ERROR',
        ):
            require(token in sdk_cmake, f'unitree_lidar_sdk: CMake token is missing: {token}')


def validate_repository_scripts() -> None:
    script_requirements = {
        REPOSITORY_ROOT / 'build_ws.sh': (
            '/opt/ros/${UGV_ROS_DISTRO}/setup.bash',
            'UGV_ROS_DISTRO="humble"',
            'UGV_CLEAN_BUILD',
            'colcon build --symlink-install',
        ),
        REPOSITORY_ROOT / 'scripts' / 'verify_build.sh': (
            '/opt/ros/${UGV_ROS_DISTRO}/setup.bash',
            'UGV_ROS_DISTRO="humble"',
            'colcon list --base-paths',
            'lib/aarch64/libunilidar_sdk2.a',
            'unitree_sdk_library.txt',
            'colcon test-result --verbose',
        ),
        REPOSITORY_ROOT / 'scripts' / 'verify_unitree_phase2.sh': (
            '/opt/ros/${UGV_ROS_DISTRO}/setup.bash',
            'UGV_ROS_DISTRO="humble"',
            'UGV_PHASE2_OUTPUT_DIR',
            'colcon --log-base',
            '--packages-select "${UGV_UNITREE_PACKAGE}"',
            'lib/aarch64/libunilidar_sdk2.a',
            'unitree_sdk_library.txt',
            'colcon test-result',
            'ros2 pkg executables',
            'initialize_type:=0',
            'launch.py --show-args',
        ),
        REPOSITORY_ROOT / 'run_lidar.sh': (
            '/opt/ros/${UGV_ROS_DISTRO}/setup.bash',
            'UGV_ROS_DISTRO="humble"',
            'ros2 launch unitree_lidar_ros2 launch.py',
        ),
    }

    for script_path, required_tokens in script_requirements.items():
        require(script_path.is_file(), f'{script_path.relative_to(REPOSITORY_ROOT)} is missing')
        if not script_path.is_file():
            continue
        script_text = script_path.read_text(encoding='utf-8')
        # The Jazzy migration is deferred; active scripts must not source its setup path.
        require(
            '/opt/ros/jazzy' not in script_text,
            f'{script_path.relative_to(REPOSITORY_ROOT)} still requires the deferred ROS 2 Jazzy target',
        )
        for token in required_tokens:
            require(
                token in script_text,
                f'{script_path.relative_to(REPOSITORY_ROOT)}: required token is missing: {token}',
            )


def main() -> int:
    manifest_paths = sorted(SOURCE_ROOT.glob('*/package.xml'))
    discovered_ros_packages: set[str] = set()
    discovered_source_packages = {
        path.name
        for path in SOURCE_ROOT.iterdir()
        if path.is_dir() and ((path / 'package.xml').is_file() or (path / 'CMakeLists.txt').is_file())
    }

    for manifest_path in manifest_paths:
        package_dir = manifest_path.parent
        try:
            manifest_root = ET.parse(manifest_path).getroot()
        except (OSError, ET.ParseError) as exc:
            errors.append(f'{manifest_path.relative_to(REPOSITORY_ROOT)}: XML parse failed: {exc}')
            continue

        package_name = (manifest_root.findtext('name') or '').strip()
        discovered_ros_packages.add(package_name)
        require(package_name == package_dir.name, f'{package_dir.name}: manifest name is {package_name!r}')

        build_type = (manifest_root.findtext('./export/build_type') or '').strip()
        dependencies = manifest_dependencies(manifest_root)
        if build_type == 'ament_python':
            validate_ament_python(package_dir, package_name, dependencies)
        elif build_type == 'ament_cmake' and package_name == 'unitree_lidar_ros2':
            validate_unitree_package(package_dir, manifest_root, dependencies)
        else:
            errors.append(f'{package_name}: unsupported or missing build type {build_type!r}')

    require(
        EXPECTED_ROS_PACKAGES == discovered_ros_packages,
        'ROS package set differs from expected: '
        f'missing={sorted(EXPECTED_ROS_PACKAGES - discovered_ros_packages)}, '
        f'unexpected={sorted(discovered_ros_packages - EXPECTED_ROS_PACKAGES)}',
    )
    require(
        EXPECTED_SOURCE_PACKAGES == discovered_source_packages,
        'source package set differs from recovered eleven-package baseline: '
        f'missing={sorted(EXPECTED_SOURCE_PACKAGES - discovered_source_packages)}, '
        f'unexpected={sorted(discovered_source_packages - EXPECTED_SOURCE_PACKAGES)}',
    )

    for python_path in sorted(SOURCE_ROOT.rglob('*.py')):
        parse_python(python_path)

    validate_repository_scripts()

    if errors:
        print(f'FAIL: {len(errors)} static package validation error(s)', file=sys.stderr)
        for error in errors:
            print(f'  - {error}', file=sys.stderr)
        return 1

    print(
        'PASS: validated '
        f'{len(discovered_ros_packages)} ROS manifests, '
        f'{len(discovered_source_packages)} source packages, Python entry points/import dependencies, '
        'Humble build-script defaults, Unitree SDK architecture/install assets, '
        'and installed launch/config/RViz resource paths.'
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
