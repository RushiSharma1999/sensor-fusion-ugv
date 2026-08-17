#!/usr/bin/env python3
"""Validate the Unitree driver, launch, TF, and vendor-archive contract statically."""

from __future__ import annotations

import ast
import hashlib
import json
import re
import sys
from pathlib import Path
from typing import Any


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPOSITORY_ROOT / 'ws' / 'src' / 'unitree_lidar_ros2'
SDK_ROOT = REPOSITORY_ROOT / 'ws' / 'src' / 'unitree_lidar_sdk'
LAUNCH_PATH = PACKAGE_ROOT / 'launch' / 'launch.py'
CONFIG_PATH = PACKAGE_ROOT / 'config' / 'driver.defaults.yaml'
SOURCE_PATH = PACKAGE_ROOT / 'src' / 'unitree_lidar_ros2_node.cpp'
HEADER_PATH = PACKAGE_ROOT / 'include' / 'unitree_lidar_ros2.h'

EXPECTED_PARAMETERS: dict[str, Any] = {
    'initialize_type': 2,
    'work_mode': 0,
    'use_system_timestamp': True,
    'range_min': 0.0,
    'range_max': 100.0,
    'cloud_scan_num': 18,
    'serial_port': '/dev/ttyACM0',
    'baudrate': 4000000,
    'lidar_port': 6101,
    'lidar_ip': '192.168.1.62',
    'local_port': 6201,
    'local_ip': '192.168.1.2',
    'cloud_frame': 'unilidar_lidar',
    'cloud_topic': 'unilidar/cloud',
    'imu_frame': 'unilidar_imu',
    'imu_topic': 'unilidar/imu',
    'initial_imu_frame': 'unilidar_imu_initial',
    'publish_imu_tf': True,
    'publish_imu_to_lidar_tf': True,
    'publish_robot_to_imu_tf': False,
    'robot_frame': '',
    'imu_to_lidar.translation.x': 0.007698,
    'imu_to_lidar.translation.y': 0.014655,
    'imu_to_lidar.translation.z': -0.00667,
    'imu_to_lidar.rotation.x': 0.0,
    'imu_to_lidar.rotation.y': 0.0,
    'imu_to_lidar.rotation.z': 0.0,
    'imu_to_lidar.rotation.w': 1.0,
    'qos_depth': 10,
    'poll_interval_ms': 1,
}

REQUIRED_WHEN_ROBOT_TF_ENABLED = {
    'robot_to_imu.translation.x',
    'robot_to_imu.translation.y',
    'robot_to_imu.translation.z',
    'robot_to_imu.rotation.x',
    'robot_to_imu.rotation.y',
    'robot_to_imu.rotation.z',
    'robot_to_imu.rotation.w',
}

EXPECTED_LAUNCH_DEFAULTS = {
    'namespace': '',
    'cloud_topic': '',
    'imu_topic': '',
    'cloud_frame': '',
    'imu_frame': '',
    'initial_imu_frame': '',
    'use_rviz': 'false',
}

EXPECTED_WORK_MODE_BITS = {
    'kWorkModeWideAngleFovBit': 0,
    'kWorkMode2dMeasurementBit': 1,
    'kWorkModeDisableImuBit': 2,
    'kWorkModeSerialTransportBit': 3,
    'kWorkModeWaitForStartBit': 4,
}

EXPECTED_VENDOR_HASHES = {
    'aarch64': '4e334b67c1a92152c89363d8014a6e361d7bf590e58484d7d6ddc8541389de28',
    'x86_64': 'a8530545cbd435e3a96b9a099cd57921202aed3d690066f55ea0ade4310212d3',
}

errors: list[str] = []


def require(condition: bool, message: str) -> None:
    if not condition:
        errors.append(message)


def load_json_parameter_file() -> dict[str, Any]:
    try:
        document = json.loads(CONFIG_PATH.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        errors.append(f'{CONFIG_PATH.relative_to(REPOSITORY_ROOT)}: parse failed: {exc}')
        return {}

    require(set(document) == {'/**'}, 'Unitree parameter file must use the /** ROS wildcard key')
    wildcard = document.get('/**', {})
    require(
        isinstance(wildcard, dict) and set(wildcard) == {'ros__parameters'},
        'Unitree parameter file must contain only /**.ros__parameters',
    )
    parameters = wildcard.get('ros__parameters', {}) if isinstance(wildcard, dict) else {}
    require(isinstance(parameters, dict), 'Unitree ros__parameters must be a mapping')
    return parameters if isinstance(parameters, dict) else {}


def literal_value(node: ast.AST) -> Any:
    try:
        return ast.literal_eval(node)
    except (ValueError, TypeError):
        return None


def parse_launch_contract() -> None:
    try:
        launch_tree = ast.parse(LAUNCH_PATH.read_text(encoding='utf-8'), filename=str(LAUNCH_PATH))
    except (OSError, SyntaxError) as exc:
        errors.append(f'{LAUNCH_PATH.relative_to(REPOSITORY_ROOT)}: parse failed: {exc}')
        return

    declarations: dict[str, Any] = {}
    node_calls: list[dict[str, Any]] = []
    for node in ast.walk(launch_tree):
        if not isinstance(node, ast.Call):
            continue
        function_name = node.func.id if isinstance(node.func, ast.Name) else None
        if function_name == 'DeclareLaunchArgument' and node.args:
            argument_name = literal_value(node.args[0])
            default_node = next(
                (keyword.value for keyword in node.keywords if keyword.arg == 'default_value'),
                None,
            )
            if isinstance(argument_name, str):
                declarations[argument_name] = literal_value(default_node) if default_node else None
        elif function_name == 'Node':
            node_calls.append(
                {
                    keyword.arg: literal_value(keyword.value)
                    for keyword in node.keywords
                    if keyword.arg in {'package', 'executable', 'name'}
                }
            )

    expected_argument_names = set(EXPECTED_LAUNCH_DEFAULTS) | {'config_file', 'rviz_config'}
    require(
        set(declarations) == expected_argument_names,
        f'launch arguments differ: expected {sorted(expected_argument_names)}, found {sorted(declarations)}',
    )
    for name, expected_default in EXPECTED_LAUNCH_DEFAULTS.items():
        require(
            declarations.get(name) == expected_default,
            f'launch argument {name!r} default must be {expected_default!r}',
        )

    require(
        {
            'package': 'unitree_lidar_ros2',
            'executable': 'unitree_lidar_ros2_node',
            'name': 'unitree_lidar_ros2_node',
        } in node_calls,
        'launch does not construct the expected Unitree driver node',
    )
    require(
        {'package': 'rviz2', 'executable': 'rviz2', 'name': 'rviz2'} in node_calls,
        'launch does not retain the optional RViz node',
    )

    launch_text = LAUNCH_PATH.read_text(encoding='utf-8')
    for required_token in (
        'get_package_share_directory',
        "'config', 'driver.defaults.yaml'",
        "'rviz', 'view.rviz'",
        'OpaqueFunction(function=_launch_setup)',
        "IfCondition(LaunchConfiguration('use_rviz')).evaluate(context)",
        "on_exit=[Shutdown(reason='Unitree LiDAR driver exited')]",
        "_require_file(config_file, 'Unitree driver parameter file')",
        'driver_parameters = [config_file]',
        'driver_parameters.append(parameter_overrides)',
        'parameters=driver_parameters',
    ):
        require(required_token in launch_text, f'launch contract token is missing: {required_token}')
    require(
        launch_text.count('namespace=namespace') == 2,
        'driver and optional RViz must share the configured namespace',
    )

    override_function = next(
        (
            node
            for node in launch_tree.body
            if isinstance(node, ast.FunctionDef) and node.name == '_explicit_parameter_overrides'
        ),
        None,
    )
    require(
        override_function is not None,
        'launch helper _explicit_parameter_overrides is missing',
    )
    if override_function is not None:
        namespace: dict[str, Any] = {}
        helper_module = ast.Module(body=[override_function], type_ignores=[])
        exec(compile(helper_module, str(LAUNCH_PATH), 'exec'), namespace)
        helper = namespace['_explicit_parameter_overrides']
        require(
            helper({'cloud_topic': '', 'imu_frame': ''}) == {},
            'empty launch values must leave config_file parameters authoritative',
        )
        require(
            helper({'cloud_topic': 'custom/cloud', 'imu_frame': ''}) == {
                'cloud_topic': 'custom/cloud'
            },
            'an explicit non-empty launch value must override only its matching parameter',
        )


PARAMETER_DECLARATION = re.compile(
    r'declare_parameter<(?P<type>bool|int|double|std::string)>\('
    r'\s*"(?P<name>[^"]+)"\s*,\s*'
    r'(?P<default>true|false|"[^"]*"|[-+]?(?:\d+(?:\.\d*)?|\.\d+))\s*\)',
    re.MULTILINE,
)


def parse_cpp_default(type_name: str, token: str) -> Any:
    if type_name == 'bool':
        return token == 'true'
    if type_name == 'int':
        return int(token)
    if type_name == 'double':
        return float(token)
    return ast.literal_eval(token)


def parse_source_contract(parameters: dict[str, Any]) -> None:
    try:
        source = SOURCE_PATH.read_text(encoding='utf-8')
        header = HEADER_PATH.read_text(encoding='utf-8')
    except OSError as exc:
        errors.append(f'Unitree C++ source read failed: {exc}')
        return

    declared_parameters = {
        match.group('name'): parse_cpp_default(match.group('type'), match.group('default'))
        for match in PARAMETER_DECLARATION.finditer(source)
    }
    required_parameters = set(
        re.findall(
            r'declare_parameter<double>\("(robot_to_imu\.[^"]+)"\);',
            source,
        )
    )
    require(
        declared_parameters == EXPECTED_PARAMETERS,
        'C++ declared parameter names/defaults differ from the Phase 2 contract',
    )
    require(
        parameters == EXPECTED_PARAMETERS,
        'installed parameter-file names/defaults differ from the Phase 2 contract',
    )
    require(
        declared_parameters == parameters,
        'C++ defaults and installed parameter-file defaults differ',
    )
    require(
        required_parameters == REQUIRED_WHEN_ROBOT_TF_ENABLED,
        'measured robot_to_imu parameters must all be declared without fabricated defaults',
    )

    work_mode_bits = {
        name: int(bit)
        for name, bit in re.findall(
            r'constexpr std::uint32_t (kWorkMode\w+Bit) = 1U << ([0-4])U;',
            source,
        )
    }
    require(
        work_mode_bits == EXPECTED_WORK_MODE_BITS,
        f'SDK2 work-mode bit definitions differ: {work_mode_bits}',
    )

    structured_invariants = {
        'parameters validated before SDK initialization':
            source.find('  validate_parameters();') < source.find('  initialize_driver();'),
        'one IMU getter call': source.count('getImuData(imu)') == 1,
        'initialization return checked': 'if (initialization_result != 0)' in source,
        'serial close implemented': 'lidar_reader_->closeSerial()' in source,
        'UDP close implemented': 'lidar_reader_->closeUDP()' in source,
        'sensor-data QoS used': 'rclcpp::SensorDataQoS' in source,
        'static TF broadcaster used': 'tf2_ros::StaticTransformBroadcaster' in header,
        'dynamic TF broadcaster used': 'tf2_ros::TransformBroadcaster' in header,
        'duplicate TF parents rejected':
            'publish_robot_to_imu_tf and publish_imu_tf cannot both be true' in source,
        'measured robot extrinsic required':
            required_parameters == REQUIRED_WHEN_ROBOT_TF_ENABLED,
        'IMU and dynamic TF share stamp': 'transform.header.stamp = message.header.stamp;' in source,
        'IMU and dynamic TF share quaternion': 'transform.transform.rotation = quaternion;' in source,
        'cloud frame assigned': 'message.header.frame_id = cloud_frame_;' in source,
        'IMU frame assigned': 'message.header.frame_id = imu_frame_;' in source,
        'PointCloud2 dimensions explicit': 'pcl_cloud->height = 1U;' in source,
        'invalid quaternion rejected': 'norm < kMinimumQuaternionNorm' in source,
        'reserved work-mode bits rejected':
            'work_mode bits 5 through 31 are reserved and must be zero' in source,
        '2D work mode rejected': 'work_mode 2D measurement mode is unsupported' in source,
        'UDP work-mode transport checked':
            'initialize_type=2 (UDP) requires work_mode bit 3 to select Ethernet (0)' in source,
        'serial work-mode transport checked':
            'initialize_type=1 (serial) requires work_mode bit 3 to select serial (1)' in source,
        'wait-for-start work mode rejected':
            'work_mode bit 4 (wait for start command) is unsupported' in source and
            'startLidarRotation(' not in source,
        'IMU-disabled dynamic TF rejected':
            'work_mode disables IMU data, so publish_imu_tf must be false' in source,
        'unexpected disabled IMU packets ignored':
            'Ignoring an unexpected IMU packet because work_mode disables IMU output' in source,
        'hardware timestamps rejected':
            'use_system_timestamp must be true; hardware-clock to ROS-time conversion is not implemented'
            in source,
        'no LaserScan implementation': 'LaserScan' not in source and '/scan' not in source,
    }
    for description, condition in structured_invariants.items():
        require(condition, f'C++ contract failed: {description}')

    quaternion_indices = {
        axis: int(index)
        for axis, index in re.findall(
            r'const double ([xyzw]) = imu\.quaternion\[(\d)\];',
            source,
        )
    }
    require(
        quaternion_indices == {'x': 0, 'y': 1, 'z': 2, 'w': 3},
        f'IMU quaternion order is not x,y,z,w: {quaternion_indices}',
    )


def validate_build_and_resources() -> None:
    cmake = (PACKAGE_ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')
    for token in (
        'UNITREE_SDK_SUPPORTED_ARCHITECTURES aarch64 x86_64',
        'CMAKE_SYSTEM_NAME STREQUAL "Linux"',
        'unitree_sdk_library.txt',
        'unitree_lidar_sdk',
        'Threads::Threads',
        'install(DIRECTORY launch',
        'install(DIRECTORY config',
        'install(DIRECTORY rviz',
    ):
        require(token in cmake, f'Unitree CMake contract token is missing: {token}')

    run_script = (REPOSITORY_ROOT / 'run_lidar.sh').read_text(encoding='utf-8')
    require(
        'ros2 launch unitree_lidar_ros2 launch.py "$@"' in run_script,
        'run_lidar.sh must preserve launch argument pass-through',
    )

    rviz = (PACKAGE_ROOT / 'rviz' / 'view.rviz').read_text(encoding='utf-8')
    require('Value: unilidar/cloud' in rviz, 'RViz cloud topic must be namespace-relative')
    require('Reliability Policy: Best Effort' in rviz, 'RViz does not match sensor-data QoS')

    contract = REPOSITORY_ROOT / 'docs' / 'interfaces' / 'unitree_driver_tf_contract.md'
    require(contract.is_file(), 'Unitree driver/TF contract document is missing')
    if contract.is_file():
        contract_text = contract.read_text(encoding='utf-8')
        for statement in (
            'YAML configuration is authoritative',
            'physical orientation semantics remain unverified',
            'SDK2 software lineage targets the Unitree L2 family',
        ):
            require(statement in contract_text, f'contract statement is missing: {statement}')

    sdk_config = (SDK_ROOT / 'include' / 'unitree_lidar_sdk_config.h').read_text(
        encoding='utf-8'
    )
    require(
        '#define unitree_lidar_sdk_VERSION "2.0.9"' in sdk_config,
        'checked-in Unitree SDK version changed from 2.0.9',
    )


def validate_vendor_archives() -> None:
    archives = {
        path.parent.name: path
        for path in sorted((SDK_ROOT / 'lib').glob('*/libunilidar_sdk2.a'))
    }
    require(set(archives) == set(EXPECTED_VENDOR_HASHES), 'Unitree vendor archive set changed')
    for architecture, expected_hash in EXPECTED_VENDOR_HASHES.items():
        archive = archives.get(architecture)
        if archive is None:
            continue
        try:
            data = archive.read_bytes()
        except OSError as exc:
            errors.append(f'cannot read Unitree {architecture} archive: {exc}')
            continue
        require(data.startswith(b'!<arch>\n'), f'Unitree {architecture} library is not an ar archive')
        actual_hash = hashlib.sha256(data).hexdigest()
        require(
            actual_hash == expected_hash,
            f'Unitree {architecture} vendor archive hash changed: {actual_hash}',
        )


def main() -> int:
    parameters = load_json_parameter_file()
    parse_launch_contract()
    parse_source_contract(parameters)
    validate_build_and_resources()
    validate_vendor_archives()

    if errors:
        print(f'FAIL: {len(errors)} Unitree Phase 2 contract error(s)', file=sys.stderr)
        for error in errors:
            print(f'  - {error}', file=sys.stderr)
        return 1

    print(
        'PASS: validated 30 Unitree defaults plus 7 measured-only TF parameters, '
        'SDK2 work-mode/timestamp restrictions, YAML-authoritative launch precedence, '
        'headless/optional-RViz launch, '
        'topic/frame/TF/QoS source invariants, installed resources, SDK architecture selection, '
        'and unchanged aarch64/x86_64 vendor archives.'
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
