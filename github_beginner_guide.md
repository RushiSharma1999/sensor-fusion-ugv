# Beginner GitHub Guide for the Sensor Fusion UGV Repository

This guide explains how to create a GitHub account, get access to the project repository, download the code using GitHub Desktop, make changes, and upload those changes back to GitHub.

This guide is written for teammates who have never used GitHub before.

---

## 1. What is GitHub?

GitHub is a website used to store code and collaborate on projects.

For our project, GitHub will let us:

- Store the latest version of the ROS 2 UGV code
- Share code between teammates
- Track changes to files
- Recover older versions if something breaks
- Let multiple people work on the project without emailing zip files back and forth

---

## 2. Create a GitHub Account

1. Go to GitHub:
   - https://github.com

2. Click **Sign up**.

3. Create an account using your email address.

4. Choose a username.

5. Verify your email address.

6. After your account is created, send your GitHub username to the person managing the repository.

Example message:

```text
My GitHub username is: your_username_here
```

---

## 3. Get Access to the Repository

The project owner needs to invite you to the repository.

There are two common ways this may happen:

1. You may be invited directly to the repository as a collaborator.
2. You may be invited to a GitHub organization that owns the repository.

When you are invited, GitHub will send you an email with an invitation link. Click the link and accept the invitation.

After accepting the invitation, you should be able to open the repository in your browser.

---

## 4. Install GitHub Desktop

GitHub Desktop is the easiest way to use GitHub without the terminal.

1. Go to:
   - https://desktop.github.com

2. Download GitHub Desktop.

3. Install it.

4. Open GitHub Desktop.

5. Sign in using your GitHub account.

GitHub Desktop lets you clone repositories, commit changes, push changes, pull changes, and create pull requests using a graphical interface.

---

## 5. Clone the Repository Using GitHub Desktop

Cloning means downloading a copy of the GitHub repository onto your computer.

1. Open GitHub Desktop.

2. Click:

```text
File -> Clone Repository
```

3. Select the repository from the **GitHub.com** tab.

4. Choose where you want the folder saved on your computer.

Recommended location:

```text
Documents/GitHub/sensor-fusion-ugv
```

5. Click **Clone**.

You now have the repository on your computer.

---

## 6. Open the Repository Folder

In GitHub Desktop:

1. Click **Repository** in the top menu.

2. Click **Show in Finder** on Mac or **Show in Explorer** on Windows.

You should see a folder similar to this:

```text
sensor-fusion-ugv/
├── README.md
├── build_ws.sh
├── run_lidar.sh
└── ws/
    └── src/
        ├── esp_bridge/
        ├── fusion_pkg/
        ├── gap_follow/
        ├── ultrasonic_pkg/
        ├── vision_uart_pkg/
        └── ...
```

---

## 7. Basic GitHub Desktop Workflow

Whenever you work on the project, use this order:

```text
Pull -> Edit files -> Commit -> Push
```

Each step is explained below.

---

## 8. Pull Before Making Changes

Before editing anything, open GitHub Desktop and click:

```text
Fetch origin
```

If GitHub Desktop shows new changes, click:

```text
Pull origin
```

This downloads the newest version of the repository.

Always pull before editing so you do not accidentally work on an old version of the code.

---

## 9. Make Changes to the Code

Open the repository folder in your code editor.

Recommended editor:

```text
Visual Studio Code
```

You can edit Python files, launch files, README files, scripts, or documentation.

Example files you might edit:

```text
ws/src/fusion_pkg/fusion_pkg/fusion_node.py
ws/src/gap_follow/gap_follow/gap_follow_node.py
ws/src/esp_bridge/esp_bridge/cmd_vel_to_serial.py
README.md
```

---

## 10. Commit Your Changes

A commit is a saved checkpoint of your changes.

After editing files:

1. Open GitHub Desktop.

2. Look at the left side. You should see the files you changed.

3. Review the changes.

4. At the bottom-left, write a short summary.

Good commit messages:

```text
Fix serial port timeout in ESP bridge
```

```text
Update LiDAR launch instructions
```

```text
Add safety mux node documentation
```

Bad commit messages:

```text
stuff
```

```text
changes
```

```text
asdf
```

5. Click:

```text
Commit to main
```

or, if you are on a different branch:

```text
Commit to your-branch-name
```

The commit saves your changes locally on your computer.

---

## 11. Push Your Changes to GitHub

After committing, click:

```text
Push origin
```

This uploads your committed changes to GitHub.

If you do not push, your changes only exist on your computer.

---

## 12. Pull Again After Someone Else Changes Code

If another teammate uploads changes, click:

```text
Fetch origin
```

Then click:

```text
Pull origin
```

This updates your local copy.

Do this often.

---

## 13. Recommended Team Workflow

For beginners, the simplest workflow is:

1. Open GitHub Desktop.
2. Click **Fetch origin**.
3. Click **Pull origin** if available.
4. Make your changes.
5. Commit your changes.
6. Click **Push origin**.

Avoid editing the same file at the same time as another teammate when possible.

---

## 14. Safer Workflow: Use Branches

A branch is a separate workspace where you can make changes without directly affecting the main code.

Use branches when working on bigger changes.

Examples:

```text
camera-uart-fix
```

```text
lidar-filter-update
```

```text
readme-cleanup
```

### Create a branch in GitHub Desktop

1. Open GitHub Desktop.

2. At the top, click the current branch name.

3. Click **New Branch**.

4. Name the branch based on what you are working on.

Example:

```text
gap-follow-tuning
```

5. Click **Create Branch**.

6. Make your changes.

7. Commit your changes.

8. Push your branch.

9. Open a Pull Request on GitHub.

---

## 15. What is a Pull Request?

A Pull Request, also called a PR, is a request to merge your branch into the main project.

Use a Pull Request when you want someone to review your changes before they go into the main code.

Pull Requests are useful because they allow teammates to:

- Review code before it becomes part of the main project
- Comment on changes
- Catch mistakes
- Keep the main branch stable

---

## 16. Create a Pull Request

After pushing your branch:

1. Go to the repository on GitHub.com.

2. GitHub may show a button that says:

```text
Compare & pull request
```

3. Click it.

4. Add a short title.

Example:

```text
Update gap follow obstacle avoidance tuning
```

5. Add a description of what changed.

Example:

```text
This pull request updates the gap-following node to improve obstacle avoidance around close-range objects.
```

6. Click **Create pull request**.

Ask a teammate to review it before merging.

---

## 17. Do Not Commit These Files

Do not upload generated or temporary files.

Avoid committing:

```text
ws/build/
ws/install/
ws/log/
__pycache__/
*.pyc
.DS_Store
```

These files are generated automatically and do not need to be stored in GitHub.

The `.gitignore` file should already prevent most of these from being added.

---

## 18. What to Do Before Pushing

Before clicking **Push origin**, check:

- Did I pull the latest version first?
- Did I only change files I meant to change?
- Did I avoid committing build, install, log, or cache files?
- Did I write a clear commit message?
- Did I test the code if I changed something important?

---

## 19. How to Run the Code on the Raspberry Pi

The Phase 1 compatibility baseline is Raspberry Pi 4 with Ubuntu 22.04 ARM64 and ROS 2 Humble. Follow the root `README.md` to install prerequisites and resolve rosdep dependencies before building. Pi 5 and ROS 2 Jazzy remain unverified.

After cloning the repository on the Pi:

```bash
cd sensor-fusion-ugv
chmod +x build_ws.sh run_lidar.sh
./build_ws.sh --clean
source ws/install/setup.bash
```

`run_lidar.sh` starts the Unitree driver and RViz and therefore is not part of Phase 1 build verification. Run it only after the LiDAR connection and configuration have been reviewed:

```bash
./run_lidar.sh
```

Useful ROS 2 checks:

```bash
ros2 topic list
ros2 node list
ros2 topic echo /unilidar/cloud
```

---

## 20. Common Problems

### Problem: I cannot see the repository

Make sure:

- You created a GitHub account.
- You sent your GitHub username to the repo owner.
- You accepted the invitation email.
- You are signed into the correct GitHub account.

### Problem: GitHub Desktop asks me to fork the repository

This usually means you do not have write access to the repository.

Ask the repository owner to add you as a collaborator or give you access through the organization.

### Problem: I changed files but GitHub Desktop shows nothing

Make sure you edited files inside the cloned repository folder, not a copy somewhere else.

### Problem: I pushed but my teammate cannot see my code

Make sure you clicked both:

```text
Commit
```

and:

```text
Push origin
```

Committing saves locally. Pushing uploads to GitHub.

### Problem: I get a merge conflict

A merge conflict happens when two people edit the same part of the same file.

Ask a teammate for help before clicking random buttons. Do not delete code unless you know which version should stay.

---

## 21. Quick Cheat Sheet

```text
Clone = download the repository for the first time
Pull = download the latest changes
Commit = save your changes locally
Push = upload your changes to GitHub
Branch = separate workspace for changes
Pull Request = request to merge your branch into main
Merge = combine changes into the main branch
```

---

## 22. Best Practices for This Project

- Pull before you start working.
- Commit small, clear changes.
- Use meaningful commit messages.
- Do not commit generated folders like build, install, and log.
- Use branches for bigger changes.
- Ask for a review before merging major code changes.
- Keep the README updated when commands or setup steps change.
