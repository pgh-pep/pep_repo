# Setting up PEP Development Environment

We will be using [ROS2](https://docs.ros.org/en/humble/index.html) as our framework

## Developing using WLS

### 1. Install Ubuntu 22.04 w/ WSL2

Make sure Windows is up to date.

To install WSL2, in a PowerShell or Windows Command Prompt run:

```powershell
wsl --install -d Ubuntu-22.04
wsl --set-default-version 2
```

You will be promted to enter a username and password followed by a successful installation message.

To confirm a successful installation of WSL and Ubuntu, you can list your currently installed distros with:

```powershell
wsl --list -v
```

I recommend installing the [Windows Terminal](https://www.microsoft.com/store/productId/9N0DX20HK701?ocid=pdpshare) tool. (Will need to select it as the default terminal)

### 3. ROS Installation

Open the bash terminal of your newly installed Ubuntu distro.

Update and upgrade packages:

```bash
sudo apt-get update && sudo apt-get upgrade -y
```

Add the ROS 2 apt repository to your system:

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update && sudo apt-get upgrade -y
```

Install ROS2 Humble:

```bash
sudo apt install ros-humble-desktop -y
```

ROS must be sourced every time the terminal is launched. This will add ROS to the `~/.bashrc` file to be automatically sourced and add/source colcon autocompletion.

```bash
sudo apt-get install python3-colcon-common-extensions -y
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
source ~/.bashrc
```

```bash
sudo apt-get install python3-rosdep -y
sudo rosdep init
rosdep update
```

### 3. Install Dependencies

Install other Ubuntu dependencies:

```bash
sudo apt-get install bash-completion nano python3-pip python-is-python3 -y
```

Install Gazebo:

```bash
sudo apt-get install ros-humble-gazebo-ros-pkgs -y
```

Verify Installation by initializing Gazebo:

```bash
gazebo
```

### 4. Set up Git

Set up Git credentials:

```bash
git config --global user.name "your_user_name"
git config --global user.email "youremail@domain.com"
```

Connect with a [SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent):
[SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent):

```bash
# Generate SSH key (Skip prompts w/ enter):
ssh-keygen -t ed25519 -C "your_email@example.com"
# Display public key in terminal:
cat ~/.ssh/id_ed25519.pub
# Copy the entire key
```

Add the key to your [github account](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account): `Settings > SSH and GPG Keys > New SSH Key`

### 5. Set up the Repository

In your bash terminal, clone the `pep_repo` repository with SSH:

```bash
git clone git@github.com:pgh-pep/pep_repo.git
```

To edit the code using VSCode, either open using bash:

```bash
cd pep_repo
code .
```

Or open up [VSCode](https://code.visualstudio.com/download) on your Windows machine. Install the Remote Development Extension.
In the bottom-left, click on the Remote Window button (blue w/ arrows).
Select `Connect to WSL using Distro` and select `Ubuntu-22.04`.

This will open VSCode in your newly made WSL environment.

Install the reccomended VSCode extensions and install PyPI dependencies with:

```bash
pip install -r requirements.txt
```

To start your ROS2 workspace (make sure you are in `pep_repo`):

```bash
colcon build --symlink-install
```
Finally, source your ROS2 workspace:

```bash
echo "source ~/pep_repo/install/setup.bash" >> ~/.bashrc
```

Your PEP developer desktop is all set up! You can work on other projects as well using this WSL Distro.

## Common Git Commands

To change git settings:

```bash
git config --global --edit
```

To create a new branch:

```bash
git checkout -b branchName
```

To commit changes :

```bash
git commit -m "write_a_message"
```

To uncommit the last commit:

```bash
git reset HEAD~
```

To push changes to the git branch you are on:

```bash
git push
```
