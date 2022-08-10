# install on ubuntu

## 1. installation version guideline

ROS is not strictly tied to Ubuntu-based operating systems; however, Ubuntu is the primarily supported operating system for ROS. "LTS" (long term support) distributions of ROS are synchronized with the LTS distributions of Ubuntu. To maximize compatibility, the distribution of ROS you install should match the version of Ubuntu you are running based on this list:

- Ubuntu 14.04.06 LTS (Trusty Tahr) --> ROS Indigo Igloo
- Ubuntu 16.04.7 LTS (Xenial Xerus) --> ROS Kinetic Kame
- Ubuntu 18.04.5 LTS (Bionic Beaver) --> ROS Melodic Morenia
- Ubuntu 20.04.1 LTS (Focal Fossa) --> ROS Noetic Ninjemys 

[ROS/Installation - ROS Wiki](http://wiki.ros.org/ROS/Installation)







## 1. First Method

[http://wiki.ros.org/noetic/Installation/Ubuntu](http://wiki.ros.org/noetic/Installation/Ubuntu)

### 1.1 Configure your Ubuntu repositories

[https://help.ubuntu.com/community/Repositories/Ubuntu](https://help.ubuntu.com/community/Repositories/Ubuntu)

![](images/2022-06-28_135240.png)

### 1.2 Setup your sources.list

Setup your computer to accept software from packages.ros.org.

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

![](images/2022-06-28_135729.png)

### 1.3 Set up your keys

```
sudo apt install curl # if you haven't already installed curl
```

![](images/2022-06-29_134053.png)

```
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

![](images/2022-06-29_134201.png)

to resolve this issue "**gpg: no valid OpenPGP data found.**"

```
curl -O -k https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc
sudo apt-key add ros.asc 
```

![](images/2022-06-29_135304.png)

### 1.4 Installation

First, make sure your Debian package index is up-to-date:

```
sudo apt update
```

![](images/2022-06-29_135441.png)

Now pick how much of ROS you would like to install.

**Desktop-Full Install: (Recommended)** : Everything in **Desktop** plus 2D/3D simulators and 2D/3D perception packages.

```
sudo apt install ros-noetic-desktop-full
```

![](images/2022-06-29_135658.png)

There are even more packages available in ROS. You can always install a specific package directly.

**sudo apt install ros-noetic-PACKAGE**

e.g.

```
sudo apt install ros-noetic-slam-gmapping
```

![](images/2022-06-29_151110.png)

To find available packages, see [ROS Index](https://index.ros.org/packages/page/1/time/#noetic) or use:

```
apt search ros-noetic
```

![](images/2022-06-29_151211.png)

### 1.5 Environment setup

You must source this script in every **bash** terminal you use ROS in.

```
source /opt/ros/noetic/setup.bash
```

It can be convenient to automatically source this script every time a new shell is launched. These commands will do that for you.

```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

![](images/2022-06-29_151832.png)

### 1.6 Dependencies for building packages

```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

![](images/2022-06-29_151932.png)

### 1.7 Initialize rosdep

Before you can use many ROS tools, you will need to initialize `rosdep`. `rosdep` enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS. If you have not yet installed `rosdep`, do so as follows.

```
sudo apt install python3-rosdep
```

![](images/2022-06-29_152046.png)

With the following, you can initialize `rosdep`.

```
sudo rosdep init
rosdep update
```

![](images/2022-06-29_152148.png)

```
roscore
```

![](images/2022-06-29_152244.png)

## 2. Second Method

### 2.1 update software center address

Please find setting item, to choose about item, than update software center address.

![](images/2022-05-15_140458.png)

### 2.2 ROS Noetic installation instruction

These instructions will install **ROS Noetic Ninjemys**, which is available for Ubuntu Focal (20.04), Debian Buster (10). [noetic/Installation/Ubuntu - ROS Wiki](http://wiki.ros.org/noetic/Installation/Ubuntu)

#### 2.2.1 fixed rosdep update issues

Step1 download into your local filesystem

[ros/rosdistro: This repo maintains a lists of repositories for each ROS distribution (github.com)](https://github.com/ros/rosdistro)

![](images/2022-05-17_132226.png)

**Step2 update /etc/ros/rosdep/sources.list.d/20-default.list**

```
sudo gedit /etc/ros/rosdep/sources.list.d/20-default.list
```

```
yaml file:///home/pfnie/Desktop/rosdistro-master/rosdep/osx-homebrew.yaml osx

# generic
yaml file:////home/pfnie/Desktop/rosdistro-master/rosdep/base.yaml
yaml file:///home/pfnie/Desktop/rosdistro-master/rosdep/python.yaml
yaml file:///home/pfnie/Desktop/rosdistro-master/rosdep/ruby.yaml
gbpdistro file:///home/pfnie/Desktop/rosdistro-master/releases/fuerte.yaml fuerte
```

**Step3 update /usr/lib/python3/dist-packages/rosdep2/sources_list.py  in line 72**

```
sudo gedit /usr/lib/python3/dist-packages/rosdep2/sources_list.py
```

```
DEFAULT_SOURCES_LIST_URL = 'file:///home/pfnie/Desktop/rosdistro-master/rosdep/sources.list.d/20-default.list'
```

**Step4 update /usr/lib/python3/dist-packages/rosdep2/rep3.py  in line 39**

```
sudo gedit /usr/lib/python3/dist-packages/rosdep2/rep3.py
```

```
REP3_TARGETS_URL = 'file:///home/pfnie/Desktop/rosdistro-master/releases/targets.yaml'
```

**Step5 update /usr/lib/python3/dist-packages/rosdistro/init.py  in line 68**

```
sudo gedit /usr/lib/python3/dist-packages/rosdistro/__init__.py
```

```
DEFAULT_INDEX_URL = 'file:///home/pfnie/Desktop/rosdistro-master/index-v4.yaml'
```

**Step6**

```
sudo rosdep init
```

**Step7**

```
rosdep update
```
