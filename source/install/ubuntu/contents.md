# install on ubuntu

## 1. installation version guideline

ROS is not strictly tied to Ubuntu-based operating systems; however, Ubuntu is the primarily supported operating system for ROS. "LTS" (long term support) distributions of ROS are synchronized with the LTS distributions of Ubuntu. To maximize compatibility, the distribution of ROS you install should match the version of Ubuntu you are running based on this list:

- Ubuntu 14.04.06 LTS (Trusty Tahr) --> ROS Indigo Igloo
- Ubuntu 16.04.7 LTS (Xenial Xerus) --> ROS Kinetic Kame
- Ubuntu 18.04.5 LTS (Bionic Beaver) --> ROS Melodic Morenia
- Ubuntu 20.04.1 LTS (Focal Fossa) --> ROS Noetic Ninjemys 

[ROS/Installation - ROS Wiki](http://wiki.ros.org/ROS/Installation)

### 1.1 update software center address

Please find setting item, to choose about item, than update software center address.

![2022-05-15_140458](images/2022-05-15_140458.png)

## 2. ROS Noetic installation instruction

These instructions will install **ROS Noetic Ninjemys**, which is available for Ubuntu Focal (20.04), Debian Buster (10). [noetic/Installation/Ubuntu - ROS Wiki](http://wiki.ros.org/noetic/Installation/Ubuntu)

### 2.1 fixed rosdep update issues

Step1 download into your local filesystem

[ros/rosdistro: This repo maintains a lists of repositories for each ROS distribution (github.com)](https://github.com/ros/rosdistro)

![2022-05-15_140458](images/2022-05-17_132226.png)

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
