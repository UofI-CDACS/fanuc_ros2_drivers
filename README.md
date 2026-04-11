<a name="readme-top"></a>
<!-- PROJECT SHIELDS -->
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

<!-- PROJECT LOGO -->
<!--
<br />
<div align="center">
  <a href="https://github.com/UniversityOfIdahoCDACS/FANUC-ROS2_Drivers">
    <img src="images/logo.png" alt="Logo TBD" width="80" height="80">
  </a>
-->
<h3 align="center">FANUC ROS2 Drivers</h3>

  <p align="center">
    Release 0.1.1
    <br />
    <!--
    <a href="https://github.com/UniversityOfIdahoCDACS/FANUC-ROS2_Drivers"><strong>Explore the docs »</strong></a>
-->
    <br />
    <br />
  </p>
</div>

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

<!-- ADD SCREENSHOT HERE -->
ROS2 Solution for FANUC robots

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Built With

* [![Python][Python-shield]][Python-url]

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- GETTING STARTED -->
## Getting Started

### Prerequisites

* ROS2 Jazzy (or Humble)
* Python 3 and pip
* catkin_pkg (required for ROS2 build system)
  ```sh
  sudo apt install python3-catkin-pkg python3-rosdep
  ```
* Put Fanuc TP programs on controller
    - 'ros2_eip_back.tp' needs to be running in the background
    - 'ros2_eip_mainv2.tp' runs in the foreground when you want to use ROS

#### Camera Prerequisites (optional — required only for vision features in `tests/`)

The vision test scripts use a MindVision GigE camera via the MindVision SDK.

1. Download the MindVision SDK for Linux from the [MindVision website](https://www.mindvision.com.cn/rjxz/list_12.aspx)
2. Install the SDK to register the shared library system-wide:
   ```sh
   cd <path-to-sdk>
   sudo bash install.sh
   ```
3. Copy the Python wrapper into the `tests/` directory:
   ```sh
   cp <path-to-sdk>/demo/python_demo/mvsdk.py tests/
   ```
4. If using a GigE camera on a dedicated Ethernet adapter, add a static secondary IP on that adapter so the host is in the same subnet as the camera. To make it persistent via NetworkManager:
   ```sh
   nmcli connection modify "<your-adapter-connection-name>" +ipv4.addresses "<host-ip>/<prefix>"
   nmcli connection up "<your-adapter-connection-name>"
   ```

### Installation (Linux)

#### Create a ROS2 Workspace
1. Source ROS2 Environment (use jazzy or humble depending on your installation)
   ```sh
   source /opt/ros/jazzy/setup.bash
   ```
Optionally, you can also run the following command to add it to your .bashrc file:
   ```sh
   echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
   ```
2. Create a new directory
   ```sh
   mkdir -p ~/ros2_ws/
   cd ~/ros2_ws/
   ```
3. Clone repo
   ```sh
   git clone https://github.com/UofI-CDACS/fanuc_ros2_drivers/ --branch v1.1
   ```
4. Initialize rosdep (first time only)
   ```sh
   sudo rosdep init
   rosdep update
   ```
5. Resolve Dependencies (use jazzy or humble to match your ROS2 version)
   ```sh
   rosdep install -i --from-path src --rosdistro jazzy -y
   ```
   Note: pycomm3 may need to be installed separately (handle this yourself via some sort of venv management):
   ```sh
   pip3 install pycomm3
   ```
6. Build the workspace with colcon
   ```sh
   colcon build
   ```

   _Full guide here: [ROS2 Humble Documentation](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)_
   
   Installation Complete
   
<p align="right">(<a href="#readme-top">back to top</a>)</p>
<!-- USAGE EXAMPLES -->

## Usage

## Running Nodes (Server)
1. Open new terminal and go to workspace
2. Source overlay (after building workspace):
   ```sh
   source install/setup.sh
   ```
3. Run launch
   ```sh
   ros2 launch launch/start.launch.py robot_name:=NAME_OF_ROBOT robot_ip:=0.0.0.0 # Parameters must be formated this way or the command will give you an error
   ```
  - This script only works for one robot. If you need to start more than 1 robot, this should be done in mulitple terminals. (WIP)
## Using Nodes (Client)
1. Open new terminal and go to workspace
2. Source overlay
   ```sh
   source install/setup.sh
   ```
3. Start using!
    - run a regular python file 
    - see tests/ for examples

**Note: You will always need to source the overlay for each new terminal you open**

_For more examples, please refer to the [Documentation](https://example.com)_

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- ROADMAP -->
## Roadmap

- [ ] More ROS2 Packages

See the [open issues](https://github.com/UniversityOfIdahoCDACS/FANUC-Ethernet_IP_Drivers/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- LICENSE -->
## License

Distributed under the GNU General Public License v3. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p

<!-- CONTACT -->
## Contact

Project Link: [https://github.com/UniversityOfIdahoCDACS/FANUC-ROS2_Drivers](https://github.com/UniversityOfIdahoCDACS/FANUC-ROS2_Drivers  )

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[Python-shield]:  https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white
[Python-url]: https://www.python.org/
