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
    Release 0.1.0
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

* pycomm
  ```sh
  pip3 install pycomm
  ```

### Installation (Linux)

#### Create a ROS2 Workspace
1. Source ROS2 Environment
   ```sh
   source /opt/ros/humble/setup.bash
   ```
2. Create a new directory
   ```sh
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```
3. Clone repo
   ```sh
   https://github.com/UofI-CDACS/fanuc_ros2_drivers.git
   ```
4. Resolve Dependencies
   ```sh
   # cd if you're still in the ``src`` directory
   cd ..
   rosdep install -i --from-path src --rosdistro humble -y
   ```
5. Build the workspace with colcon
   ```sh
   colcon build
   ```
6. Source the overlay
   ```sh
   source /opt/ros/humble/setup.bash
   ```
   Go into the root of your workspace
   ```sh
   cd ~/ros2_ws
   ```
   In the root, source the overlay
   ```sh
   source install/local_setup.bash
   ```
   _Full guide here: [ROS2 Humble Documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)_
   
   Installation Complete
   
<p align="right">(<a href="#readme-top">back to top</a>)</p>
<!-- USAGE EXAMPLES -->

## Usage

## Running Action Server/s
!Note: Always source when opening new terminal
1. Open new terminal
2. Source the overlay
   ```sh
   source /opt/ros/humble/setup.bash
   ```
   Go into the root of your workspace
   ```sh
   cd ~/ros2_ws
   ```
   In the root, source the overlay
   ```sh
   source install/local_setup.bash
   ```
3. Navigate to desired package
   ```sh
   #example:
   cd ~/ros2_ws/src/fanuc_ros2_driver/fanuc_ros2_driver/fanuc_ros2_driver/
   ```
4. Spin up action servers
   ```sh
   #example: pass robot IP
   python3 write_joint_pose_server.py 129.1.1.1
   ```
   Action server is now waiting to receive goals


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
