# `prcp_parking_ransac_lidar` package
ROS 2 C++ package.  [![Static Badge](https://img.shields.io/badge/ROS_2-Humble-34aec5)](https://docs.ros.org/en/humble/)
## Packages and build

It is assumed that the workspace is `~/ros2_ws/`.

### Clone the packages
``` r
cd ~/ros2_ws/src
```
``` r
git clone https://github.com/jkk-research/prcp_parking_ransac_lidar
```

### Build ROS 2 packages
``` r
cd ~/ros2_ws
```
``` r
colcon build --packages-select prcp_parking_ransac_lidar --symlink-install  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

<details>
<summary> Don't forget to source before ROS commands.</summary>

``` bash
source ~/ros2_ws/install/setup.bash
```
</details>

``` r
ros2 launch prcp_parking_ransac_lidar launch_example1.launch.py
```