REM call c:/ros2-windows/local_setup.bat
echo off

cd ROS2_ws

rmdir /s /q build
rmdir /s /q install
rmdir /s /q log

colcon build --merge-install

cd ..
