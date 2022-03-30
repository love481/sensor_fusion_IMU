# Sensor Fusion
This repo consists of the code for fusing acceleration,gyroscope and magnetometer data using simple publisher and subscriber in ROS.You can also visualize the raw and clear data after performing required calibration and applying fusion algorithms. 


## Installation
Use following command to clone the repository.
```bash
git clone https://github.com/love481/sensor_fusion_IMU
```
Make sure you have ros installed along with needed packages to communicate with arduino. For more info visit this [page](http://wiki.ros.org/rosserial_python) on how to communicate via ros_serial_package.

## Usage
Upload .ino code in the arduino of the examples folder and run the following command:
```bash
mkdir -p <your_project_name>/src
```
Now, go to the /src dir and copy filters_algorithms folder there and run from <your_project_name> directory run:
```bash
catkin_make
```
Don't forget to source the devel folder and run the following command to communicate with arduino.
```bash
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM1 _baud:=115200
rosrun filters_algorithms imu_node
```
For visualization run:
```bash
rosrun filters_algorithms visualization_imu_data.py
```
## Plot
**Red plot denotes yaw data from magnetometer before applying kalman filter and Blue plot denotes filtered yaw data**
![Screenshot from 2021-06-27 19-44-13](https://user-images.githubusercontent.com/54012619/123627959-53c02d80-d832-11eb-8f4f-2bdc80b94b14.png)


**Magnetometer data before Calibration**

![before_cal](https://user-images.githubusercontent.com/54012619/123629421-fe851b80-d833-11eb-9380-37ccd1c2dc4f.png)

**Magnetometer data after Calibration**

![after_cal](https://user-images.githubusercontent.com/54012619/123629607-2ffde700-d834-11eb-8e7f-4db24a32e92a.png)
## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.
## License
[MIT](https://choosealicense.com/licenses/mit/)
