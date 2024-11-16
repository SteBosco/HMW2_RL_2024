## Enable acces to the USB port

Ensure you add the `--privileged` flag to the Docker run command to grant the container additional privileges, such as access to hardware resources.

Add your current user to the dialout group
```
sudo usermod -aG dialout <username>
```
This grants your user access to serial devices (e.g., `/dev/ttyUSB0`)

Then, enable access to the port

```
sudo chmod a+rw /dev/ttyUSB0
```

## Build Dynamixel SDK
Build the `dynamixel_sdk` libraries to use methods for reading and writing to the motor registers.

- **dynamixel_sdk**: Build the libraries to use methods for reading and writing to the motor registers.
- **dynamixel_sdk_custom_interfaces**: This library generates custom messages and services that will be used by the `read_write` node.

```
colcon build --packages-select dynamixel_sdk dynamixel_sdk_custom_interfaces dynamixel_sdk_examples
```

## Run arm_description, necessary to build the KDL chain
```
ros2 launch arm_description display.launch.py
```
## Run dynamixel_sdk_examples, the subscriber that accepts commands to the dynamixel
```
ros2 run dynamixel_sdk_examples read_write_node
```

## Run kdl_arm_dynamixel
```
ros2 run kdl_arm_dynamixel main_real
```

 