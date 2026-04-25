This ros-driver is modified from [https://github.com/IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros) and is maintained for Intel RealSense D400 FW 5.16.0.1 with librealsense2 2.55.1.
Tests are performed on Intel RealSense D435 and D435i.

The first modification we added is to turn on/off the emitter from launch file by ```enable_emitter```.

The second modification is that a blacklists is added to file _nodelet_plugins.xml_.
It blocks some rarely used topics, which makes the rostopic lists more clear.

What's more
This driver enables the laser emitter strobe every other frame, allowing the device to output high quality depth images with the help of emitter, and along with binocular images free from laser interference. On FW 5.16.0.1 this is controlled through librealsense `RS2_OPTION_EMITTER_ON_OFF`.

Parameter ```emitter_on_off``` is to turn on/off the added feature.
Note that if this feature is turned on, the output frame rate from the device will be reduced to half of the frame rate you set, since the device uses half of the stream for depth estimation and the other half as binocular grayscale outputs.

Note that when ```emitter_on_off``` is set to true, parameters ```depth_fps``` and ```infra_fps``` must be identical. The driver will force ```enable_emitter=true``` because alternating emitter mode requires the laser.
For FW 5.16.0.1 the default ```emitter_on_off_depth_phase``` is 0. If infrared images still contain laser spots, flip it to 1. The optional ```emitter_on_off_use_metadata``` parameter is disabled by default because emitter metadata can be unreliable for this alternating sub-preset on some D400 firmware/kernel combinations.

By wangzhepei@zju.edu.cn & iszhouxin@zju.edu.cn

==compile error! fixed==

By LinkaiTang@mail.nwpu.edu.cn
