# glove_tekscan_ros_wrapper

This package reads data from various sensors (Tekscan/Cyberglove/ATI FT sensor/Vision Tracker), and then synchronizes and publishes them to a ROS topic (/LasaDataStream). The executable file is IntTacMain that you can just run from the command line.

#### Requirements:

OS: Ubuntu 14.04
ROS compatibility: Indigo
YARP

### Initial Setup

##### On the Tekscan computer (currently lasapc43)
Run yarpserver
Initialize the glove and tactile sensors utility
check that data is streaming correctly over yarp ports 
in browser >> 128.178.145.25:10000

##### On the local computer 
```
$ yarp detect
```

```
$ yarp conf
```

```
$ vim /home/user/.yarp/conf/yarp.conf
```

```
$ yarp namespace /Network2
```

```
$ yarp server IP 128.178.145.25
```


### About the data fields

The data fields are indexed with the name of each phalange (e.g., index_f_1 refers to index frontal side and the first phalange). 

Note there are some fields with a suffix “_avg” means that’s the average reading from that phalange tile,  while the original data is accessible from the corresponding field without this suffix. It is a vector indicating 4x4 or 4x3 layout.
 
