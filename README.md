==================================
read_omni_dataset
==================================

This is a ROS package to systematically read sensor msgs from the omni-dataset (soccer robots dataset). 

==================================
Dataset download
==================================

Please download the soccer robot dataset in the rosbags format here (Please see instructions below on how to use the rosbags):

http://datasets.isr.ist.utl.pt/lrmdataset/4_Robots_DataSet/rosbags/

In case you want the raw data, i.e., simply the image files from the robot's camera, odometry log etc, in simple text format, please click on the following link. A short readme is also provided over there.

http://datasets.isr.ist.utl.pt/lrmdataset/4_Robots_DataSet/rawdata/

==================================
Citation
==================================

If you plan to use the dataset, please cite it as follows:

@PHDTHESIS {aahmad13phd,
    author  = "Aamir Ahmad",
    title   = "An Integrated Bayesian Approach to Multi-Robot Cooperative Perception",
    school  = "Instituto Superior T\'ecnico, Universidade T\'ecnica de Lisboa",
    year    = "2013",
    address = "Instituto Superior T\'ecnico, Av. Rovisco Pais 1, 1049-001 Lisboa, Portugal",
    month   = "apr",
    note    = "\"Instituto Superior T\'ecnico\" was formerly within  \"Universidade T\'ecnica de Lisboa\" and is now within \"Universidade de Lisboa\""
}

==================================
Breif Description
==================================

This package is designed as a read module that asynchronously reads messages from the socrob omni dataset. The purpose of this package is to provide a quick and ready-to-start platform for developing and implementing perception related algorithms utilizing the the omni dataset. Below, we first describe what this dataset has, then provide instructions on how to download, extract and run the bag files. Subsequently, you can find information on how to use this package to access the contents of the omni-dataset in the rosbags format. Of course, you are welcome to develop and extend this package as per your requirement. Comments and suggestions on improving this read-module package are also appreciated. On the other hand, you can write your own ros package or non-ros softwares to read the omni-dataset in either the rosbags format or the the raw format (as mentioned in the download section above).


==================================
What is the omni-dataset? 
==================================

The omni-dataset is a collection of time-stamped sensor data acquired from four omni-directional soccer robots. These four robots (and one more robot, not present in this dataset), called OMNI and named as OMNI1--OMNI5,  formed the SocRob Middle Sized League (MSL) team which participated in the RoboCup and national MSL championships from the years 1999 to 2013. Please note that the dataset also consists of ground truth data, acquired from an external stereo vision system.

For more information on the SocRob MSL team, please visit http://socrob.isr.ist.utl.pt/ 

For a more detailed description of the robot hardware and the vision system mounted on the robot, please see Appendix A of the thesis [1] downloadable here: http://welcome.isr.ist.utl.pt/pub/?accao=showpublication&id_publication=2873

Further description of the dataset:

Sensor data from robots OMNI1, OMNI3, OMNI4 and OMNI5 are present in this dataset. During data collection, uniquely colored markers were placed on top of each of the robot for the purpose of ground truth evaluation.

Two soccer balls (of different colors and FIFA standard size-5), connected to a thin string, were moved around in the IRSLab soccer field in a way such that their positions vary over time in 3D space. 10 distinct colored landmarks were placed at known locations on the field. The robots were manually maneuvered in such a way that they do not collide with each other as well as come far and close to the ball positions at different times. A ground truth system (GTS), explained in detail in Appendix B of [1], was used to save stereo image pairs while the raw sensor data collection was done. Each robot's laptop, which was placed inside the robot during the raw data collection, was used to log the following raw data along with their associated timestamps (in microseconds):

 -- Camera images from the fish-eye lens-based dioptric vision system of the robots at ~25 frames per second (FPS).
 -- Odometry readings from the motors at ~40 Hz.

The GTS was connected to an external desktop computer which logged stereo image pairs at ~20 FPS. All the robots' laptops and the GTS were synchronized using a time-synchronization software called `chrony'. 

Once the raw data was collected, 3D detection algorithm [2] for one of the balls (orange colored) and color based static landmark detection was performed on the images saved by each robot's vision system. Subsequently, the robot-frame logs containing the following items were generated for the full dataset.
 
 -- The orange ball's 3D positions in each robot's local frame with timestamps.
 -- All landmark's 2D positions (landmarks are fixed and are at known-positions on the ground plane) in each robot's local frame with timestamps.
 
Thereafter, the orange Ball's 3D position detection using its color and all the robot's 2D position (not the pose) detection using the colored marker placed on top of them were performed on the GTS' stereo pair images. Subsequently, the global-frame (note that we know the exact positions where the landmarks were placed in order to have the global map) GT logs contain

 -- The 3D GT positions of the orange ball and the 2D positions of all the robots with timestamps.

In summary, the full dataset consists of all of the above enumerated items, the robot-frame data from all four robots and the global-frame GT data for all 4 robots from the external stereo vision GT system. 
Note that the items enumerated above are less than all the measurement data that could be extracted from the raw data in the datasets made during the data collection.



==================================
Downloading, extracting and using 
the omni-dataset. 
==================================
################################################################################
################################################################################

Caution: The dataset is huge and may take hours to download depending upon your 
internet connection speed. Furthermore, after decompression and extraction, you
will require around 50 GB of space to store and access all files (either raw 
format or rosbags format)

################################################################################
################################################################################

################################################################################
Raw format: http://datasets.isr.ist.utl.pt/lrmdataset/4_Robots_DataSet/rawdata/
In here, you will find the following files:
1. Robots_Images_and_Odometry.tar.gz
2. GroundTruth_Images.tar.gz
3. How_To_Use.txt
4. thanks.txt
The file names above are self-explanatory. Currently, in the raw dataset you will only find the raw images and odometry. As mentioned in the previous section, the extracted robot-frame measurement data of the orange ball and the landmarks are not present in the raw format of the dataset. However, they are present in the rosbags format as explained further. In case you really require this secondary measurements in raw format, please feel free to email me directly.

Please use any decompression and file extraction software available of unix-like systems or perhaps Windows (Not tested, though)
################################################################################


################################################################################
Rosbags format: http://datasets.isr.ist.utl.pt/lrmdataset/4_Robots_DataSet/rosbags/
In here, you will find the following files:
1. four_msl_robots_dataset.tar.xz
2. four_robot_experiment_GT.bag
3. Right.tar.xz
We will now explain each of the above three files.

1. four_msl_robots_dataset.tar.xz
To decompress and extract, use the following command:

tar xf four_msl_robots_dataset.tar.xz

After extraction, you will find the following four rosbags, one for each robot:

OMNI1_odomballLandmarks.bag
OMNI3_odomballLandmarks.bag
OMNI4_odomballLandmarks.bag
OMNI5_odomballLandmarks.bag

The names are self explanatory. Each one contains the raw images (ppm format), ball and landmark measurement data and odometry data from the corresponding OMNI robot.
An example output of rosbag info on one of these bags is as follows.

----------------------------------------------------------------------
path:        /path-to-datasetDirectory/OMNI1_odomballLandmarks.bag
version:     2.0
duration:    8:00s (480s)
start:       May 07 2012 16:50:02.68 (1336402202.68)
end:         May 07 2012 16:58:03.12 (1336402683.12)
size:        10.6 GB
messages:    52342
compression: none [12272/12272 chunks]
types:       nav_msgs/Odometry                      [cd5e73d190d741a2f92e81eda573aca7]
             sensor_msgs/Image                      [060021388200f6f0f447d0fcd9c64743]
             socrobdata_to_rosbags/BallData         [161090ceadc5449a9fe867d5bfd7bba3]
             socrobdata_to_rosbags/LRMLandmarksData [1f23ef47c0a1f8ac7b3dc813240ff7b0]
topics:      omni1/devices/topcam/image_raw   12271 msgs    : sensor_msgs/Image                     
             omni1/landmarkspositions         12271 msgs    : socrobdata_to_rosbags/LRMLandmarksData
             omni1/odometry                   15530 msgs    : nav_msgs/Odometry                     
             omni1/orangeball3Dposition       12270 msgs    : socrobdata_to_rosbags/BallData
----------------------------------------------------------------------

Other rosbags have similar contents. Please note here that the ros message type "LRMLandmarksData" and "BallData" are custom-made and in fact provided by the package read_omni_dataset. You can either copy these message files from the msg folder of the read_omni_dataset ros package and use them in your own custom packages or simply use them within this package. Remember that the path to custom messages (here it is socrobdata_to_rosbags/) is irrelevant.

2. four_robot_experiment_GT.bag
Output of rosbag info on this bag is as follows.
----------------------------------------------------------------------
path:        /path-to-datasetDirectory/four_robot_experiment_GT.bag                                                                                                       
version:     2.0                                                                                                                                                                              
duration:    7:50s (470s)                                                                                                                                                                     
start:       May 07 2012 16:50:06.99 (1336402206.99)                                                                                                                                          
end:         May 07 2012 16:57:57.67 (1336402677.67)                                                                                                                                          
size:        14.8 MB                                                                                                                                                                          
messages:    10118                                                                                                                                                                            
compression: none [20/20 chunks]
types:       socrobdata_to_rosbags/LRMGTData [a7f5dd7185c9e08c3c3b2baefda96b70]
topics:      gtData_4robotExp   10118 msgs    : socrobdata_to_rosbags/LRMGTData
----------------------------------------------------------------------

The message type LRMGTData has the following contents:
----------------------------------------------------------------------
read_omni_dataset/BallData orangeBall3DGTposition
string LeftFilename
string RightFilename
bool foundOMNI4
bool foundOMNI3
bool foundOMNI1
bool foundOMNI5
geometry_msgs/PoseWithCovariance poseOMNI4
geometry_msgs/PoseWithCovariance poseOMNI3
geometry_msgs/PoseWithCovariance poseOMNI1
geometry_msgs/PoseWithCovariance poseOMNI5
----------------------------------------------------------------------

Almost all of the above contents are named in a self-explanatory way. "RightFilename" refers to the image file name from the riht-camera of the stereo-vision ground truth system's camera. Similarly, the left filename. This filename is provided to look up for the image file in the folder named "Right" which can be obtained by extracting the third archive (see point 3 below). The image file can then be used in many ways, for example, to overlay the GT values of thhe robot position/ball position on to the images from the right camera.

3. Right.tar.xz
To decompress and extract, use the following command:

tar xf Right.tar.xz

After extraction, you will find the folder "Right" which has the images from the right-side camera of the stereo vision ground truth system. See point 2 above on how to use it. 

             
################################################################################


==================================
How to use the read_omni_dataset
package with the omni-dataset
==================================
Step-wise instructions
0. Make sure that you have ros (indigo or hydro) installed.
1. Make sure you have all the dataset files downloaded.
2. Make sure you downloaded the latest version of the read_omni_dataset rospackage and have compiled it (basically putting it in your catkin_ws/src folder, currectly setting up the source path and doing catkin make). 
3. Note that you can choose to run one or more bag files (corresponding to each one of the robot) at a time, depending on how many robots you want to use in your application. Meaning, if you want to use a single robot say OMNI4, you can simply run the bag file corresponding to OMNI4. Or else, if you want to use all four robots, play simultaneously all 4 bag files. In the instructions further on, we choose a case where all 4 robots are supposed to be used.
4. In the first terminal screen execute roscore.
> roscore
5. In a second terminal screen execute the bags
> rosbag play -l /path-to-datasetDirectory/OMNI1_odomballLandmarks.bag /path-to-datasetDirectory/OMNI3_odomballLandmarks.bag /path-to-datasetDirectory/OMNI4_odomballLandmarks.bag /path-to-datasetDirectory/OMNI5_odomballLandmarks.bag /path-to-datasetDirectory/four_robot_experiment_GT.bag 
6. In a third terminal screen execute the following command
> rosrun read_omni_dataset read_omni_dataset

If everything went fine, you will see a huge stream of rosouts (a lot of messages outputted onto the terminal). An example extract of those outputs is given below:

----------------------------------------------------------------------
[ WARN] [1417774021.789838378]:  got landmark data from teammate robot (ID=4)
[ INFO] [1417774021.789848316]: Landmark 4 found in the image, refer to the method to see how covariances are calculated
[ INFO] [1417774021.789856914]: Landmark 5 found in the image, refer to the method to see how covariances are calculated
[ INFO] [1417774021.789866232]: Landmark 6 found in the image, refer to the method to see how covariances are calculated
[ INFO] [1417774021.789874357]: Landmark 8 found in the image, refer to the method to see how covariances are calculated
[ INFO] [1417774021.789884725]: Landmark 9 found in the image, refer to the method to see how covariances are calculated
[ WARN] [1417774021.795990169]:  got odometry from self-robot (ID=1) at time 1336402229
[ INFO] [1417774021.796059119]: Odometry propogated self robot state is x=4.975660, y=-2.648219, theta=3.138352
[ WARN] [1417774021.807255918]: Got ball data from self robot 1
[ INFO] [1417774021.807296673]: Ball found in the image, refer to the method to see how covariances are calculated
[ WARN] [1417774021.807314958]:  got landmark data from self robot (ID=1)
[ INFO] [1417774021.807326304]: Landmark 6 found in the image, refer to the method to see how covariances are calculated
[ INFO] [1417774021.807335590]: Landmark 7 found in the image, refer to the method to see how covariances are calculated
[ INFO] [1417774021.807343925]: Landmark 8 found in the image, refer to the method to see how covariances are calculated
----------------------------------------------------------------------

What this basically shows is what read_omni_dataset rosnode is able to read directly from the rosbags. As stated before, the purpose of this rospackage is to provide a starting platform to develop perception related algorithms. The read_omni_dataset rosnode asynchronously reads the sensor messages from the rosbags and generates a callback specific to the message. The callbacks are per robot per sensor but can be synced and stitched using synchronization packages available in ros. The developer is invited to check the definitions of the callback functions and develop their perception algorithm there.

In the header file of the package there are a number of static variables dependent on the kind of application. Some basic ones, which you must always set manually are:

NUM_ROBOTS = 4; // total number of robots in the team including self which is required to run in the particular application.
ROB_HT = 0.81; // fixed height of the OMNI robots above ground in meter.
MY_ID = 1; // Use this flag to set the ID of the robot expected to run a certain decentralized algorithm. Robot with MY_ID will be trated as the self robot running the algorithm while the rest will be considered teammates. Note that in the dataset there are 4 robots with IDs 1,3,4 and 5. Robot with ID=2 is not present.

In the next improvements, these parameters will be moved to rosnode arguments and can be set in the launch file.
==================================
References
==================================

[1] An Integrated Bayesian Approach to Multi-Robot Cooperative Perception, Aamir Ahmad, Ph.D. Thesis, Instituto Superior Técnico, Universidade Técnica de Lisboa, 2013.
[2] 3D to 2D bijection for spherical objects under equidistant fisheye projection, Aamir Ahmad, João Xavier, José Santos-Victor, Pedro Lima, Computer Vision and Image Understanding (CVIU), Volume 125, August 2014, Pages 172-183, ISSN 1077-3142, http://dx.doi.org/10.1016/j.cviu.2014.04.004.


