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
Brief Description
==================================

This package is designed as a read module that asynchronously reads messages from the socrob omni dataset. The purpose of this package is to provide a quick and ready-to-start platform for developing and implementing perception related algorithms utilizing the the omni dataset. In the wiki of this package ( https://github.com/aamirahmad/read_omni_dataset/wiki ), we first describe what this dataset has, then provide instructions on how to download, extract and run the bag files. Subsequently, you can find information on how to use this package to access the contents of the omni-dataset in the rosbags format. Of course, you are welcome to develop and extend this package as per your requirement. Comments and suggestions on improving this read-module package are also appreciated. On the other hand, you can write your own ros package or non-ros softwares to read the omni-dataset in either the rosbags format or the the raw format (as mentioned in the download section above).

Please see the wiki page of this package for more info on how to use the dataset and the package.
