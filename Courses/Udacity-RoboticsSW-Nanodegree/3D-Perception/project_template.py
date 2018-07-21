#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)

    return yaml_dict


# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

    ## Outlier removal
    # Create an outlier filter
    outlier_filter = cloud.make_statistical_outlier_filter()
    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(100)
    # Set threshold scale factor
    x = 0.5
    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)
    # Apply filter function to generate filtered cloud 
    cloud_filtered = outlier_filter.filter()

    #pcl_scene_pub.publish(pcl_to_ros(cloud_filtered))

    ## Voxel Grid Downsampling
    # create voxel grid filter
    vox = cloud.make_voxel_grid_filter()

    # set leaf size
    LEAF_SIZE = 0.005
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    # Apply filter function to generate filtered cloud
    cloud_filtered = vox.filter()

    ## PassThrough Filtering

    # create passthrough filter
    passthru = cloud_filtered.make_passthrough_filter()
    # set filter axis to be z-axis
    filter_axis = 'z'

    # set min, max limits for cropping along z-axis
    axis_min = 0.5
    axis_max = 1.0
    passthru.set_filter_field_name(filter_axis)
    passthru.set_filter_limits(axis_min, axis_max)
    
    cloud_filtered = passthru.filter()

    passthru = cloud_filtered.make_passthrough_filter()
    filter_axis = 'y'
    axis_min = -0.5
    axis_max = 0.5
    passthru.set_filter_field_name(filter_axis)
    passthru.set_filter_limits(axis_min, axis_max)


    # apply passthrough filter to generated filtered cloud
    cloud_filtered = passthru.filter()
    pcl_scene_pub.publish(pcl_to_ros(cloud_filtered))
    

    ## RANSAC Plane Segmentation

    # create segmentation filter
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # set max distance for point to be inlier 
    max_distance = 0.02
    seg.set_distance_threshold(max_distance)

    # Extract inliers and outliers
    inliers, coeff = seg.segment()
    cloud_objects = cloud_filtered.extract(inliers, negative=True)

    ## Euclidean Clustering

    # obtain XYZ point cloud from XYZRGB point cloud
    white_cloud = XYZRGB_to_XYZ(cloud_objects)

    # create kd tree
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(10000)
    ec.set_SearchMethod(tree)

    # extract indices corresponding to inliers
    cluster_indices = ec.Extract()
 
    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []

    # enumerate over list of cloud indices
    for j, indices in enumerate(cluster_indices):
        # enumerate over indices in each cloud index list
        for i, index in enumerate(indices):
            color_cluster_point_list.append([white_cloud[index][0],
            					white_cloud[index][1], 
                                                white_cloud[index][2],
                                                rgb_to_float(cluster_color[j])])
 
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    #Create cluster cloud from cluster point list
    cluster_cloud.from_list(color_cluster_point_list)

    # Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS message
    # pcl_scene_pub.publish(ros_cloud_objects)
    print "Eucledian Clustering completed"

    # Exercise-3 TODOs:
    detected_objects_labels = []
    detected_objects = []

    # Classify the clusters! (loop through each detected cluster one at a time)
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
    	pcl_cluster = cloud_objects.extract(pts_list)
        # Compute the associated feature vector
        ros_cluster = pcl_to_ros(pcl_cluster)
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction
	# print feature.shape
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))


        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    # Publish the list of detected objects
    # pcl_scene_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    detected_objects_list = detected_objects

    detected_objects_pub.publish(detected_objects_list)

    #for x in detected_objects_list:
    #    print x.label
    # pcl_scene_pub.publish(detected_objects_list)
    try:
        pr2_mover(detected_objects_list)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):


    # calculate centroids for detected objects and store in list
    labels = []
    centroids = [] # to be list of tuples (x, y, z)
    for object in object_list:
        labels.append(object.label)
        points_arr = ros_to_pcl(object.cloud).to_array()
        centroids.append(np.mean(points_arr, axis=0)[:3])

    # TODO: Initialize variables
    yaml_dict_list = []
    test_scene_num = Int32()
    object_name = String()
    which_arm =  String()
    pick_pose = Pose()
    place_pose = Pose()

    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')
    left_place_pose = dropbox_param[0]['position']
    right_place_pose = dropbox_param[1]['position']

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    detected_labels = [object.label for object in object_list]
    # TODO: Loop through the pick list
    for i in range(len(object_list_param)):  #iterate over all objects in the pick list
	print i
        if object_list_param[i]['name'] in detected_labels:
            # print object_list_param[i]['name'] 
            index = detected_labels.index(object_list_param[i]['name'])
            object_name.data = object_list_param[i]['name']
            test_scene_num.data = 3
            which_arm.data = object_list_param[i]['group']
            if (which_arm.data == 'red'):
                place_pose.position.x = left_place_pose[0]
                place_pose.position.y = left_place_pose[1]
                place_pose.position.z = left_place_pose[2]
                print 'left'
            else:
                place_pose.position.x = right_place_pose[0]
                place_pose.position.y = right_place_pose[1]
                place_pose.position.z = right_place_pose[2]
                print 'right'
	    #print(centroids[index])
            pick_pose.position.x = np.asscalar(centroids[index][0])
            pick_pose.position.y = np.asscalar(centroids[index][1])
            pick_pose.position.z = np.asscalar(centroids[index][2])

            # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
            yaml_dict_list.append(make_yaml_dict(test_scene_num, which_arm, object_name, pick_pose, place_pose))

            # Wait for 'pick_place_routine' service to come up
            rospy.wait_for_service('pick_place_routine')
            print 'service wait completed'
            try:
                pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)
                # TODO: Insert your message variables to be sent as a service request
                resp = pick_place_routine(test_scene_num, object_name, which_arm, pick_pose, place_pose)
                print ("Response: ",resp.success)

            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file
    	    #yaml_dict_list.append(make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose))
    send_to_yaml('output_3.yaml', yaml_dict_list)
    print 'file printed'

if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node("clustering", anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_scene_pub = rospy.Publisher("pr2/world/pcl_scene", pc2.PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)


    # TODO: Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
