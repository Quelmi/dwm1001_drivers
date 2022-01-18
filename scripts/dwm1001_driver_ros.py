#!/usr/bin/env python

""" 
@file: dwm1001_driver_ros.py
@description: ros node to publish distance between tag and anchors
              using UART API from https://www.decawave.com/dwm1001/api/
@author: Esau Ortiz
@date: july 2021
"""

import rospy, tf, time, serial
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from uwb_msgs.msg import AnchorInfo

import sys
sys.path.append('/home/miquelserra/catkin_ws/src/dwm1001_drivers/scripts')

from dwm1001_apiCommands import DWM1001_API_COMMANDS, DWM1001_UART_API
from dwm1001_apiCommands import DWMRangingReq

class ReadyToLocalize(DWM1001_UART_API):

    def __init__(self, anchor_id_list, anchor_coord_list, world_frame_id, visualize_anchors = False):
        self.anchor_id_list = anchor_id_list
        self.anchor_coord_list = anchor_coord_list
        self.range_error_counts = [0 for i in range(len(self.anchor_id_list))]
        self.world_frame_id = world_frame_id
        self.visualize_anchors = visualize_anchors
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Get some other params
        self.is_location_engine_enabled = bool(rospy.get_param('~location_engine_enable'))
        self.dwm_port = rospy.get_param('~serial_port')
        self.use_network = rospy.get_param('~use_network', False)
        self.network = rospy.get_param('~network_id', "default")
        self.verbose = rospy.get_param('~verbose', False)

        # Empty dictionary to store topics being published
        self.topics = {}
        
    def getRangingData(self, verbose = False):
        """ Read and formats serial data 
            (i.e. ranges between tag and anchors in the same network)
        Parameters
        ----------
        Returns
        ----------
        anchor_poses: list
            list of anchor poses, each element has the following
            format [anchor_id, anchor_pose, anchor_distance]
        tag_pose: list
            list of [x, y, z] if localition engine is enabled both
            in the module and /params/tag_operation_mode.yaml
        """
        # Show distances to ranging anchors and the position if location engine is enabled
        ranging_request = DWMRangingReq(self.is_location_engine_enabled)
        data = self.getDataFromSerial(ranging_request, verbose = verbose)
        if data == []:
            return [], None
        if self.is_location_engine_enabled == True:
            anchor_data_array, tag_pose = [data[:-2], data[-1]]
            if tag_pose[:3] != 'est':
                tag_pose = None
        else:
            anchor_data_array = data
            tag_pose = None
        # Now each element of anchor_data_array has de following format: anchor_id[X,Y,Z]=distance_to_tag
        anchor_poses = []
        for anchor_data in anchor_data_array:
            try:
                anchor_id, anchor_data = anchor_data.split("[")
                anchor_id = anchor_id[-4:] # ensure 4 bytes anchor_id
                anchor_pose, anchor_distance = anchor_data.split("]=")
                anchor_poses.append([anchor_id, anchor_pose, anchor_distance])
            except ValueError:
                return [], None

        return anchor_poses, tag_pose

    def loop(self, verbose = False) :
        """
        Read and publish ranging data
        Parameters
        ----------
        verbose: bool
        Returns
        ----------
        """
        # read anchor data (always) and estimated tag pose (optional, done if location engine is enabled)
        anchor_data_list, tag_pose = self.getRangingData(verbose)
        anchor_id_list = []
        anchor_coord_list = []
        anchor_distance_list = []
        for anchor_data in anchor_data_list:
            anchor_id_list.append('DW' + anchor_data[0])
            anchor_coord_list.append(anchor_data[1].split(','))
            anchor_distance_list.append(anchor_data[2])
            
        if tag_pose is not None:
            # tag_pose has "est[x,y,z,quality]" format
            tag_pose = tag_pose[4:]
            tag_pose_x, tag_pose_y, tag_pose_z, _ = tag_pose.split(',')
            # Topic: PoseWitchCovariance
            pwc = PoseWithCovarianceStamped()
            pwc.header.stamp = rospy.get_rostime()
            pwc.header.frame_id = self.world_frame_id
            pwc.pose.pose.position.x = float(tag_pose_x)
            pwc.pose.pose.position.y = float(tag_pose_y)
            pwc.pose.pose.position.z = float(tag_pose_z)
            pub_pose_with_cov.publish(pwc)
            
            # Topic: PoseStamped
            ps = PoseStamped()
            ps.header.stamp = rospy.get_rostime()
            ps.header.frame_id = self.world_frame_id
            ps.pose.position = pwc.pose.pose.position
            ps.pose.orientation =  pwc.pose.pose.orientation
            pub_pose.publish(ps)

        # Topic: Anchors Info
        if anchor_data_list != []:
            if verbose:
                print(anchor_id_list)
            for self_anchor_id in self.anchor_id_list:
                dr = AnchorInfo()
                dr.header.stamp = rospy.get_rostime()
                dr.header.frame_id = self.world_frame_id
                dr.id = self_anchor_id
                dr.RSS = 0
                self_idx = self.anchor_id_list.index(dr.id)
                idx = None

                if dr.id in anchor_id_list:
                    try:
                        idx = anchor_id_list.index(dr.id)
                        dr.status = True
                        self.range_error_counts[self_idx] = 0

                        x, y, z = [float(coord) for coord in self.anchor_coord_list[self.anchor_id_list.index(dr.id)]] # forcing coords from cfg file
                        dr.position.x = x
                        dr.position.y = y
                        dr.position.z = z
                        dr.distance = float(anchor_distance_list[idx])

                        if self.visualize_anchors:
                            self.tf_broadcaster.sendTransform(  (x, y, z),
                                                                (0, 0, 0, 1), # orientation is not specified
                                                                rospy.Time.now(),
                                                                self_anchor_id,
                                                                self.world_frame_id)
                    except:
                        dr.status = False
                        self.range_error_counts[self_idx] += 1
                        if self.range_error_counts[self_idx] > 9:
                            self.range_error_counts[self_idx] = 0
                            #rospy.logerr("Anchor %d (%s) lost", i, dr.id)

                    #print('found anchor ' + str(dr.id) + ' with coords (' + str(dr.position.x) + ', ' + str(dr.position.y) + ', ' + str(dr.position.x) + ') and distance ' + str(dr.distance))
                else:
                    dr.status = False
                    self.range_error_counts[self_idx] += 1
                    if self.range_error_counts[self_idx] > 9:
                        self.range_error_counts[self_idx] = 0
                        #rospy.logerr("Anchor %d (%s) lost", i, dr.id)

                dr.child_frame_id = self_anchor_id
                pub_anchor_info[self_idx].publish(dr)
        elif verbose:
            print("Anchor data has not been received")

if __name__ == "__main__":

    rospy.init_node('dwm1001_node')

    # Read parameters
    tag_id = rospy.get_param('~tag_id')
    serial_port = '/dev/' + tag_id

    # find tag_id's network
    n_networks = int(rospy.get_param('~n_networks'))
    network_list = [rospy.get_param('~network' + str(i)) for i in range(n_networks)]
    network = None
    for network in network_list:
        if network['tag_id'] == tag_id: break

    # publish anchor as tf frame to visualize
    visualize_anchors = rospy.get_param('~visualize_anchors')
    world_frame_id = rospy.get_param('~world_frame_id', 'world')

    # Read anchor_id and its coordinates
    anchor_id_list = []
    anchor_coord_list = []
    for i in range(network['n_anchors']):
        anchor_id = network["anchor" + str(i) + "_id"]
        anchor_coord = network["anchor" + str(i) + "_coordinates"]
        anchor_id_list.append(anchor_id)
        anchor_coord_list.append(anchor_coord)
        print(anchor_coord)

    # Creating publishers
    pub_pose_with_cov = rospy.Publisher('~tag_pose_with_cov', PoseWithCovarianceStamped, queue_size=1)
    pub_pose = rospy.Publisher('~tag_pose', PoseStamped , queue_size=1)
    pub_anchor_info = []

    for i in range(network['n_anchors']):
        topic_name = "~anchor_info_" + str(i)
        pub_anchor_info.append(rospy.Publisher(topic_name, AnchorInfo, queue_size=1))

    # ROS rate
    rate = rospy.Rate(12.5)
    # Starting communication with DWM1001 module
    rdl = ReadyToLocalize(anchor_id_list, anchor_coord_list, world_frame_id, visualize_anchors)
    rdl.initSerial(serial_port)
    # set ranging mode
    rdl.les()
    
    #rdl.setup()
    while not rospy.is_shutdown():
        try:
            rdl.loop(verbose=True)
        except KeyboardInterrupt:
            rdl.handleKeyboardInterrupt()
        rate.sleep()

    rdl.quit()
