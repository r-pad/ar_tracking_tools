#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import os
import open3d 
import tf
import tf2_ros
from ar_track_alvar_msgs.msg import AlvarMarkers
import xml.etree.ElementTree as ET

def parseBundleXml(filename):
    tree = ET.parse(filename)
    root = tree.getroot()
    centers = {}
    corners = {}
    min_pts = []
    max_pts = []
    for marker in root: 
        idx = int(marker.get('index')) 
        corner_pts = [] 
        for corner in marker: 
            corner_pts.append([float(corner.get('x'))/100.0, 
                               float(corner.get('y'))/100.0, 
                               float(corner.get('z'))/100.0]) 
        corners[idx] = corner_pts
        centers[idx] = np.mean(corner_pts, axis=0) 
        min_pts.append(np.min(corner_pts, axis=0)) 
        max_pts.append(np.max(corner_pts, axis=0)) 
        interior = (np.max(min_pts, axis=0), np.min(max_pts, axis=0))
    return corners, centers, interior

class ARTagRANSAC(object):
    def __init__(self):
        rospy.init_node("ar_tag_ransac")
        self.bundle_file = rospy.get_param('~bundle_file')
        self.use_corners = rospy.get_param('~use_corners', True)
        self.frame_id = rospy.get_param('~output_frame', 'marker_bundle')

        self.max_corres_dist = rospy.get_param('~max_distance', 0.01)
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        self.marker_corners, self.marker_centers, self.marker_interior = parseBundleXml(self.bundle_file) 
        self.marker_ids = list(self.marker_centers.keys())
        self.marker_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.markersCallback, queue_size = 1) 
        #self.marker_sub = rospy.Subscriber('in_markers', AlvarMarkers, self.markersCallback, queue_size = 1) 

    def markersCallback(self, markers_msg):
        stamp = markers_msg.header.stamp
        confidence = []
        marker_pts = []
        bundle_pts = []
        for marker in markers_msg.markers:
            if(marker.id in self.marker_centers.keys()):
                q = marker.pose.pose.orientation
                pt = marker.pose.pose.position

                marker_pts.append([pt.x, pt.y, pt.z])
                bundle_pts.append(list(self.marker_centers[marker.id]))
                if(self.use_corners):
                    trans_mat = tf.transformations.quaternion_matrix([q.x,q.y,q.z,q.w])
                    trans_mat[:3,3] = [pt.x, pt.y, pt.z]
                    corners = np.concatenate([self.marker_corners[marker.id] - self.marker_centers[marker.id], 
                        np.ones([4,1])], axis=1).dot(trans_mat.T)
                    #marker_poses[marker.id] = trans_mat
                    #confidence.append(marker.confidence)
                    marker_pts.extend(corners[:,:3])
                    bundle_pts.extend(list(self.marker_corners[marker.id]))
         
        pts_src = open3d.PointCloud()
        pts_src.points = open3d.Vector3dVector(marker_pts) 
        pts_tgt = open3d.PointCloud()
        pts_tgt.points = open3d.Vector3dVector(bundle_pts)
        corres = open3d.Vector2iVector(np.tile(np.arange(len(marker_pts)),[2,1]).T)
        ransac_res = open3d.registration_ransac_based_on_correspondence(pts_src, pts_tgt, corres, self.max_corres_dist)
        trans_ransac = ransac_res.transformation

        self.tf_broadcaster.sendTransform(trans_ransac[:3,3],
                                          tf.transformations.quaternion_from_matrix(trans_ransac),
                                          marker.header.stamp,
                                          marker.header.frame_id,
                                          self.frame_id,
                                          )
def main():
    rospy.init_node("ar_tag_ransac") 
    obj_masker = ARTagRANSAC()
  
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down pose_labeler module")

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
   
