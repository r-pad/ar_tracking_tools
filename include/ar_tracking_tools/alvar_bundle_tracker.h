#ifndef ALVAR_BUNDLE_TRACKER_H
#define ALVAR_BUNDLE_TRACKER_H

#include "ar_track_alvar/CvTestbed.h"
#include "ar_track_alvar/MarkerDetector.h"
#include "ar_track_alvar/MultiMarkerBundle.h"
#include "ar_track_alvar/MultiMarkerInitializer.h"

#include <cv_bridge/cv_bridge.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>

#include <boost/shared_ptr.hpp>

#include <tf/tf.h>

using namespace alvar;
using namespace std;

tf::Transform 
poseToTF(Pose &p);

visualization_msgs::Marker 
makeRvizMarkerMsg(int bundle_id, int id, double marker_size,
                  tf::Transform &marker_pose, 
                  sensor_msgs::ImageConstPtr image_msg);

ar_track_alvar_msgs::AlvarMarker 
makeAlvarMarkerMsg(int id, 
                   tf::Transform &marker_pose,
                   sensor_msgs::ImageConstPtr image_msg, 
                   tf::StampedTransform &cam_to_output);

class AlvarBundleTracker
{
  public:
    AlvarBundleTracker(ros::NodeHandle nh, 
                       ros::NodeHandle nh_p, 
                       image_transport::ImageTransport it);

    void 
    getMultiMarkerPoses(IplImage *image);
    
    void 
    imageCallback (const sensor_msgs::ImageConstPtr & image_msg);
    
    void 
    getPointCloudCallback (const sensor_msgs::PointCloud2ConstPtr &msg);

  protected:
    boost::shared_ptr<Camera> camera_ptr_;
    //ros::NodeHandle nh_;
    //image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
        
    ros::Publisher ar_marker_pub_;
    ros::Publisher rviz_marker_pub_;
    
    boost::shared_ptr<tf::TransformListener> tf_listener_ptr_;
    boost::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_ptr_;
    
    
    MarkerDetector<MarkerData> marker_detector_;
    double max_new_marker_error_;
    double max_track_error_;
        
    double default_marker_size_;
    map<int, double> bundle_marker_sizes_;
    vector<string> bundle_names_;
    
    vector<boost::shared_ptr<MultiMarkerBundle> > multi_marker_bundles_;
    vector<Pose> bundle_poses_;
    vector<int> master_ids_;
    std::vector<std::vector<int> > bundle_indices_;
    
   
    bool publish_marker_tf_;

    std::string image_topic_; 
    std::string info_topic_; 
    std::string output_frame_;
    int num_bundles_;
};


#endif