#ifndef ALVAR_BUNDLE_TRACKER_H
#define ALVAR_BUNDLE_TRACKER_H

#include <ar_track_alvar/CvTestbed.h>
#include <ar_track_alvar/MarkerDetector.h>
#include <ar_track_alvar/MultiMarkerBundle.h>
#include <ar_track_alvar/MultiMarkerInitializer.h>
#include <ar_track_alvar/filter/kinect_filtering.h>
#include <ar_track_alvar/filter/medianFilter.h>

#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>

#include "ar_tracking_tools/consensus_bundle_refinement.h"

using namespace alvar;
using namespace std;

typedef pcl::PointXYZRGB ARPoint;
typedef pcl::PointCloud<ARPoint> ARCloud;

tf::Transform 
poseToTF(Pose &p);

int
planeFitPoseImprovement(const ARCloud &corners_3D, 
                        ARCloud::Ptr selected_points, 
                        const ARCloud &cloud, 
                        Pose &p);

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
    using PointT = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;

  public:
    AlvarBundleTracker(ros::NodeHandle nh, 
                       ros::NodeHandle pnh, 
                       image_transport::ImageTransport it);

    void 
    getMultiMarkerPoses(IplImage *image);

    void
    getMultiMarkerPoses(IplImage *image, ARCloud &cloud);
    
    int
    inferCorners(const ARCloud &cloud, MultiMarkerBundle &master, ARCloud &bund_corners);
    
    void 
    imageCallback (const sensor_msgs::ImageConstPtr & image_msg);
    
    void 
    pointCloudCallback (const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

  protected:
    boost::shared_ptr<Camera> camera_ptr_;
    //ros::NodeHandle nh_;
    //image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber cloud_sub_;

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

    vector<boost::shared_ptr<ar_track_alvar::MedianFilter> > median_filts_;
    int median_filt_size_;

    bool use_ransac_;
    vector<ConsensusBundleRefinement> bundle_refiners_;
    bool ransac_use_corners_;
    double ransac_inlier_threshold_;
    int ransac_max_iterations_;
    bool ransac_refine_;
    double ransac_dense_spacing_;
   
    bool publish_marker_tf_;

    std::string output_frame_;
    int num_bundles_;
};


#endif