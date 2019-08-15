#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "ar_tracking_tools/alvar_bundle_tracker.h" 

int main(int argc, char *argv[])
{
    ros::init (argc, argv, "alvar_bundle_tracker");
    ros::NodeHandle nh, nh_p("~");
    image_transport::ImageTransport it(nh);

    AlvarBundleTracker tracker_node(nh, nh_p, it);
    
    ros::spin();

    return 0;
}