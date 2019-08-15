#include <ar_tracking_tools/alvar_bundle_tracker.h>
#include <stdexcept>

tf::Transform
poseToTF(Pose &p)
{
    double px,py,pz,qx,qy,qz,qw;
    
    px = p.translation[0]/100.0;
    py = p.translation[1]/100.0;
    pz = p.translation[2]/100.0;
    qx = p.quaternion[1];
    qy = p.quaternion[2];
    qz = p.quaternion[3];
    qw = p.quaternion[0];

    //Get the marker pose in the camera frame
    tf::Quaternion rotation (qx,qy,qz,qw);
    tf::Vector3 origin (px,py,pz);
    tf::Transform marker_pose(rotation, origin);

    return marker_pose;
}

visualization_msgs::Marker
makeRvizMarkerMsg(int bundle_id, int id, double marker_size, 
                  tf::Transform &marker_pose, 
                  sensor_msgs::ImageConstPtr image_msg)
{
    visualization_msgs::Marker rviz_marker;
    
    tf::poseTFToMsg (marker_pose, rviz_marker.pose);
    rviz_marker.header.frame_id = image_msg->header.frame_id;
    rviz_marker.header.stamp = image_msg->header.stamp;
    rviz_marker.id = id;

    rviz_marker.scale.x = 1.0 * marker_size/100.0;
    rviz_marker.scale.y = 1.0 * marker_size/100.0;
    rviz_marker.scale.z = 0.2 * marker_size/100.0;

    if(bundle_id>=0)
    {
        rviz_marker.ns = "main_shapes";
    }
    else
    {
        rviz_marker.ns = "basic_shapes";
    }

    rviz_marker.type = visualization_msgs::Marker::CUBE;
    rviz_marker.action = visualization_msgs::Marker::ADD;

    if(bundle_id >= 0)
    {
        rviz_marker.color.r = 1.0f;
        rviz_marker.color.g = 0.0f;
        rviz_marker.color.b = 0.0f;
        rviz_marker.color.a = 1.0;
    }
    else
    {
        rviz_marker.color.r = 0.5f;
        rviz_marker.color.g = 0.5f;
        rviz_marker.color.b = 0.5f;
        rviz_marker.color.a = 0.5;
    }

    rviz_marker.lifetime = ros::Duration (1.0);
    return rviz_marker;
}


ar_track_alvar_msgs::AlvarMarker 
makeAlvarMarkerMsg(int id, 
                   tf::Transform &marker_pose,
                   sensor_msgs::ImageConstPtr image_msg, 
                   tf::StampedTransform &cam_to_output)
{
    ar_track_alvar_msgs::AlvarMarker ar_pose_marker;
    
    tf::Transform ar_pose_output = cam_to_output * marker_pose;
    tf::poseTFToMsg (ar_pose_output, ar_pose_marker.pose.pose);
    
    if(cam_to_output.frame_id_.size() > 0)
    {
        ar_pose_marker.header.frame_id = cam_to_output.frame_id_;
    }
    else
    {
        ar_pose_marker.header.frame_id = image_msg->header.frame_id;
    }

    ar_pose_marker.header.stamp = image_msg->header.stamp;
    ar_pose_marker.id = id;
    
    return ar_pose_marker;
}

void 
AlvarBundleTracker::getMultiMarkerPoses(IplImage *image)
{
    if (marker_detector_.Detect(image, camera_ptr_.get(), true, false, 
                                max_new_marker_error_, 
                                max_track_error_, CVSEQ, true))
    {
        for(size_t i=0; i<num_bundles_; i++)
        {
            multi_marker_bundles_[i]->Update(marker_detector_.markers, 
                                             camera_ptr_.get(), bundle_poses_[i]);
        }

        if(marker_detector_.DetectAdditional(image, camera_ptr_.get(), false) > 0)
        {
            for(size_t i=0; i<num_bundles_; i++)
            {
                if ((multi_marker_bundles_[i]->SetTrackMarkers(marker_detector_, camera_ptr_.get(), 
                                                               bundle_poses_[i], image) > 0))
                {
                    multi_marker_bundles_[i]->Update(marker_detector_.markers, 
                                                     camera_ptr_.get(), bundle_poses_[i]);
                }
            }
        }
    }
}

void 
AlvarBundleTracker::imageCallback (const sensor_msgs::ImageConstPtr & image_msg)
{    
    if(camera_ptr_->getCamInfo_)
    {
        
        tf::StampedTransform cam_to_output;
        try
        {
            if(output_frame_.size() > 0)
            {
                tf_listener_ptr_->waitForTransform(output_frame_, image_msg->header.frame_id, 
                                                   image_msg->header.stamp, ros::Duration(1.0));
                tf_listener_ptr_->lookupTransform(output_frame_, image_msg->header.frame_id, 
                                                  image_msg->header.stamp, cam_to_output);
            }
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
        }
        
        cv_bridge::CvImagePtr img_ptr;
        try
        {
            img_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR ("Could not convert from '%s' to 'rgb8'.", image_msg->encoding.c_str ());
            return;
        }

        ar_track_alvar_msgs::AlvarMarkers ar_pose_markers;

        IplImage ipl_image = img_ptr->image;
        getMultiMarkerPoses(&ipl_image);

        vector<bool> bundles_seen(num_bundles_, false);

        for (size_t i=0; i<marker_detector_.markers->size(); i++)
        {
            int id = (*(marker_detector_.markers))[i].GetId();
            
            int bundle_id = -1;
            
            if(id >= 0)
            {
                for(int j=0; j<num_bundles_; j++)
                {
                    for(size_t k=0; k<bundle_indices_[j].size(); k++)
                    {
                        if(bundle_indices_[j][k] == id)
                        {
                            bundle_id = j;
                            bundles_seen[j] = true;
                            break;
                        }
                    }
                }

                Pose p = (*(marker_detector_.markers))[i].pose;
                
                double marker_size = default_marker_size_;
                if(bundle_marker_sizes_.find(id)!=bundle_marker_sizes_.end())
                {
                    marker_size = bundle_marker_sizes_[id];
                }
                
                tf::Transform marker_pose = poseToTF(p);
                visualization_msgs::Marker rviz_marker = 
                    makeRvizMarkerMsg(bundle_id, id, marker_size, marker_pose, image_msg);
                rviz_marker_pub_.publish(rviz_marker);
                
                if(bundle_id >= 0 && publish_marker_tf_)
                {
                    std::string marker_frame = "ar_marker_" + to_string(id);
                    tf::StampedTransform cam_to_marker(marker_pose, image_msg->header.stamp, 
                                                       image_msg->header.frame_id, marker_frame.c_str());
                    
                    tf_broadcaster_ptr_->sendTransform(cam_to_marker);
                    ar_track_alvar_msgs::AlvarMarker tag_marker_msg = makeAlvarMarkerMsg(id, marker_pose, 
                                                                          image_msg, cam_to_output);

                    ar_pose_markers.markers.push_back(tag_marker_msg);
                }
            }
        }

        for(size_t i=0; i<num_bundles_; i++)
        {
            if(bundles_seen[i])
            {
                ar_track_alvar_msgs::AlvarMarker bundle_pose_marker;
                tf::Transform bundle_pose = poseToTF(bundle_poses_[i]);
                tf::StampedTransform cam_to_bundle(bundle_pose, 
                                                   image_msg->header.stamp, 
                                                   image_msg->header.frame_id, 
                                                   bundle_names_[i].c_str());
                tf_broadcaster_ptr_->sendTransform(cam_to_bundle);
                ar_track_alvar_msgs::AlvarMarker bundle_marker_msg = makeAlvarMarkerMsg(-i, bundle_pose, image_msg, cam_to_output);
                ar_pose_markers.markers.push_back(bundle_marker_msg);
            }
        }

        ar_marker_pub_.publish (ar_pose_markers);
    }
}
 
AlvarBundleTracker::AlvarBundleTracker(ros::NodeHandle nh, 
                                       ros::NodeHandle nh_p, 
                                       image_transport::ImageTransport it)
{
    nh_p.param("default_marker_size", default_marker_size_, 5.0);
    nh_p.param("publish_marker_tf", publish_marker_tf_, false);
    nh_p.param("max_new_marker_error", max_new_marker_error_, 0.2);
    nh_p.param("max_track_error", max_track_error_, 0.2);
    nh_p.param<string>("output_frame", output_frame_, "");
      
    //pn.param("image_topic", image_topic);
    //pn.param("info_topic", info_topic);
    string bundle_filenames_str;
    nh_p.param<string>("bundle_filenames", bundle_filenames_str, "");
    vector<string> bundle_filenames;
    boost::split(bundle_filenames, bundle_filenames_str, boost::is_any_of(","));
    num_bundles_ = bundle_filenames.size();
    
    marker_detector_.SetMarkerSize(default_marker_size_);
    
    multi_marker_bundles_.resize(num_bundles_);
    bundle_poses_.resize(num_bundles_);
    master_ids_.resize(num_bundles_);
    bundle_indices_.resize(num_bundles_); 

    for(size_t i=0; i<num_bundles_; i++)
    {
        bundle_poses_[i].Reset();
        MultiMarker load_helper;
        string xml_filename = bundle_filenames[i];
        if(load_helper.Load(xml_filename.c_str(), FILE_FORMAT_XML))
        {
            size_t fn_start = xml_filename.find_last_of("/\\")+1;
            size_t fn_len = xml_filename.find_last_of(".") - fn_start;
            bundle_names_.push_back(xml_filename.substr(fn_start, fn_len));
            vector<int> id_vector = load_helper.getIndices();
            for(int j=0; j<id_vector.size(); j++)
            {
                CvPoint3D64f corner1 = load_helper.pointcloud[load_helper.pointcloud_index(id_vector[j], 0)];
                CvPoint3D64f corner2 = load_helper.pointcloud[load_helper.pointcloud_index(id_vector[j], 1)];

                double edge_length = sqrt(pow(corner1.x - corner2.x, 2) 
                      + pow(corner1.y - corner2.y, 2) 
                      + pow(corner1.z - corner2.z, 2)); 
                marker_detector_.SetMarkerSizeForId(id_vector[j], edge_length);
                bundle_marker_sizes_[id_vector[j]] = edge_length;
            }

            multi_marker_bundles_[i] = boost::shared_ptr<MultiMarkerBundle>(new MultiMarkerBundle(id_vector));
            multi_marker_bundles_[i]->Load(xml_filename.c_str(), FILE_FORMAT_XML);
            master_ids_[i] = multi_marker_bundles_[i]->getMasterId();
            bundle_indices_[i] = multi_marker_bundles_[i]->getIndices();
        }
        else
        {
            ROS_ERROR("Cannot load file %s", xml_filename.c_str());
            throw std::runtime_error("Could not open file " + xml_filename);
        }
    }  

    camera_ptr_ = boost::shared_ptr<Camera>(new Camera(nh, "in_camera_info"));
    
    tf_listener_ptr_ = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener(nh));
    tf_broadcaster_ptr_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());
    
    ar_marker_pub_ = nh.advertise < ar_track_alvar_msgs::AlvarMarkers > ("ar_pose_marker", 0);
    rviz_marker_pub_ = nh.advertise < visualization_msgs::Marker > ("visualization_marker", 0);
    
    image_sub_ = it.subscribe ("in_camera_image", 1, &AlvarBundleTracker::imageCallback, this);
    
}