#include <ar_tracking_tools/alvar_bundle_tracker.h>
#include <tf_conversions/tf_eigen.h>
#include <stdexcept>
#include "ar_tracking_tools/colors.h"
#include <boost/algorithm/string/trim.hpp>

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
        Color color = getColor(bundle_id);
        rviz_marker.color.r = color.r/255.0;
        rviz_marker.color.g = color.g/255.0;
        rviz_marker.color.b = color.b/255.0;
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

int
planeFitPoseImprovement(const ARCloud &corners_3D, 
                        ARCloud::Ptr selected_points, 
                        const ARCloud &cloud, 
                        Pose &p)
{
    ar_track_alvar::PlaneFitResult res = ar_track_alvar::fitPlane(selected_points);
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = pcl_conversions::fromPCL(cloud.header).stamp;
    pose.header.frame_id = cloud.header.frame_id;
    pose.pose.position = ar_track_alvar::centroid(*res.inliers);

    //draw3dPoints(selected_points, cloud.header.frame_id, 1, id, 0.005);

    //Get 2 points that point forward in marker x direction   
    int i1,i2;
    if(isnan(corners_3D[0].x) || isnan(corners_3D[0].y) || isnan(corners_3D[0].z) || 
       isnan(corners_3D[3].x) || isnan(corners_3D[3].y) || isnan(corners_3D[3].z))
    {
        if(isnan(corners_3D[1].x) || isnan(corners_3D[1].y) || isnan(corners_3D[1].z) || 
           isnan(corners_3D[2].x) || isnan(corners_3D[2].y) || isnan(corners_3D[2].z))
        {
            return -1;
        }
        else
        {
            i1 = 1;
            i2 = 2;
        }
    }
    else
    {
        i1 = 0;
        i2 = 3;
    }

    //Get 2 points the point forward in marker y direction   
    int i3,i4;
    if(isnan(corners_3D[0].x) || isnan(corners_3D[0].y) || isnan(corners_3D[0].z) || 
       isnan(corners_3D[1].x) || isnan(corners_3D[1].y) || isnan(corners_3D[1].z))
    {   
        if(isnan(corners_3D[3].x) || isnan(corners_3D[3].y) || isnan(corners_3D[3].z) || 
           isnan(corners_3D[2].x) || isnan(corners_3D[2].y) || isnan(corners_3D[2].z))
        {   
            return -1;
        }
        else
        {
            i3 = 2;
            i4 = 3;
        }
    }
    else
    {
        i3 = 1;
        i4 = 0;
    }

    ARCloud::Ptr orient_points(new ARCloud());
    orient_points->points.push_back(corners_3D[i1]);
    //draw3dPoints(orient_points, cloud.header.frame_id, 3, id+1000, 0.008);

    orient_points->clear();
    orient_points->points.push_back(corners_3D[i2]);
    //draw3dPoints(orient_points, cloud.header.frame_id, 2, id+2000, 0.008);

    int succ;
    succ = ar_track_alvar::extractOrientation(res.coeffs, corners_3D[i1], corners_3D[i2], 
                                              corners_3D[i3], corners_3D[i4], pose.pose.orientation);
    if(succ < 0) 
    {
        return -1;
    }

    tf::Matrix3x3 mat;
    succ = ar_track_alvar::extractFrame(res.coeffs, corners_3D[i1], corners_3D[i2], 
                                        corners_3D[i3], corners_3D[i4], mat);
    if(succ < 0) 
    {
        return -1;
    }

    //drawArrow(pose.pose.position, mat, cloud.header.frame_id, 1, id);

    p.translation[0] = pose.pose.position.x * 100.0;
    p.translation[1] = pose.pose.position.y * 100.0;
    p.translation[2] = pose.pose.position.z * 100.0;
    p.quaternion[1] = pose.pose.orientation.x;
    p.quaternion[2] = pose.pose.orientation.y;
    p.quaternion[3] = pose.pose.orientation.z;
    p.quaternion[0] = pose.pose.orientation.w; 

    return 0;
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

int 
makeMasterTransform (const CvPoint3D64f& p0, const CvPoint3D64f& p1,
                     const CvPoint3D64f& p2, const CvPoint3D64f& p3,
                     tf::Transform &retT)
{
    const tf::Vector3 q0(p0.x, p0.y, p0.z);
    const tf::Vector3 q1(p1.x, p1.y, p1.z);
    const tf::Vector3 q2(p2.x, p2.y, p2.z);
    const tf::Vector3 q3(p3.x, p3.y, p3.z);
  
    // (inverse) matrix with the given properties
    const tf::Vector3 v = (q1-q0).normalized();
    const tf::Vector3 w = (q2-q1).normalized();
    const tf::Vector3 n = v.cross(w);
    tf::Matrix3x3 m(v[0], v[1], v[2], w[0], w[1], w[2], n[0], n[1], n[2]);
    m = m.inverse();
    
    //Translate to quaternion
    if(m.determinant() <= 0)
        return -1;
  
    //Use Eigen for this part instead, because the ROS version of bullet appears to have a bug
    Eigen::Matrix3f eig_m;
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            eig_m(i,j) = m[i][j];
        }
    }
    Eigen::Quaternion<float> eig_quat(eig_m);
    
    // Translate back to bullet
    tfScalar ex = eig_quat.x();
    tfScalar ey = eig_quat.y();
    tfScalar ez = eig_quat.z();
    tfScalar ew = eig_quat.w();
    tf::Quaternion quat(ex,ey,ez,ew);
    quat = quat.normalized();
    
    double qx = (q0.x() + q1.x() + q2.x() + q3.x()) / 4.0;
    double qy = (q0.y() + q1.y() + q2.y() + q3.y()) / 4.0;
    double qz = (q0.z() + q1.z() + q2.z() + q3.z()) / 4.0;
    tf::Vector3 origin (qx,qy,qz);
    
    tf::Transform tform (quat, origin);  //transform from master to marker
    retT = tform;
    
    return 0;
}

int 
calcAndSaveMasterCoords(MultiMarkerBundle &master)
{
    int mast_id = master.master_id;
    std::vector<tf::Vector3> rel_corner_coords;
    
    //Go through all the markers associated with this bundle
    for (size_t i=0; i<master.marker_indices.size(); i++){
        int mark_id = master.marker_indices[i];
        rel_corner_coords.clear();
        
        //Get the coords of the corners of the child marker in the master frame
        CvPoint3D64f mark_corners[4];
        for(int j=0; j<4; j++){
            mark_corners[j] = master.pointcloud[master.pointcloud_index(mark_id, j)];
        }
        
        //Use them to find a transform from the master frame to the child frame
        tf::Transform tform;
        makeMasterTransform(mark_corners[0], mark_corners[1], mark_corners[2], mark_corners[3], tform);
    
        //Finally, find the coords of the corners of the master in the child frame
        for(int j=0; j<4; j++){
            
            CvPoint3D64f corner_coord = master.pointcloud[master.pointcloud_index(mast_id, j)];
            double px = corner_coord.x;
            double py = corner_coord.y;
            double pz = corner_coord.z;
        
            tf::Vector3 corner_vec (px, py, pz);
            tf::Vector3 ans = (tform.inverse()) * corner_vec;
            rel_corner_coords.push_back(ans);
        }
        
        master.rel_corners.push_back(rel_corner_coords);
    }
    
    return 0;
}

int 
AlvarBundleTracker::inferCorners(const ARCloud &cloud, MultiMarkerBundle &master, ARCloud &bund_corners)
{
    bund_corners.clear();
    bund_corners.resize(4);
    for(int i=0; i<4; i++)
    {
        bund_corners[i].x = 0;
        bund_corners[i].y = 0;
        bund_corners[i].z = 0;
    }

    for (size_t i=0; i<master.marker_status.size(); i++) 
    {
        if (master.marker_status[i] > 0)
        {
            master.marker_status[i]=1;
        }
    }

    int n_est = 0;

    for (size_t i=0; i<marker_detector_.markers->size(); i++)
    {
        const Marker* marker = &((*marker_detector_.markers)[i]);
        int id = marker->GetId();
        int index = master.get_id_index(id);
        int mast_id = master.master_id;
        if (index < 0) 
        {
          continue;
        }

        if (master.marker_status[index] > 0 && marker->valid)
        {
            n_est++;

            std::string marker_frame = "ar_marker_";
            std::stringstream mark_out;
            mark_out << id;
            std::string id_string = mark_out.str();
            marker_frame += id_string;
             
            for(int j = 0; j < 4; ++j)
            {
                tf::Vector3 corner_coord = master.rel_corners[index][j];
                geometry_msgs::PointStamped p, output_p;
                p.header.frame_id = marker_frame;
                p.point.x = corner_coord.y()/100.0;  
                p.point.y = -corner_coord.x()/100.0;
                p.point.z = corner_coord.z()/100.0;
                
                try
                {
                    tf_listener_ptr_->waitForTransform(cloud.header.frame_id, marker_frame, ros::Time(0), ros::Duration(0.1));
                    tf_listener_ptr_->transformPoint(cloud.header.frame_id, p, output_p);
                }
                catch (tf::TransformException ex)
                {
                    ROS_ERROR("ERROR InferCorners: %s",ex.what());
                    return -1;
                }

                bund_corners[j].x += output_p.point.x;
                bund_corners[j].y += output_p.point.y;
                bund_corners[j].z += output_p.point.z;
            }
            master.marker_status[index] = 2;
        }
    }
  
    if(n_est > 0)
    {
        for(int i=0; i<4; i++)
        {
            bund_corners[i].x /= n_est;
            bund_corners[i].y /= n_est;
            bund_corners[i].z /= n_est;
        }
    }

    return 0;
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
				
				Pose ret_pose;
                if(median_filt_size_ > 0)
                {
                    median_filts_[i]->addPose(bundle_poses_[i]);
                    median_filts_[i]->getMedian(ret_pose);
                    bundle_poses_[i] = ret_pose;
                }

            }
        }
    }
}

void 
AlvarBundleTracker::getMultiMarkerPoses(IplImage *image, ARCloud &cloud)
{
    if(marker_detector_.Detect(image, camera_ptr_.get(), true, false, 
                               max_new_marker_error_, 
                               max_track_error_, CVSEQ, true)) 
    {
        vector<bool> bundles_seen(num_bundles_, false);

        for (size_t i=0; i<marker_detector_.markers->size(); i++)
        {
            vector<cv::Point, Eigen::aligned_allocator<cv::Point> > pixels;
            Marker *m = &((*marker_detector_.markers)[i]);
            int id = m->GetId();
            int resol = m->GetRes();
            int ori = m->ros_orientation;
      
            PointDouble pt1, pt2, pt3, pt4;
            pt4 = m->ros_marker_points_img[0];
            pt3 = m->ros_marker_points_img[resol-1];
            pt1 = m->ros_marker_points_img[(resol*resol)-resol];
            pt2 = m->ros_marker_points_img[(resol*resol)-1];

            m->ros_corners_3D[0] = cloud(pt1.x, pt1.y);
            m->ros_corners_3D[1] = cloud(pt2.x, pt2.y);
            m->ros_corners_3D[2] = cloud(pt3.x, pt3.y);
            m->ros_corners_3D[3] = cloud(pt4.x, pt4.y);

            if(ori >= 0 && ori < 4)
            {
                if(ori != 0){
                  std::rotate(m->ros_corners_3D.begin(), m->ros_corners_3D.begin() + ori, m->ros_corners_3D.end());
                }
            }
            else
            {
                ROS_ERROR("FindMarkerBundles: Bad Orientation: %i for ID: %i", ori, id);
            }


            int bundle_id = -1;
            for(int j=0; j<num_bundles_; j++)
            {
                for(int k=0; k<bundle_indices_[j].size(); k++)
                {
                    if(bundle_indices_[j][k] == id)
                    {
                        bundle_id = j;
                        break;
                    }
                }
            }

            BOOST_FOREACH (const PointDouble& p, m->ros_marker_points_img)
            {
                pixels.push_back(cv::Point(p.x, p.y));
            }
            ARCloud::Ptr selected_points = ar_track_alvar::filterCloud(cloud, pixels);

            if(planeFitPoseImprovement(m->ros_corners_3D, selected_points, cloud, m->pose) < 0)
            {
                m->valid = false;
            }
            else if(bundle_id >= 0)
            {
                m->valid = true;
                bundles_seen[bundle_id] = true;
            }     
                
        }
            
        ARCloud inferred_corners;
        for(int i=0; i<num_bundles_; i++)
        {
            if(bundles_seen[i])
            {
                if(inferCorners(cloud, *(multi_marker_bundles_[i]), inferred_corners) >= 0)
                {
                    ARCloud::Ptr inferred_cloud(new ARCloud(inferred_corners));
                    planeFitPoseImprovement(inferred_corners, inferred_cloud, cloud, bundle_poses_[i]);
                }

                Pose ret_pose;
                if(median_filt_size_ > 0)
                {
                    median_filts_[i]->addPose(bundle_poses_[i]);
                    median_filts_[i]->getMedian(ret_pose);
                    bundle_poses_[i] = ret_pose;
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
            else
            {
                cam_to_output.setData(tf::Transform::getIdentity());
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
        visualization_msgs::MarkerArray rviz_pose_markers;

        IplImage ipl_image = img_ptr->image;
        getMultiMarkerPoses(&ipl_image);

        vector<bool> bundles_seen(num_bundles_, false);
        std::map<int, Eigen::Matrix4f> tag_transforms;
        
        for (size_t i=0; i<marker_detector_.markers->size(); i++)
        {
            int id = (*(marker_detector_.markers))[i].GetId();
            
            if(id >= 0)
            {
                int bundle_id = -1;
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
                rviz_pose_markers.markers.push_back(rviz_marker);
                //rviz_marker_pub_.publish(rviz_marker);
                if(use_ransac_)
                {
                    Eigen::Affine3d marker_affine;
                    tf::transformTFToEigen(marker_pose, marker_affine);
                    tag_transforms.insert(pair<int, Eigen::Matrix4f>(id, marker_affine.cast<float>().matrix()));
                }
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
                if(use_ransac_)
                {
                    Eigen::Affine3f refined_transform;
                    if(bundle_refiners_[i].refine(tag_transforms, refined_transform.matrix()))
                    {
                        tf:poseEigenToTF(refined_transform.cast<double>(), bundle_pose);
                    }
                }
                
                tf::StampedTransform cam_to_bundle(bundle_pose, 
                                                   image_msg->header.stamp, 
                                                   image_msg->header.frame_id, 
                                                   bundle_names_[i].c_str());
                tf_broadcaster_ptr_->sendTransform(cam_to_bundle);
                ar_track_alvar_msgs::AlvarMarker bundle_marker_msg = makeAlvarMarkerMsg(-i-1, bundle_pose, image_msg, cam_to_output);
                ar_pose_markers.markers.push_back(bundle_marker_msg);
            }
        }

        ar_marker_pub_.publish (ar_pose_markers);
        rviz_marker_pub_.publish(rviz_pose_markers);
    }
}

void 
AlvarBundleTracker::pointCloudCallback (const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    if(camera_ptr_->getCamInfo_)
    {
        tf::StampedTransform cam_to_output;
        try
        {
            if(output_frame_.size() > 0)
            {
                tf_listener_ptr_->waitForTransform(output_frame_, cloud_msg->header.frame_id, 
                                                   cloud_msg->header.stamp, ros::Duration(1.0));
                tf_listener_ptr_->lookupTransform(output_frame_, cloud_msg->header.frame_id, 
                                                  cloud_msg->header.stamp, cam_to_output);
            }
            else
            {
                cam_to_output.setData(tf::Transform::getIdentity());
            }
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
        }

        ar_track_alvar_msgs::AlvarMarkers ar_pose_markers;
        visualization_msgs::MarkerArray rviz_pose_markers;

        ARCloud cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);

        sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);
        pcl::toROSMsg (*cloud_msg, *image_msg);
        image_msg->header.stamp = cloud_msg->header.stamp;
        image_msg->header.frame_id = cloud_msg->header.frame_id;
            
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
        
        IplImage ipl_image = img_ptr->image;
        getMultiMarkerPoses(&ipl_image, cloud);
        
        vector<bool> bundles_seen(num_bundles_, false);
        std::map<int, Eigen::Matrix4f> tag_transforms;
        
        for (size_t i=0; i<marker_detector_.markers->size(); i++)
        {
            int id = (*(marker_detector_.markers))[i].GetId();
            
            if(id >= 0)
            {
                int bundle_id = -1;
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
                
                if((*(marker_detector_.markers))[i].valid)
                {
                    Pose p = (*(marker_detector_.markers))[i].pose;
                    double marker_size = default_marker_size_;
                    if(bundle_marker_sizes_.find(id)!=bundle_marker_sizes_.end())
                    {
                        marker_size = bundle_marker_sizes_[id];
                    }

                    tf::Transform marker_pose = poseToTF(p);
                    visualization_msgs::Marker rviz_marker = 
                        makeRvizMarkerMsg(bundle_id, id, marker_size, marker_pose, image_msg);
                    rviz_pose_markers.markers.push_back(rviz_marker);
                    //rviz_marker_pub_.publish(rviz_marker);
                    if(use_ransac_)
                    {
                        Eigen::Affine3d marker_affine;
                        tf::transformTFToEigen(marker_pose, marker_affine);
                        tag_transforms.insert(pair<int, Eigen::Matrix4f>(id, marker_affine.cast<float>().matrix()));
                    }
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
        }
        
		for(size_t i=0; i<num_bundles_; i++)
        {
            if(bundles_seen[i])
            {
                ar_track_alvar_msgs::AlvarMarker bundle_pose_marker;
                tf::Transform bundle_pose = poseToTF(bundle_poses_[i]);
                
                if(use_ransac_)
                {
                    Eigen::Affine3f refined_transform;
                    if(bundle_refiners_[i].refine(tag_transforms, refined_transform.matrix()))
                    {
                        tf:poseEigenToTF(refined_transform.cast<double>(), bundle_pose);
                    }
                }
                
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
        rviz_marker_pub_.publish(rviz_pose_markers);
    }
}

AlvarBundleTracker::AlvarBundleTracker(ros::NodeHandle nh, 
                                       ros::NodeHandle pnh, 
                                       image_transport::ImageTransport it)
{
    pnh.param("default_marker_size", default_marker_size_, 5.0);
    pnh.param("publish_marker_tf", publish_marker_tf_, false);
    pnh.param("max_new_marker_error", max_new_marker_error_, 0.2);
    pnh.param("max_track_error", max_track_error_, 0.2);
    pnh.param<string>("output_frame", output_frame_, "");
    
    bool use_depth;
    pnh.param("use_depth", use_depth, false);
    pnh.param("median_filt_size", median_filt_size_, 10);
    
    pnh.param("use_ransac", use_ransac_, false);
    pnh.param("ransac_use_corners", ransac_use_corners_, true);
    pnh.param("ransac_inlier_threshold", ransac_inlier_threshold_, 0.05);
    pnh.param("ransac_max_iterations", ransac_max_iterations_, 1000);
    pnh.param("ransac_refine", ransac_refine_, true);
    pnh.param("ransac_dense_spacing", ransac_dense_spacing_, 0.0);
    ransac_use_corners_ = ransac_use_corners_ || ransac_dense_spacing_ > 0.0;
    
    string bundle_filenames_str;
    pnh.param<string>("bundle_filenames", bundle_filenames_str, "");
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
        boost::algorithm::trim(xml_filename);
        if(xml_filename.size() > 0 && load_helper.Load(xml_filename.c_str(), FILE_FORMAT_XML))
        {
            size_t fn_start = xml_filename.find_last_of("/\\")+1;
            size_t fn_len = xml_filename.find_last_of(".") - fn_start;
            bundle_names_.push_back(xml_filename.substr(fn_start, fn_len));
            vector<int> id_vector = load_helper.getIndices();
            
            std::vector<PointCloudPtr> tag_points;
            std::vector<double> tag_sizes;
            
            for(int j=0; j<id_vector.size(); j++)
            {
                CvPoint3D64f corner1 = load_helper.pointcloud[load_helper.pointcloud_index(id_vector[j], 0)];
                CvPoint3D64f corner2 = load_helper.pointcloud[load_helper.pointcloud_index(id_vector[j], 1)];
                CvPoint3D64f corner3 = load_helper.pointcloud[load_helper.pointcloud_index(id_vector[j], 2)];
                CvPoint3D64f corner4 = load_helper.pointcloud[load_helper.pointcloud_index(id_vector[j], 3)];
                
                double edge_length = sqrt(pow(corner1.x - corner2.x, 2) 
                      + pow(corner1.y - corner2.y, 2) 
                      + pow(corner1.z - corner2.z, 2)); 
                marker_detector_.SetMarkerSizeForId(id_vector[j], edge_length);
                bundle_marker_sizes_[id_vector[j]] = edge_length;
                
                if(use_ransac_)
                {
                    PointCloudPtr tag_pts(new PointCloud());
                    tag_pts->points.push_back(PointT((corner1.x + corner2.x + corner3.x + corner4.x)/400.,
                                                     (corner1.y + corner2.y + corner3.y + corner4.y)/400.,
                                                     (corner1.z + corner2.z + corner3.z + corner4.z)/400.));
                    
                    if(ransac_use_corners_)
                    {
                        tag_pts->points.push_back(PointT(corner1.x/100., corner1.y/100., corner1.z/100.));
                        tag_pts->points.push_back(PointT(corner2.x/100., corner2.y/100., corner2.z/100.));
                        tag_pts->points.push_back(PointT(corner3.x/100., corner3.y/100., corner3.z/100.));
                        tag_pts->points.push_back(PointT(corner4.x/100., corner4.y/100., corner4.z/100.));
                    }
                    if(ransac_dense_spacing_ > 0)
                    {
                        std::pair<int, int> dense_size = ConsensusBundleRefinement::createDenseTagCloud(*tag_pts, *tag_pts, 
                                                                                                        ransac_dense_spacing_);
                        tag_pts->width = dense_size.first;
                        tag_pts->height = dense_size.second;
                    }
                    tag_points.push_back(tag_pts);
                    tag_sizes.push_back(edge_length);
                }
            }
            if(use_ransac_)
            {
                bundle_refiners_.push_back(ConsensusBundleRefinement(id_vector, tag_points, tag_sizes, 
                                                                     ransac_use_corners_,
                                                                     ransac_inlier_threshold_, 
                                                                     ransac_max_iterations_, 
                                                                     ransac_refine_,
                                                                     ransac_dense_spacing_));
            }
            multi_marker_bundles_[i] = boost::shared_ptr<MultiMarkerBundle>(new MultiMarkerBundle(id_vector));
            multi_marker_bundles_[i]->Load(xml_filename.c_str(), FILE_FORMAT_XML);
            master_ids_[i] = multi_marker_bundles_[i]->getMasterId();
            bundle_indices_[i] = multi_marker_bundles_[i]->getIndices();
            if(use_depth)
            {
                calcAndSaveMasterCoords(*(multi_marker_bundles_[i]));
            }
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
    rviz_marker_pub_ = nh.advertise < visualization_msgs::MarkerArray > ("visualization_marker_array", 0);
	
	if(median_filt_size_ > 0)
	{
		median_filts_.resize(num_bundles_);
		for(int i=0; i<num_bundles_; i++)
		{
			median_filts_[i] = boost::shared_ptr<ar_track_alvar::MedianFilter>(new ar_track_alvar::MedianFilter(median_filt_size_));
		}
	}   
    
	if(use_depth)
    {
        publish_marker_tf_ = true;
        cloud_sub_ = nh.subscribe("in_cloud", 1, &AlvarBundleTracker::pointCloudCallback, this);

    }
    else
    {
        image_sub_ = it.subscribe ("in_image", 1, &AlvarBundleTracker::imageCallback, this);
    }
}
