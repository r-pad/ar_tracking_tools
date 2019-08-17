#include <ar_tracking_tools/alvar_bundle_tracker.h>
#include <assert.h>

ConsensusBundleRefinement::ConsensusBundleRefinement(std::vector<int> &tag_ids,
                          std::vector<PointCloudPtr> &tag_points, 
                          std::vector<double> &tag_sizes, 
                          bool use_corners,
                          double inlier_threshold,
                          int max_iterations,
                          bool refine,
                          double dense_spacing):dense_spacing_(dense_spacing)
{
    for(size_t j=0; j < tag_ids.size(); ++j)
    {
        tag_points_.insert(std::pair<int, PointCloudPtr>(tag_ids[j], tag_points[j]));
        tag_sizes_.insert(std::pair<int, double>(tag_ids[j], tag_sizes[j]));
    }

    tag_template_.points.push_back(PointT(0,0,0));
    
    if(use_corners || dense_spacing_ > 0)
    {
        tag_template_.points.push_back(PointT(-0.5,-0.5,0));
        tag_template_.points.push_back(PointT( 0.5,-0.5,0));
        tag_template_.points.push_back(PointT( 0.5, 0.5,0));
        tag_template_.points.push_back(PointT(-0.5, 0.5,0));
    }

    ransac_.setInlierThreshold(inlier_threshold);
    ransac_.setMaximumIterations(max_iterations);
    ransac_.setRefineModel(refine);
}

std::pair<int, int>
ConsensusBundleRefinement::createDenseTagCloud(PointCloud &corners, PointCloud &output_cloud, 
                                               double spacing, int width, int height)
{
    Eigen::Vector3f corner = corners[1].getVector3fMap();
    Eigen::Vector3f axis_x = corners[2].getVector3fMap() - corner;
    double len_x = axis_x.norm();
    axis_x *= spacing/len_x;
    Eigen::Vector3f axis_y = corners[4].getVector3fMap() - corner;
    double len_y = axis_y.norm();
    axis_y *= spacing/len_y;
    
    if(width < 0)
    {
        width = len_x/spacing;
    }
    if(height < 0)
    {
        height = len_y/spacing;
    }
    
    for(int j = 1; j < width; ++j)
    {
        for(int k = 1; k < height; ++k)
        {
            PointT pt;
            pt.getVector3fMap() = corner + j*axis_x + k*axis_y;
            output_cloud.points.push_back(pt);
        }
    }
    
    return std::make_pair(width, height);
}

bool 
ConsensusBundleRefinement::refine(std::map<int, Eigen::Matrix4f> tag_transforms, Eigen::Matrix4f& refined_transform)
{
    PointCloudPtr source_cloud(new PointCloud());
    PointCloudPtr target_cloud(new PointCloud());
    pcl::Correspondences correspondences, inliers;

    for(std::pair<int, double> element : tag_sizes_)
    {
        int tag_id = element.first;
        if(tag_transforms.find(tag_id)!=tag_transforms.end())
        {
            PointCloud scaled_template, transformed_corners;
            Eigen::Matrix4f transform = tag_transforms[tag_id];
            double tag_size = tag_sizes_[tag_id];
            Eigen::Affine3f scaled_transform(Eigen::Affine3f::Identity());
            scaled_transform.prescale(tag_size/100.);
            pcl::transformPointCloud (tag_template_, scaled_template, scaled_transform);
            if(dense_spacing_ > 0)
            {
                createDenseTagCloud(scaled_template, scaled_template, dense_spacing_, 
                                    tag_points_[tag_id]->width, tag_points_[tag_id]->height);
            }
            pcl::transformPointCloud (scaled_template, transformed_corners, transform);
            *source_cloud += transformed_corners;
            *target_cloud += *tag_points_[tag_id];
        }
    }
    source_cloud->width = source_cloud->points.size();
    target_cloud->width = target_cloud->points.size();

    assert(source_cloud->size() == target_cloud->size());

    for(int j=0; j < source_cloud->size(); ++j)
    {
        correspondences.push_back(pcl::Correspondence(j, j, 1.0));
    }

    ransac_.setInputSource(source_cloud);
    ransac_.setInputTarget(target_cloud);
    ransac_.getRemainingCorrespondences(correspondences, inliers);

    if(inliers.size() > 3)
    {
        refined_transform = ransac_.getBestTransformation().inverse();
        return true;
    }
    return false;
}
    
