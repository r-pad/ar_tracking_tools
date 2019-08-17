#ifndef CONSENSUS_BUNDLE_REFINEMENT_H
#define CONSENSUS_BUNDLE_REFINEMENT_H

#include <vector>
#include <map>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>


class ConsensusBundleRefinement
{
    using PointT = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;
    
  public:
    ConsensusBundleRefinement(std::vector<int> &tag_ids,
                              std::vector<PointCloudPtr> &tag_points, 
                              std::vector<double> &tag_sizes, 
                              bool use_corners = true,
                              double inlier_threshold = 0.05,
                              int max_iterations = 1000,
                              bool refine = true,
                              double dense_spacing = 0.);

    bool
    refine(std::map<int, Eigen::Matrix4f> tag_transforms, Eigen::Matrix4f& refined_transform);
    
    static std::pair<int,int>
    createDenseTagCloud(PointCloud &corners, PointCloud &output_cloud, 
                        double spacing, int width = -1, int height = -1);
    
  protected:
    PointCloud tag_template_;
    std::map<int, PointCloudPtr> tag_points_;
    std::map<int, double> tag_sizes_;
    pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> ransac_;
    double dense_spacing_;

};

#endif
