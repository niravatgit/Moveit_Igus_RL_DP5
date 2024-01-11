#include <ros/ros.h>
#include <string>
#include <iostream>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/io/pcd_io.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>


using namespace pcl;

class PointCloudProcessor {
public:
    PointCloudProcessor() {
        // Subscribe to the input point cloud topic
        pointCloudSub = nh.subscribe("/royale_cam0/point_cloud", 1, &PointCloudProcessor::pointCloudCallback, this);

        // Create publishers for segmented point clouds
        pubBinHead = nh.advertise<sensor_msgs::PointCloud2>("/output_cloud", 1);
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& inputCloudMsg) 
    {

        double scale1 = 0.2;
        double scale2 = 2.0;
        double threshold = 0.3;
        double segradius = 0.3;

        // Convert ROS PointCloud2 message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*inputCloudMsg, *cloud);

        // Create a search tree, use KDTreee for non-organized data.
        pcl::search::Search<PointXYZ>::Ptr tree;
        if (cloud->isOrganized ())
        {
          tree.reset (new pcl::search::OrganizedNeighbor<PointXYZ> ());
        }
        else
        {
          tree.reset (new pcl::search::KdTree<PointXYZ> (false));
        }
        
        // Set the input pointcloud for the search tree
        tree->setInputCloud (cloud);

        // Compute normals using both small and large scales at each point
        pcl::NormalEstimationOMP<PointXYZ, PointNormal> ne;
        ne.setInputCloud(cloud);
        ne.setSearchMethod(tree);



        ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());
        
        // Calculating Normals with the small scale
        std::cout<<"Calculating normals for scale..."<< scale1 <<std::endl;
        pcl::PointCloud<PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<PointNormal>);

        ne.setRadiusSearch((scale1));
        ne.compute(*normals_small_scale);

        // Calculating normals with the large scale
        std::cout << "Calculating normals for the scale ..." << scale2 << std::endl;
        pcl::PointCloud<PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<PointNormal>);
        ne.setRadiusSearch((scale2));
        ne.compute(*normals_large_scale);

        // Create output cloud for DoN results
        PointCloud<PointNormal>::Ptr doncloud (new pcl::PointCloud<PointNormal>);
        copyPointCloud (*cloud, *doncloud);

        std::cout << "Calculating DoN... " << std::endl;
        // Create DoN operator
        pcl::DifferenceOfNormalsEstimation<PointXYZ, PointNormal, PointNormal> don;
        don.setInputCloud (cloud);
        don.setNormalScaleLarge (normals_large_scale);
        don.setNormalScaleSmall (normals_small_scale);

        if (!don.initCompute ())
        {
          std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
          exit (EXIT_FAILURE);
        }

        // Compute DoN
        don.computeFeature (*doncloud);

        // Filter by magnitude
        std::cout << "Filtering out DoN mag <= " << threshold << "..." << std::endl;

        // Build the condition for filtering
        pcl::ConditionOr<PointNormal>::Ptr range_cond (
          new pcl::ConditionOr<PointNormal> ()
          );
        range_cond->addComparison (pcl::FieldComparison<PointNormal>::ConstPtr (
                                    new pcl::FieldComparison<PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold))
                                  );
        // Build the filter
        pcl::ConditionalRemoval<PointNormal> condrem;
        condrem.setCondition (range_cond);
        condrem.setInputCloud (doncloud);

        pcl::PointCloud<PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<PointNormal>);

        // Apply filter
        condrem.filter (*doncloud_filtered);

        doncloud = doncloud_filtered;

        // Save filtered output
        std::cout << "Filtered Pointcloud: " << doncloud->size () << " data points." << std::endl;

        // Filter by magnitude
        std::cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << std::endl;

        pcl::search::KdTree<PointNormal>::Ptr segtree (new pcl::search::KdTree<PointNormal>);
        segtree->setInputCloud (doncloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointNormal> ec;

        ec.setClusterTolerance (segradius);
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (100000);
        ec.setSearchMethod (segtree);
        ec.setInputCloud (doncloud);
        ec.extract (cluster_indices);

        int j = 0;
        for (const auto& cluster : cluster_indices)
        {
          pcl::PointCloud<PointNormal>::Ptr cloud_cluster_don (new pcl::PointCloud<PointNormal>);
          for (const auto& idx : cluster.indices)
          {
            cloud_cluster_don->points.push_back ((*doncloud)[idx]);
          }

          cloud_cluster_don->width = cloud_cluster_don->size ();
          cloud_cluster_don->height = 1;
          cloud_cluster_don->is_dense = true;
        }

        sensor_msgs::PointCloud2 output_cloud;
        pcl::toROSMsg(*doncloud, output_cloud);

        pubBinHead.publish(output_cloud);
        std::cout<< std::endl;
  }


private:
    ros::NodeHandle nh;
    ros::Subscriber pointCloudSub;
    ros::Publisher pubBinHead;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_processor_node");
    PointCloudProcessor processor;
    ros::spin();
    return 0;
}