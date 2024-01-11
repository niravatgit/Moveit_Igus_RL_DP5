#include <ros/ros.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/search/kdtree.h>

ros::Publisher pub;
double radius = 0.03; // Default radius for radius search
int k = 5; // Default number of neighbors for k-nearest neighbor search

/*
 * @brief Callback function to process incoming point cloud data.
 * @param input - The input point cloud message.
*/
void cloud_cb(const pcl::PCLPointCloud2ConstPtr &input)
{
    // Convert data to PointCloud<T>
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*input, *xyz);

    // Normal estimation setup
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(xyz);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    if (xyz->points.size() <= k)
    {
        // For datasets smaller than k, use radius search
        ne.setRadiusSearch(radius);
    }
    else
    {
        // For larger datasets, use k-nearest neighbor search
        ne.setKSearch(k);
    }

    // Compute normals
    ne.compute(*cloud_normals);

    // Convert data back
    pcl::PCLPointCloud2 output_normals;
    pcl::toPCLPointCloud2(*cloud_normals, output_normals);

    // Publish the data
    pub.publish(output_normals);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "normal_calculation");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/royale_cam0/segmented_point_cloud", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<pcl::PCLPointCloud2>("/cloud_normals", 1);

    // Spin
    ros::spin();
}

/*
* @section Parameters
*
* - `radius` (double): The radius used for radius search if the point cloud is smaller than `k`.
* - `k` (int): The number of neighbors used for k-nearest neighbor search if the point cloud is larger than `k`.
*
* @section Node_Structure
*
* The node consists of a `main` function and a callback function `cloud_cb` to handle incoming point cloud data.
*
* @subsection Callbacks
*
* - `cloud_cb(const pcl::PCLPointCloud2ConstPtr &input)`: Callback function to process incoming point cloud data.
*   - Converts the incoming `pcl::PCLPointCloud2` point cloud message to a `pcl::PointCloud<pcl::PointXYZ>`.
*   - Performs normal estimation using either radius search or k-nearest neighbor search based on the point cloud size.
*   - Publishes the computed normals on the specified topic.
*
* @subsection Main_Function
*
* - `main(int argc, char **argv)`: Main function of the node.
*   - Initializes ROS.
*   - Creates a ROS subscriber for the input point cloud.
*   - Creates a ROS publisher for the output point cloud.
*   - Enters the ROS spin loop to keep the node active and responsive to incoming messages.
*
* @section Running_the_Node
*
* 1. Compile the code with appropriate dependencies and libraries.
* 2. Launch the ROS master.
* 3. Run the node with `rosrun rl_dp_5_pcl normal_calculation`.
* 4. The node will subscribe to the specified point cloud topic, compute normals, and publish the results.
*
* @section ROS_Topics
*
* - Subscribed Topic: "/royale_cam0/segmented_point_cloud" (PointCloud2)
* - Published Topic: "/cloud_normals" (PointCloud2)
*
* @section ROS_Parameters
*
* - `radius` and `k` can be adjusted dynamically using ROS parameter tools to fine-tune the normal estimation process during runtime.
*
* @section Notes
*
* - The node assumes that the input point cloud is of type `pcl::PointXYZ`.
* - The parameters `radius` and `k` should be tuned based on the characteristics of the input point cloud.
*
*/
