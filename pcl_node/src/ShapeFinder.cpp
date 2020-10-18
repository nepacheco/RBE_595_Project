#include <ros/ros.h>
#include <iostream>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl-1.8/pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

ros::Publisher pubCoeffs;
ros::Publisher pub_ext_objs;

void extractObjects(pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud);
bool segmentCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud);


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*input, *cloud);

  // Cloud of just the objects (no floor plane)
  pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // Remove floor plane from cloud
  extractObjects(cloud, obj_cloud);
  segmentCylinder(obj_cloud, cylinder_cloud);

  // convert clouds to ros msgs for outputting
  sensor_msgs::PointCloud2 outputCloud;
  pcl::toROSMsg(*cylinder_cloud, outputCloud);
  pub_ext_objs.publish (outputCloud);

  // Publish the model coefficients
  // pcl_msgs::ModelCoefficients ros_coefficients;
  // pcl_conversions::fromPCL(*coefficients, ros_coefficients);
  
  // pubCoeffs.publish (ros_coefficients);
}

bool segmentCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud){
  
  // Objects needed
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg; 
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  // Dataset objects
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0, 0.2);
  seg.setInputCloud (cloud);
  seg.setInputNormals (cloud_normals);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);

  // Extract Cylinder 
  extract.setInputCloud (cloud);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  extract.filter (*filtered_cloud);

  // std::cout << *coefficients_cylinder << std::endl;

  return true;
}

/**
 * extracts largest plane from cloud 
 * 
 **/
void extractObjects(pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
{
  // outputs objects of segmentaion
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory parameters for segmetation
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (original_cloud->makeShared ());   // deep copy of input cloud
  seg.segment (*inliers, *coefficients);      // get largest plane from cloud

  // create extractor
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  // remove largest plane from cloud
  extract.setInputCloud (original_cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*filtered_cloud);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber camSub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/points", 10, cloud_cb);

  // Create a ROS publisher for the output point cloud
  // pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  pub_ext_objs = nh.advertise<sensor_msgs::PointCloud2> ("extracted_objects", 1);
  pubCoeffs = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);

  // Spin
  ros::spin ();
}