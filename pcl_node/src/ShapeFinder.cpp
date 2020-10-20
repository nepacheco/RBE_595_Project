#include <ros/ros.h>
#include <iostream>
#include <math.h>  

// PCL specific includes
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
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
#include <std_msgs/String.h>

#include <shape_node/shapeArray.h>
#define PI 3.14159265

ros::Publisher pubCoeffs;
ros::Publisher pub_ext_objs;
ros::Publisher pub_shape;
ros::Publisher pub_pose;

void extractObjects(pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud);
bool segmentCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud);
bool segmentSphere(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud);
void coordsFromCam(float *points, float *coords);


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
  // segmentCylinder(obj_cloud, cylinder_cloud);
  segmentSphere(obj_cloud, cylinder_cloud);

  // convert clouds to ros msgs for outputting
  sensor_msgs::PointCloud2 outputCloud;
  pcl::toROSMsg(*cylinder_cloud, outputCloud);
  pub_ext_objs.publish (outputCloud);
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

  // Get parameters of cylinder
  float radius = coefficients_cylinder->values[6];  
  float maxH = 0;
  float axis[3] = {coefficients_cylinder->values[0], coefficients_cylinder->values[1], coefficients_cylinder->values[2]};
  float centroid[3] = {0, 0, 0};

  float sumX = 0, sumY = 0, avgX = 0, avgY = 0;
  int num_points = filtered_cloud->points.size();

  for (int i = 0; i < num_points; i++) {
    float points[3] = {filtered_cloud->points[i].x, filtered_cloud->points[i].y, filtered_cloud->points[i].z};
    float coords[3];

    coordsFromCam(points, coords);
    if (coords[2] > maxH) maxH = coords[2];

    sumX += coords[0];
    sumY += coords[1];
  }

  avgX = sumX / num_points;
  avgY = sumY / num_points;

  float gamma = atan2(avgY, avgX);
  // float centroid[3] = {(avgX + cos(gamma) * radius), (avgY + sin(gamma) * radius), maxH/2};
  coordsFromCam(axis, centroid);

  // std::cout << "Radius " << radius
  //           << " Height " << maxH
  //           << " Centroid " << centroid[0]
  //           << "  " << centroid[1]
  //           << "  " << centroid[2] <<
  // std::endl;

  // Publish the shape message
  shape_node::shapeArray msg;

  msg.shapetype.data = "Cylinder";
  msg.pose.position.x = centroid[0];
  msg.pose.position.y = centroid[1];
  msg.pose.position.z = centroid[2]/2;
  msg.parameters = {1, 1, 1, 1, maxH, 0, 0, radius};
  pub_shape.publish(msg);

  // Publish pose message for centroid
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.pose = msg.pose;
  pose_msg.header.frame_id = "/map";
  pub_pose.publish(pose_msg);

  return true;
}

bool segmentSphere(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud){
  
  // Objects needed
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg; 
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  // Dataset objects
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_sphere (new pcl::PointIndices);

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for SPHERE segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_SPHERE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0, 0.2);
  seg.setInputCloud (cloud);
  seg.setInputNormals (cloud_normals);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_sphere, *coefficients);

  // Extract Cylinder 
  extract.setInputCloud (cloud);
  extract.setIndices (inliers_sphere);
  extract.setNegative (false);
  extract.filter (*filtered_cloud);

  // Get parameters of cylinder
  float radius = coefficients->values[3];  
  float maxH = 0;
  float axis[3] = {coefficients->values[0], coefficients->values[1], coefficients->values[2]};
  float centroid[3] = {0, 0, 0};

  // float sumX = 0, sumY = 0, avgX = 0, avgY = 0;
  // int num_points = filtered_cloud->points.size();

  // for (int i = 0; i < num_points; i++) {
  //   float points[3] = {filtered_cloud->points[i].x, filtered_cloud->points[i].y, filtered_cloud->points[i].z};
  //   float coords[3];

  //   coordsFromCam(points, coords);
  //   if (coords[2] > maxH) maxH = coords[2];

  //   sumX += coords[0];
  //   sumY += coords[1];
  // }

  // avgX = sumX / num_points;
  // avgY = sumY / num_points;

  // float gamma = atan2(avgY, avgX);
  // float centroid[3] = {(avgX + cos(gamma) * radius), (avgY + sin(gamma) * radius), maxH/2};
  coordsFromCam(axis, centroid);

  // std::cout << "Radius " << radius
  //           << " Height " << maxH
  //           << " Centroid " << centroid[0]
  //           << "  " << centroid[1]
  //           << "  " << centroid[2] <<
  // std::endl;

  // Publish the shape message
  shape_node::shapeArray msg;

  msg.shapetype.data = "Sphere";
  msg.pose.position.x = centroid[0];
  msg.pose.position.y = centroid[1];
  msg.pose.position.z = centroid[2];
  msg.parameters = {1, 1, 1, 1, 0, 0, 0, radius};
  pub_shape.publish(msg);

  // Publish pose message for centroid
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.pose = msg.pose;
  pose_msg.header.frame_id = "/map";
  pub_pose.publish(pose_msg);

  return true;
}

void coordsFromCam(float *points, float *coords){
  float x = points[0];
  float y = points[1];
  float z = points[2];

  float d = sqrt(pow(z,2) + pow(y,2));
  float alpha = atan2(z,y);
  float beta = alpha - PI / 4;
  float a = cos(beta) * d;

  float b = sin(beta) * d;

  coords[0] = b;
  coords[1] = -x;
  coords[2] = 1 - a;
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
  ros::init (argc, argv, "pcl_node");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber camSub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/points", 10, cloud_cb);

  // Create a ROS publisher for the output point cloud
  // pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  pub_ext_objs = nh.advertise<sensor_msgs::PointCloud2> ("extracted_objects", 1);
  pubCoeffs = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);
  pub_shape = nh.advertise<shape_node::shapeArray> ("ShapeArray", 1);
  pub_pose = nh.advertise<geometry_msgs::PoseStamped> ("ShapePose", 1);

  // Spin
  ros::spin ();
}