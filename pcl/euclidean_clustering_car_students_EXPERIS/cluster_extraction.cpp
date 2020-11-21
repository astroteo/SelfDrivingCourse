#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include "Renderer.hpp"
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <chrono>
#include "tree_utilities.hpp"

using namespace lidar_obstacle_detection;


void
ProcessAndRenderPointCloud (Renderer& renderer, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  // 1) Downsample the dataset
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize (0.1f, 0.1f, 0.1f); //this value defines how much the PC is filtered
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new   pcl::PointCloud<pcl::PointXYZ>);
  sor.filter (*cloud_filtered);

  // 2) here we crop the points that are far away from us, in which we are not interested
  pcl::CropBox<pcl::PointXYZ> cb(true);
  cb.setInputCloud(cloud_filtered);
  cb.setMin(Eigen::Vector4f (-20, -6, -2, 1));
  cb.setMax(Eigen::Vector4f ( 30, 7, 5, 1));
  cb.filter(*cloud_filtered);

  // 3) Segmentation and apply RANSAC

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  // Optional
  seg.setOptimizeCoefficients (true);

  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.61); // determines how close a point must be to the model in order to be considered an inlier



  // 4) iterate over the filtered cloud, segment and remove the planar inliers
  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  int i = 0, nr_points = (int) cloud_filtered->size ();
  while (cloud_filtered->size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients); //inliers represent the points of the point cloud representing the plane, coefficients of the model that represents the plane (4 points of the plane)
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers (here we extract the plane and we moved the plane to cloud_p)
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_plane);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    std::stringstream ss;


    // Create the filtering object (here we extract the plane of the filtered point cloud)
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    i++;
  }


  // 5) Create the KDTree and the vector of PointIndices
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);



  // 6) Set the spatial tolerance for new cluster candidates (pay attention to the tolerance!!!)
  // Here we are creating a vector of PointIndices, which contain the actual index information in a vector<int>. The indices of each detected cluster are saved here.
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

  //Set the spatial tolerance for new cluster candidates
  //If you take a very small value, it can happen that an actual object can be seen as multiple clusters. On the other hand, if you set the value too high, it could happen, that multiple objects are seen as one cluster
  ec.setClusterTolerance (0.2); // 2cm

  // 7) Set up the euclidean cluster parameters
  //We impose that the clusters found must have at least setMinClusterSize() points and maximum setMaxClusterSize() points
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (250000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);


  std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1), Color(1,0,1), Color(0,1,1)};


  /**Now we extracted the clusters out of our point cloud and saved the indices in cluster_indices.

  To separate each cluster out of the vector<PointIndices> we have to iterate through cluster_indices, create a new PointCloud for each entry and write all points of the current cluster in the PointCloud.
  Compute euclidean distance
  **/
  int j = 0;
  int clusterId = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->push_back ((*cloud_filtered)[*pit]);
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";

    renderer.RenderPointCloud(cloud,"obstCloud"+std::to_string(clusterId),Color(0,1,1));
    //Point cloud bounding box

    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);

    Box box{minPt.x, minPt.y, minPt.z,
            maxPt.x, maxPt.y, maxPt.z};
    renderer.RenderBox(box, j);

    ++clusterId;
    j++;
  }

}


int main(int argc, char* argv[])
{

  Renderer renderer;
  renderer.InitCamera(CameraAngle::XY);
  // Clear viewer
  renderer.ClearViewer();

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<boost::filesystem::path> stream(boost::filesystem::directory_iterator{"../../dataset_1/"},
                                             boost::filesystem::directory_iterator{});

  // sort files in ascending (chronological) order
  std::sort(stream.begin(), stream.end());


  auto streamIterator = stream.begin();

  while (not renderer.WasViewerStopped())
  {
    renderer.ClearViewer();


    pcl::PCDReader reader;
    reader.read (streamIterator->string(), *input_cloud);
    auto startTime = std::chrono::steady_clock::now();

    ProcessAndRenderPointCloud(renderer,input_cloud);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "[PointCloudProcessor<PointT>::ReadPcdFile] Loaded "
              << input_cloud->points.size() << " data points from " << streamIterator->string() <<  "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    streamIterator++;
    if(streamIterator == stream.end())
      streamIterator = stream.begin();

    renderer.SpinViewerOnce();

  }

}
