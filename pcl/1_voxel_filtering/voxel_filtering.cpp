#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#define visualize_pcl

int
main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // Fill in the cloud data
  pcl::PCDReader reader;

  // Replace the path below with the path where you saved your file
  reader.read ("../../table_scene_lms400.pcd", *cloud); // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (50.0f, 50.0f, 50.0f); //this value defines how much the PC is filtered
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;
  pcl::PCDWriter writer;
  writer.write ("../../table_scene_lms400_downsampled.pcd", *cloud_filtered,
                Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

  #ifdef visualize_pcl


    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_filtered_display((new pcl::PointCloud<pcl::PointXYZ>));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_display((new pcl::PointCloud<pcl::PointXYZRGB>)) ;

    pcl::fromPCLPointCloud2( *cloud_filtered, *cloud_filtered_display );
    pcl::fromPCLPointCloud2( *cloud, *cloud_display );
    std::uint8_t r = 255, g = 0, b = 0;

    for(auto pp : cloud_display->points)
        pp.rgb = (static_cast<uint32_t>(0) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));


    for(auto ppf : cloud_filtered_display->points)
    {
      pcl::PointXYZRGB ppf_color(ppf.x,ppf.y,ppf.z);
      ppf_color.rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      cloud_display->push_back(ppf_color);
    }

    viewer.showCloud(cloud_display);

    while (!viewer.wasStopped ())
    {

    }

    writer.write ("../../table_scene_lms400_disp.pcd", *cloud_display);
                  //Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

#endif


  return (0);
}
