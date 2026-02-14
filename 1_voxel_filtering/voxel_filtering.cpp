#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int
main (int argc, char** argv)
{
   //controllo parametri
   if (argc < 2) {
       std::cerr << "Uso: ./voxel_filter <input.pcd>" << std::endl;
       return -1;
   }

   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

   // Fill in the cloud data
   pcl::PCDReader reader;
   // Replace the path below with the path where you saved your file
   reader.read (argv[1], *cloud); // Remember to download the file first!

   std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
   << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

   // Create the filtering object
   pcl::VoxelGrid<pcl::PointXYZ> sor;
   sor.setInputCloud (cloud);
   sor.setLeafSize (0.1f, 0.1f, 0.1f); //this value defines how much the PC is filtered
   sor.filter (*cloud_filtered);

   std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
   << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

   pcl::PCDWriter writer;
   std::stringstream ss;
   ss << "/home/gibbo_plays/AutoSystem/1st part/1st Assignment/cloud_filtered.pcd";
   writer.write<pcl::PointXYZ> (ss.str (), *cloud_filtered, false); //*

   return (0);
}


