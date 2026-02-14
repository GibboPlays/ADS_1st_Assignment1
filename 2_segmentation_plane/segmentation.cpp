#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

/*
	Objective: Segment the ground from the input cloud. 
	Input: point cloud
	Output: 
			1) point cloud/s representing the ground
			2) point cloud without the ground

*/

int
main (int argc, char** argv)
{
    //controllo parametri
   if (argc < 2) {
       std::cerr << "Uso: ./segmentation <input.pcd>" << std::endl;
       return -1;
   }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input (new pcl::PointCloud<pcl::PointXYZ>), //point cloud input
        cloud_segmented (new pcl::PointCloud<pcl::PointXYZ>),  //point cloud with planes
        cloud_aux (new pcl::PointCloud<pcl::PointXYZ>), //aux point cloud
        cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>); //point cloud filtered
    
    pcl::PCDWriter writer;

    // Fill in the cloud data
    pcl::PCDReader reader;
    reader.read (argv[1], *cloud_input);

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_input);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.1); // determines how close a point must be to the model in order to be considered an inlier

    int i = 0, nr_points = (int) cloud_filtered->size ();
    
    // Now we will remove the planes from the filtered point cloud 
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ()); //the resultant model coefficients
    //inliers represent the points of the point cloud representing the plane, coefficients of the model that represents the plane (4 points of the plane)
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ()); 
    // While 30% of the original cloud is still there
    while (cloud_filtered->size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud <-
        seg.setInputCloud (cloud_filtered);
        /*
        Base method for segmentation of a model in a PointCloud given by <setInputCloud (), setIndices ()>
        [out]	inliers	the resultant point indices that support the model found (inliers)
        [out]	model_coefficients	the resultant model coefficients that describe the plane 
        */
        seg.segment (*inliers, *coefficients); //we get one of the planes and we put it into the inliers variable
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        // Extract the inliers (here we extract the points of the plane moving the indices representing the plane to cloud_segmented)
        extract.setInputCloud (cloud_filtered); 
        
        //PCL defines a way to define a region of interest / list of point indices that the algorithm should operate on, rather than the entire cloud, via setIndices.
        extract.setIndices (inliers);
        extract.setNegative (false); // Retrieve indices to all points in cloud_filtered but only those referenced by inliers:
        extract.filter (*cloud_segmented);   // We effectively retrieve JUST the plane
        
        std::cerr << "PointCloud representing the planar component: " << cloud_segmented->width * cloud_segmented->height << " data points." << std::endl;

        std::stringstream ss;
        ss << "/home/gibbo_plays/AutoSystem/1st part/1st Assignment/table_scene_lms400_plane_" << i << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_segmented, false);

        // Here we will extract the plane from the original filtered point cloud
        extract.setNegative (true); // original cloud - plane 
        extract.filter (*cloud_aux);  // We write into cloud_f the cloud without the extracted plane
        
        cloud_filtered.swap (cloud_aux); // Here we swap the cloud (the removed plane one) with the original
        i++;
    }
    std::stringstream ss2;
    ss2 << "/home/gibbo_plays/AutoSystem/1st part/1st Assignment/table_scene_lms400_notable.pcd";
    // We write the point cloud without the ground
    writer.write<pcl::PointXYZ> (ss2.str (), *cloud_filtered, false);
    return (0);
}

