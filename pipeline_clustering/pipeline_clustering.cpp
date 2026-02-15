#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/colors.h> //per generare colori casuali
#include <pcl/common/centroid.h> //calcola distanza euclidea
#include <pcl/common/common.h> //per ottenere punto minimo e massimo

void myEuclideanClustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                           pcl::search::KdTree<pcl::PointXYZ>::Ptr& tree, 
                           float tolerance, int minSize, int maxSize,
                           std::vector<pcl::PointIndices>& cluster_indices)
{
    //vettore punti visitati
    std::vector<bool> visited(cloud->points.size(), false);

    for (int i = 0; i < cloud->points.size(); ++i) 
    {
        if (visited[i]) continue;

        //nuovo cluster
        std::vector<int> cluster_ids;
        std::vector<int> queue;
        queue.push_back(i);
        visited[i] = true;

        //esploro vicini
        for(int idx = 0; idx < queue.size(); idx++) 
        {
            int current_point = queue[idx];
            std::vector<int> neighbors;
            std::vector<float> distances;

            //trovo vicini entro la tolleranza
            if (tree->radiusSearch(cloud->points[current_point], tolerance, neighbors, distances) > 0) 
            {
                for (int neighbor_idx : neighbors) 
                {
                    if (!visited[neighbor_idx]) 
                    {
                        visited[neighbor_idx] = true;
                        queue.push_back(neighbor_idx);
                    }
                }
            }
        }

        //se cluster rispetta i vincoli
        if (queue.size() >= minSize && queue.size() <= maxSize) {
            pcl::PointIndices indices;
            indices.indices = queue;
            cluster_indices.push_back(indices);
        }
    }
}

int
main (int argc, char** argv)
{
   //controllo parametri
   if (argc < 2) {
       std::cerr << "Uso: ./pipeline_clustering <input.pcd>" << std::endl;
       return -1;
   }

   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), //point cloud input
        cloud_segmented (new pcl::PointCloud<pcl::PointXYZ>),  //point cloud with planes
        cloud_aux (new pcl::PointCloud<pcl::PointXYZ>), //aux point cloud
        cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>); //point cloud filtered

   // Fill in the cloud data
   pcl::PCDReader reader;
   // Replace the path below with the path where you saved your file
   reader.read (argv[1], *cloud); // Remember to download the file first!

   std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
   << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

   // Create the filtering object
   pcl::VoxelGrid<pcl::PointXYZ> sor;
   sor.setInputCloud (cloud);
   sor.setLeafSize (0.02f, 0.02f, 0.02f); //this value defines how much the PC is filtered
   sor.filter (*cloud_filtered);

   std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
   << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

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
    while (cloud_filtered->size () > 0.5 * nr_points)
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

        // Here we will extract the plane from the original filtered point cloud
        extract.setNegative (true); // original cloud - plane 
        extract.filter (*cloud_aux);  // We write into cloud_f the cloud without the extracted plane
        
        cloud_filtered.swap (cloud_aux); // Here we swap the cloud (the removed plane one) with the original
        i++;
    }

   // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered); 

    std::vector<pcl::PointIndices> cluster_indices;

    myEuclideanClustering(cloud_filtered, tree, 0.15, 100, 25000, cluster_indices);
    /**

    Now we extracted the clusters out of our point cloud and saved the indices in cluster_indices. 

    To separate each cluster out of the vector<PointIndices> we have to iterate through cluster_indices, create a new PointCloud for each entry and write all points of the current cluster in the PointCloud.

    **/

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clusters_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::cout << "Detected clusters number: " << cluster_indices.size() << std::endl;

    int nCluster = 0;

    for (int i = 0; i < cluster_indices.size(); ++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : cluster_indices[i].indices)
        {
            cloud_cluster->push_back((*cloud_filtered)[idx]);
        }

        //calcolo altezza cluster
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);
        float cluster_height = maxPt.z - minPt.z;

        //se un cluster è meno alto di 20 cm allora lo scarto
        if (cluster_height < 0.20) continue;

        nCluster++;

        //calcolo distanza euclidea dall'origine (veicolo)
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_cluster, centroid);
        float distance = std::sqrt(centroid[0]*centroid[0] + centroid[1]*centroid[1] + centroid[2]*centroid[2]);

        std::cout << "Cluster [" << i << "] - Posizione: (" << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << ") - Distanza: " << distance << "m" << std::endl;

        //confine pericolo (larghezza auto 2m + 1m di margine per lato)
        float road_width_threshold = 2.0;

        uint8_t r, g, b;
        //se ho il cluster davanti (x > 0) e a meno di 5 metri
        if (centroid[0] > 0 && std::abs(centroid[1]) < road_width_threshold && distance < 5.0) {
            r = 255; 
            g = 0; 
            b = 0;
            std::cout << "FRENARE! Distanza sotto i" << distance << "m" << std::endl;
        } else {
            //colore casuale
            r = rand() % 256; 
            g = rand() % 256; 
            b = rand() % 256;
        }

        for (const auto& idx : cluster_indices[i].indices)
        {
            pcl::PointXYZRGB point;
            point.x = (*cloud_filtered)[idx].x;
            point.y = (*cloud_filtered)[idx].y;
            point.z = (*cloud_filtered)[idx].z;
            point.r = r;
            point.g = g;
            point.b = b;
            cloud_clusters_colored->push_back(point);
        }

    }

    cloud_clusters_colored->width = cloud_clusters_colored->size();
    cloud_clusters_colored->height = 1;
    cloud_clusters_colored->is_dense = true;

    std::cout << "Remaining clusters number: " << nCluster << std::endl;

    pcl::PCDWriter writer;
    if (cloud_clusters_colored->points.empty()) 
    {
    std::cout << "Nessun cluster rilevato, no creazione file pcd" << std::endl;
    } else 
    {
    writer.write<pcl::PointXYZRGB>("clusters.pcd", *cloud_clusters_colored, false);
    }

   return (0);
}
