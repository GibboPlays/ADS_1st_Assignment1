#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
    
int main (int argc,char **argv){

    //controllo parametri
    if (argc < 2) {
        std::cerr << "Uso: ./pcl_reader file.pcd" << std::endl;
        return -1;
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile (argv[1], *cloud) == -1) return -1;
    std::cout << "Punti caricati: " << cloud->size() << std::endl;
    
    //apertura manuale
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    //forza l'uso del backend software prima di aggiungere dati
    viewer->setShowFPS(false);

    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample_cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample_cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    std::cout << "Apertura finestra..." << std::endl;

    while (!viewer->wasStopped ()) {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}