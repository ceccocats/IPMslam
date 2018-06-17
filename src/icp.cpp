#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
 #include <pcl/visualization/cloud_viewer.h>

int main (int argc, char** argv) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vizCloud(new pcl::PointCloud<pcl::PointXYZRGB>); 

    // Fill in the CloudIn data
    cloud_in->width    = 100000;
    cloud_in->height   = 1;
    cloud_in->is_dense = false;
    cloud_in->points.resize (cloud_in->width * cloud_in->height);
    for (size_t i = 0; i < cloud_in->points.size (); ++i)
    {
        cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }
    
    // transform CloudOut
    *cloud_out = *cloud_in;
    std::cout << "size:" << cloud_out->points.size() << std::endl;
    for (size_t i = 0; i < cloud_in->points.size (); ++i)
        cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
    
    // run ICP
    pcl::PointCloud<pcl::PointXYZ> Final;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputCloud(cloud_in);
    icp.setInputTarget(cloud_out);
    icp.align(Final);

    pcl::copyPointCloud(Final,*vizCloud);
    for(int i=0; i<vizCloud->points.size(); i++) {
        // pack r/g/b into rgb
        uint8_t r = 255, g = 0, b = 0;    // Example: Red color
        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        vizCloud->points[i].rgb = *reinterpret_cast<float*>(&rgb);
    }

    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud (vizCloud);
    while (!viewer.wasStopped ())
    {
    }

    return (0);
}