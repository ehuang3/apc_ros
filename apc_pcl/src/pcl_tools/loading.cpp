#include <string>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>

template<typename PointT> 
void cloud_from_ply(std::string filename, pcl::PointCloud<PointT>& cloud) {
    /* Extract a pcl::PointCloudXYZ from a .ply file*/
    pcl::PolygonMesh mesh;
    pcl::PCLPointCloud2 temp_cloud;
    pcl::io::loadPolygonFilePLY(filename, mesh);
    pcl::fromPCLPointCloud2(mesh.cloud, cloud);
}
