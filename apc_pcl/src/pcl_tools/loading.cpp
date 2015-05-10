#include <string>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>
#include "pcl_tools.h"
namespace pcl_tools {

    void cloud_from_ply(std::string filename, pcl::PointCloud<pcl::PointXYZ>& cloud) {
        /* Extract a pcl::PointCloudXYZ from a .ply file*/
        pcl::PolygonMesh mesh;
        pcl::PCLPointCloud2 temp_cloud;
        pcl::io::loadPolygonFilePLY(filename, mesh);
        pcl::fromPCLPointCloud2(mesh.cloud, cloud);
    }

    void cloud_from_ply(std::string filename, pcl::PointCloud<pcl::PointXYZRGBA>& cloud) {
        /* Extract a pcl::PointCloudXYZ from a .ply file*/
        pcl::PolygonMesh mesh;
        pcl::PCLPointCloud2 temp_cloud;
        pcl::io::loadPolygonFilePLY(filename, mesh);
        pcl::fromPCLPointCloud2(mesh.cloud, cloud);
    }

    bool cloud_from_pcd(std::string filename, pcl::PointCloud<pcl::PointXYZ>& cloud) {
        int success;
        success = pcl::io::loadPCDFile<pcl::PointXYZ> (filename, cloud);
        if (success == -1) {
            return false;
        } else{
            return true;
        }
    }

    bool cloud_from_pcd(std::string filename, pcl::PointCloud<pcl::PointXYZRGBA>& cloud) {
        int success;
        success = pcl::io::loadPCDFile<pcl::PointXYZRGBA> (filename, cloud);
        if (success == -1) {
            return false;
        } else{
            return true;
        }
    }

}