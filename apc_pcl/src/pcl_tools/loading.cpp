#include <string>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/PCLPointCloud2.h>
#include <vtkPolyData.h>
#include <vtkPLYReader.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>

#include "pcl_tools.h"

namespace pcl_tools {

    void cloud_from_ply(std::string filename, pcl::PointCloud<pcl::PointXYZ>& cloud) {
        /* Extract a pcl::PointCloudXYZ from a .ply file*/
        pcl::PolygonMesh mesh;
        pcl::io::loadPolygonFilePLY(filename, mesh);
        pcl::fromPCLPointCloud2(mesh.cloud, cloud);
    }

    void cloud_from_ply(std::string filename, pcl::PointCloud<pcl::PointXYZRGBA>& cloud) {
        /* Extract a pcl::PointCloudXYZ from a .ply file 
            Code courtesy of Nikolaus Mitchell */
        vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
        vtkSmartPointer<vtkPolyData> data = vtkSmartPointer<vtkPolyData>::New();
        reader->SetFileName (filename.c_str());
        reader->Update();
        data = reader->GetOutput();
        pcl::io::vtkPolyDataToPointCloud(data, cloud);
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

    bool cloud_from_stl(std::string filename, pcl::PCLPointCloud2& cloud) {
        pcl::PolygonMesh mesh;
        if(pcl::io::loadPolygonFileSTL(filename, mesh) < 0) {
            return false;
        }
        cloud = mesh.cloud;
        return true;
    }

    bool cloud_from_stl(std::string filename, pcl::PointCloud<pcl::PointXYZRGBA>& cloud) {
        pcl::PolygonMesh mesh;
        if(pcl::io::loadPolygonFileSTL(filename, mesh) < 0) {
            return false;
        }
        pcl::fromPCLPointCloud2(mesh.cloud, cloud);
        return true;
    }

    bool cloud_from_stl(std::string filename, pcl::PointCloud<pcl::PointNormal>& cloud) {
        pcl::PolygonMesh mesh;
        if(pcl::io::loadPolygonFileSTL(filename, mesh) < 0) {
            return false;
        }
        pcl::fromPCLPointCloud2(mesh.cloud, cloud);
        return true;
    }

    bool cloud_from_stl(std::string filename, pcl::PointCloud<pcl::PointXYZ>& cloud) {
        pcl::PolygonMesh mesh;
        if(pcl::io::loadPolygonFileSTL(filename, mesh) < 0) {
            return false;
        }
        pcl::fromPCLPointCloud2(mesh.cloud, cloud);
        return true;
    }

}