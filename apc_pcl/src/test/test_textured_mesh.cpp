#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <string>
#include <iostream>
#include <pcl/io/obj_io.h>
#include <vtkPolyData.h>
#include <vtkPLYReader.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char* argv[]) {

  // std::cout << "hello" << std::endl;
  // pcl::TextureMesh mesh1; 
  // pcl::io::loadPolygonFileOBJ (argv[1], mesh1); 
  // pcl::TextureMesh::Ptr mesh2(new pcl::TextureMesh); 
  // std::cout << "hello" << std::endl;
  // pcl::io::loadOBJFile (argv[1], *mesh2); 

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model(new pcl::PointCloud<pcl::PointXYZRGBA>);

  std::string filename = argv[1];
  
  vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
  vtkSmartPointer<vtkPolyData> data = vtkSmartPointer<vtkPolyData>::New();

  reader->SetFileName (filename.c_str());
  reader->Update();

  data=reader->GetOutput();
  std::cerr << "model loaded" << std::endl;
  pcl::io::vtkPolyDataToPointCloud(data, *model);


  pcl::visualization::PCLVisualizer visu("Test"); 
  visu.addPointCloud (model, "texture"); 
  visu.spin (); 

}