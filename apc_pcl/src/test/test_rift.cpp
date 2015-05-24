#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_plotter.h>

typedef pcl::PointXYZRGBA PointT; 
typedef pcl::PointCloud<PointT> PointCloudT; 

int 
main (int argc, 
      char** argv) 
{ 
  // Create and populate cloud 
  PointCloudT::Ptr cloud (new PointCloudT); 
  cloud->resize (500); 

  for (size_t i = 0; i < cloud->size (); i++) 
  { 
    cloud->points[i].r = 255 * (1024 * rand () / (RAND_MAX + 1.0f)); 
    cloud->points[i].g = 255 * (1024 * rand () / (RAND_MAX + 1.0f)); 
    cloud->points[i].b = 255 * (1024 * rand () / (RAND_MAX + 1.0f)); 
  } 

  // Analyze cloud red channel 
  std::vector<double> data_red; 
  for (size_t i = 0; i < cloud->size (); i++) 
  { 
    double red = (double) cloud->points[i].r; 
    data_red.push_back (red); 
    //std::cout << "Red value: " << red << std::endl; 
  } 

  int nbins = 255; 
  pcl::visualization::PCLPlotter *plotter = new pcl::visualization::PCLPlotter ("Histogram"); 
  plotter->addHistogramData (data_red, nbins);  // Histogram segfaults if given NaN data with PCL < 1.8 

  while (!plotter->wasStopped ()) 
    plotter->spinOnce (); 

  return (0); 
}