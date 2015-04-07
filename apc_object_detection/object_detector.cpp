#include "object_detector.h"

object_detector::object_detector()
{
    std::cerr  << "Starting" << std::endl;
    model.createImage("/home/niko/Downloads/picking_challenge_items/crayola_64_ct (2)/meshes/poisson.ply");
        std::cerr  << "Model created" << std::endl;
    ros::Subscriber sub=nh.subscribe("camera/depth/points", 1, &object_detector::PointCloudCallback,this);
}

void object_detector::processImage()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr points=detectKeyPoints();
    SpinImage scene;
    scene.createImage(points);
    matchKeyPoints(scene);


}

void object_detector::PointCloudCallback(sensor_msgs::PointCloud2 data_msg)
{
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(data_msg, pcl_pc);

    pcl::fromPCLPointCloud2(pcl_pc, image_cloud);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr object_detector::detectKeyPoints()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(&image_cloud);

    detector.setInputCloud(cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    detector.setSearchMethod(kdtree);
    double resolution = computeCloudResolution(cloud);
    // Set the radius of the spherical neighborhood used to compute the scatter matrix.
    detector.setSalientRadius(6 * resolution);
    // Set the radius for the application of the non maxima supression algorithm.
    detector.setNonMaxRadius(4 * resolution);
    // Set the minimum number of neighbors that has to be found while applying the non maxima suppression algorithm.
    detector.setMinNeighbors(5);
    // Set the upper bound on the ratio between the second and the first eigenvalue.
    detector.setThreshold21(0.975);
    // Set the upper bound on the ratio between the third and the second eigenvalue.
    detector.setThreshold32(0.975);
    // Set the number of prpcessing threads to use. 0 sets it to automatic.
    detector.setNumberOfThreads(4);

    detector.compute(*keypoints);
}

void object_detector::matchKeyPoints(SpinImage scene)
{
    // A kd-tree object that uses the FLANN library for fast search of nearest neighbors.
                std::cerr << "Testing" <<model.descriptors->size() <<std::endl;
        pcl::KdTreeFLANN<SpinImage_type> matching;
        std::cerr << "HI" <<model.descriptors->size() <<std::endl;
        matching.setInputCloud(model.descriptors);
        // A Correspondence object stores the indices of the query and the match,
        // and the distance/weight.
        pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());

        // Check every descriptor computed for the scene.
        for (size_t i = 0; i < scene.descriptors->size(); ++i)
        {
            std::vector<int> neighbors(1);
            std::vector<float> squaredDistances(1);
            // Ignore NaNs.
                // Find the nearest neighbor (in descriptor space)...
                int neighborCount = matching.nearestKSearch(scene.descriptors->at(i), 1, neighbors, squaredDistances);
                // ...and add a new correspondence if the distance is less than a threshold
                // (SHOT distances are between 0 and 1, other descriptors use different metrics).
                if (neighborCount == 1 && squaredDistances[0] < 0.25f)
                {
                    pcl::Correspondence correspondence(neighbors[0], static_cast<int>(i), squaredDistances[0]);
                    correspondences->push_back(correspondence);
                }
        }
        std::cout << "Found " << correspondences->size() << " correspondences." << std::endl;
}

double object_detector::computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
        double resolution = 0.0;
        int numberOfPoints = 0;
        int nres;
        std::vector<int> indices(2);
        std::vector<float> squaredDistances(2);
        pcl::search::KdTree<pcl::PointXYZ> tree;
        tree.setInputCloud(cloud);

        for (size_t i = 0; i < cloud->size(); ++i)
        {
            if (! pcl_isfinite((*cloud)[i].x))
                continue;

            // Considering the second neighbor since the first is the point itself.
            nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
            if (nres == 2)
            {
                resolution += sqrt(squaredDistances[1]);
                ++numberOfPoints;
            }
        }
        if (numberOfPoints != 0)
            resolution /= numberOfPoints;

        return resolution;
}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pointcloud2_to_pcd");
    object_detector temp;
  // Spin
  while(ros::ok)
  {
    ros::spinOnce();
    temp.processImage();
  }
}
