#include "shot_detector.h"

#include <vtkPolyData.h>
#include <vtkPLYReader.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/recognition/hv/hv_go.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/octree/octree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include "../apc_pcl/src/pcl_tools/pcl_functions.h"
#include <pcl/visualization/cloud_viewer.h>
#include <string>

shot_detector::shot_detector()
{
    data = false;
    activated = false;
    std::cerr  << "Starting" << std::endl;
    // Start the ros stuff such as the subscriber and the service
    nh = ros::NodeHandle("apc_object_detection");
    loadParameters();
    //kinect=nh.subscribe("/kinect2_cool/depth_highres/points", 1, &shot_detector::PointCloudCallback,this);
    processor = nh.advertiseService("Shot_detector", &shot_detector::processCloud, this);
    // As we use Ptr to access our pointcloud we first have to initalize something to point to
    pcl::PointCloud<PointType>::Ptr model_ (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr model_keypoints_ (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr scene_ (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr scene_keypoints_ (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<NormalType>::Ptr model_normals_ (new pcl::PointCloud<NormalType> ());
    pcl::PointCloud<NormalType>::Ptr scene_normals_ (new pcl::PointCloud<NormalType> ());
    pcl::PointCloud<DescriptorType>::Ptr model_descriptors_ (new pcl::PointCloud<DescriptorType> ());
    pcl::PointCloud<DescriptorType>::Ptr scene_descriptors_ (new pcl::PointCloud<DescriptorType> ());
    pcl::CorrespondencesPtr model_scene_corrs_ (new pcl::Correspondences ());
    pcl::PointCloud<PointType>::Ptr model_good_kp_ (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr scene_good_kp_ (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr objects_ (new pcl::PointCloud<PointType> ());
    //pcl::PointCloud<DescriptorType>::Ptr model_descriptors_ (new pcl::PointCloud<DescriptorType> ());
    //pcl::PointCloud<DescriptorType>::Ptr scene_descriptors_ (new pcl::PointCloud<DescriptorType> ());
    model = model_;
    model_keypoints = model_keypoints_;
    scene = scene_;
    scene_keypoints = scene_keypoints_;
    model_normals = model_normals_;
    scene_normals = scene_normals_;
    model_descriptors = model_descriptors_;
    scene_descriptors = scene_descriptors_;
    model_scene_corrs = model_scene_corrs_;
    model_good_kp = model_good_kp_;
    scene_good_kp = scene_good_kp_;
    objects = objects_;
    pcl::CorrespondencesPtr correspondences (new pcl::Correspondences);
    correspondences_ = correspondences;
    //Set up a couple of the pcl settings
    descr_est.setRadiusSearch (descr_rad_);
    norm_est.setKSearch (10);


    //calcPFHRGBDescriptors(model,model_keypoints,model_normals,model_descriptors);
}

void shot_detector::processImage()
{
    std::cerr << "Processing" << std::endl;
    std::string file = "/home/niko/projects/apc/catkin/src/apc_ros/apc_object_detection/visual_hull_refined_smoothed.obj";
    loadModel(model, file);
    pcl::io::loadPCDFile("/home/niko/projects/apc/catkin/src/apc_ros/apc_object_detection/niko_file.pcd", *scene);
    //Downsample the model and the scene so they have rougly the same resolution
    pcl::PointCloud<PointType>::Ptr scene_filter (new pcl::PointCloud<PointType> ());
    pcl_functions::voxelFilter(scene, scene_filter, voxel_sample_);
    scene = scene_filter;
    pcl::PointCloud<PointType>::Ptr model_filter (new pcl::PointCloud<PointType> ());
    pcl_functions::voxelFilter(model, model_filter, voxel_sample_);
    model = model_filter;
    // Randomly select a couple of keypoints so we don't calculte descriptors for everything
    sampleKeypoints(model, model_keypoints, model_ss_);
    sampleKeypoints(scene, scene_keypoints, scene_ss_);
    //Calculate the Normals
    calcNormals(model, model_normals);
    calcNormals(scene, scene_normals);
    pcl::visualization::PCLVisualizer viewer("Alignment");
    viewer.setBackgroundColor (0, 0, 0);
    viewer.addPointCloud (scene, "sample cloud");
    viewer.addPointCloudNormals<PointType, NormalType >(scene, scene_normals, 10, 0.05, "normals");;
    viewer.spin();
    //Calculate the shot descriptors at each keypoint in the scene
    calcSHOTDescriptors(model, model_keypoints, model_normals, model_descriptors);
    calcSHOTDescriptors(scene, scene_keypoints, scene_normals, scene_descriptors);
    // Compare descriptors and try to find correspondences
    //ransac(rototranslations,model,scene);
    //refinePose(rototranslations,model,scene);
    compare(model_descriptors, scene_descriptors);
    groupCorrespondences();
    visualizeCorrespondences();
    visualizeICP();

    /*Eigen::Matrix4f pose;
    if(model_scene_corrs->size ()!=0){
        groupCorrespondences();
        ransac(rototranslations,model,scene);
        pose=refinePose(rototranslations,model,scene);

    }*/
}

void shot_detector::loadModel(pcl::PointCloud<PointType>::Ptr model, std::string model_name)
{
    std::string filename = model_name;
    /*vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
    vtkSmartPointer<vtkPolyData> data=vtkSmartPointer<vtkPolyData>::New();
    reader->SetFileName ( filename.c_str() );
    reader->Update();
    data=reader->GetOutput();
    std::cerr << "model loaded" << std::endl;
    pcl::io::vtkPolyDataToPointCloud(data,*model);*/
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_ (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::io::loadOBJFile(filename, *model_);
    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud (model_);
    while (!viewer.wasStopped ()) {
    }
}

void shot_detector::ransac(std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >& transforms, pcl::PointCloud<PointType>::Ptr model, pcl::PointCloud<PointType>::Ptr scene)
{
    // Note: here you would compute or load the descriptors for both
    // the scene and the model. It has been omitted here for simplicity.
    std::cerr << "STart ransac" << std::endl;
    pcl::PointCloud<PointType>::Ptr alignedModel(new pcl::PointCloud<PointType>);
    // Object for pose estimation.
    pcl::SampleConsensusPrerejective<PointType, PointType, DescriptorType> pose;
    pose.setInputSource(model);
    pose.setInputTarget(scene);
    pose.setSourceFeatures(model_descriptors);
    pose.setTargetFeatures(scene_descriptors);
    // Instead of matching a descriptor with its nearest neighbor, choose randomly between
    // the N closest ones, making it more robust to outliers, but increasing time.
    pose.setCorrespondenceRandomness(ran_corr_random_);
    // Set the fraction (0-1) of inlier points required for accepting a transformation.
    // At least this number of points will need to be aligned to accept a pose.
    pose.setInlierFraction(ran_inlier_dist_);
    // Set the number of samples to use during each iteration (minimum for 6 DoF is 3).
    pose.setNumberOfSamples(ran_sample_num_);
    // Set the similarity threshold (0-1) between edge lengths of the polygons. The
    // closer to 1, the more strict the rejector will be, probably discarding acceptable poses.
    pose.setSimilarityThreshold(ran_sim_thresh_);
    // Set the maximum distance threshold between two correspondent points in source and target.
    // If the distance is larger, the points will be ignored in the alignment process.
    pose.setMaxCorrespondenceDistance(ran_max_corr_dist_);
    //pose.setMaximumIterations (ran_max_iter_);

    pose.align(*alignedModel);
    std::cerr << "ransac ran" << std::endl;
    if (pose.hasConverged()) {
        Eigen::Matrix4f transformation = pose.getFinalTransformation();
        transforms.push_back(transformation);
        Eigen::Matrix3f rotation = transformation.block<3, 3>(0, 0);
        Eigen::Vector3f translation = transformation.block<3, 1>(0, 3);

        std::cerr << "Transformation matrix:" << transformation << std::endl << std::endl;
        pcl::visualization::PCLVisualizer visu("Alignment");
        visu.addPointCloud (scene, "scene");
        visu.addPointCloud (alignedModel, "object_aligned");
        visu.spin ();
    } else std::cerr << "Did not converge." << std::endl;
}

/*!
 * \brief shot_detector::processCloud The service function. It process the clouds with shot and returns the pose of the object
 * \param req
 * \param res
 * \return
 */
bool shot_detector::processCloud(apc_msgs::shot_detector_srv::Request &req, apc_msgs::shot_detector_srv::Response &res)
{
    pcl_functions::convertMsg(req.targetcloud, model);
    //loadModel(*model,"/home/niko/projects/apc/catkin/src/apc_ros/apc_object_detection/optimized_poisson_textured_mesh.ply");
    pcl_functions::convertMsg(req.pointcloud, scene);
    //pcl::io::loadPCDFile("/home/niko/projects/apc/catkin/src/apc_ros/apc_object_detection/niko_file.pcd",*scene);
    std::cerr << "Originally positions" << std::endl;
    std::cerr << scene->points[1].x << std::endl;
    std::cerr << scene->points[1].y << std::endl;
    std::cerr << scene->points[1].z << std::endl;
    //Downsample the model and the scene so they have rougly the same resolution
    pcl::PointCloud<PointType>::Ptr scene_filter (new pcl::PointCloud<PointType> ());
    pcl_functions::voxelFilter(scene, scene_filter, voxel_sample_);
    scene = scene_filter;
    pcl::PointCloud<PointType>::Ptr model_filter (new pcl::PointCloud<PointType> ());
    pcl_functions::voxelFilter(model, model_filter, voxel_sample_);
    model = model_filter;
    // Randomly select a couple of keypoints so we don't calculte descriptors for everything
    sampleKeypoints(model, model_keypoints, model_ss_);
    sampleKeypoints(scene, scene_keypoints, scene_ss_);
    //Calculate the Normals
    calcNormals(model, model_normals);
    calcNormals(scene, scene_normals);
    //Calculate the shot descriptors at each keypoint in the scene
    calcSHOTDescriptors(model, model_keypoints, model_normals, model_descriptors);
    calcSHOTDescriptors(scene, scene_keypoints, scene_normals, scene_descriptors);
    ransac(rototranslations, model, scene);
    // Compare descriptors and try to find correspondences
    //  compare(model_descriptors,scene_descriptors);
    Eigen::Matrix4f pose;
    // if(model_scene_corrs->size ()!=0){
    //groupCorrespondences();
    pose = refinePose(rototranslations, model, scene);

    //}
    std::cerr << pose << std::endl;
    if(pose == Eigen::Matrix4f::Identity())
        return false;
    Eigen::Matrix4d md(pose.cast<double>());
    Eigen::Affine3d affine(md);
    geometry_msgs::Pose transform;
    tf::poseEigenToMsg(affine, transform);
    res.pose = transform;
    return true;
}

/*!
 * \brief shot_detector::refinePose Refines the poses through icp & hypothesis verification and returns the final pose
 * \param transforms vector of current poses gotten through clustering of correspondences
 * \param model the model whose pose we are trying to find
 * \param scene
 * \return
 */
Eigen::Matrix4f shot_detector::refinePose(std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transforms,
        pcl::PointCloud<PointType>::Ptr model, pcl::PointCloud<PointType>::Ptr scene)
{
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > final_transforms;
    std::cerr << "verifying" << std::endl;
    typedef pcl::visualization::PointCloudColorHandlerCustom<PointType> ColorHandlerT;

    if (transforms.size () <= 0) {
        cerr << "*** No instances found! ***" << endl;
        return Eigen::Matrix4f::Identity();
    } else {
        cerr << "Recognized Instances: " << transforms.size () << endl << endl;
    }

    /**
       * Generates clouds for each instances found
       */
    std::vector<pcl::PointCloud<PointType>::ConstPtr> instances;

    for (size_t i = 0; i < transforms.size (); ++i) {
        pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
        pcl::transformPointCloud (*model, *rotated_model, transforms[i]);
        instances.push_back (rotated_model);
    }

    /**
       * ICP
       */
    std::vector<pcl::PointCloud<PointType>::ConstPtr> registered_instances;
    if (true) {
        cerr << "--- ICP ---------" << endl;

        //for (size_t i = 0; i < transforms.size (); ++i)
        {
            // std::cerr << "initial" << transforms[i] <<std::endl;
            pcl::IterativeClosestPoint<PointType, PointType> icp;
            icp.setMaximumIterations (icp_max_iter_);
            icp.setMaxCorrespondenceDistance (icp_corr_distance_);
            icp.setInputTarget (scene);
            icp.setInputSource (instances[0]);
            pcl::PointCloud<PointType>::Ptr registered (new pcl::PointCloud<PointType>);
            icp.align (*registered);
            icp.setMaxCorrespondenceDistance (.01);
            icp.align (*registered);
            icp.setMaxCorrespondenceDistance (.005);
            icp.align (*registered);
            icp.setMaxCorrespondenceDistance (.001);
            icp.align (*registered);
            //registered_instances.push_back (registered);
            //cerr << "Instance " << i << " ";
            if (icp.hasConverged ()) {
                cerr << "Aligned!" << endl;
                registered_instances.push_back (registered);
                std::cerr << icp.getFinalTransformation() << std::endl;

                final_transforms.push_back(icp.getFinalTransformation()*transforms[0]);
            } else {
                cerr << "Not Aligned!" << endl;
            }
        }

        cerr << "-----------------" << endl << endl;
    }
    /* pcl::visualization::PCLVisualizer visu("icp");
     pcl::PointCloud<PointType>::Ptr notated_model (new pcl::PointCloud<PointType> ());
      visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
      pcl::transformPointCloud (*model, *notated_model, final_transforms[0]);
      visu.addPointCloud (notated_model, ColorHandlerT (notated_model, 0.0, 0.0, 255.0), "object_aligned");
     visu.spin();*/

    /**
       * Hypothesis Verification
       */
    cerr << "--- Hypotheses Verification ---" << endl;
    std::vector<bool> hypotheses_mask;  // Mask Vector to identify positive hypotheses

    pcl::GlobalHypothesesVerification<PointType, PointType> GoHv;

    GoHv.setSceneCloud (scene);  // Scene Cloud
    GoHv.addModels (registered_instances, true);  //Models to verify

    GoHv.setInlierThreshold (hv_inlier_th_);
    GoHv.setOcclusionThreshold (hv_occlusion_th_);
    GoHv.setRegularizer (hv_regularizer_);
    GoHv.setRadiusClutter (hv_rad_clutter_);
    GoHv.setClutterRegularizer (hv_clutter_reg_);
    GoHv.setDetectClutter (hv_detect_clutter_);
    GoHv.setRadiusNormals (hv_rad_normals_);

    GoHv.verify ();
    GoHv.getMask (hypotheses_mask);  // i-element TRUE if hvModels[i] verifies hypotheses

    for (int i = 0; i < hypotheses_mask.size (); i++) {
        if (hypotheses_mask[i]) {
            cerr << "Instance " << i << " is GOOD! <---" << endl;
            std::cerr << final_transforms[i] << std::endl;
            return final_transforms[i];
        } else {
            cerr << "Instance " << i << " is bad!" << endl;
        }
    }
    cerr << "-------------------------------" << endl;
    return Eigen::Matrix4f::Identity();

}

void shot_detector::findCorrespondences(pcl::PointCloud<DescriptorType>::Ptr source, pcl::PointCloud<DescriptorType>::Ptr target, std::vector<int> &correspondences)
{
    cout << "correspondence assignment..." << std::flush;
    correspondences.resize (source->size());

    // Use a KdTree to search for the nearest matches in feature space
    pcl::KdTreeFLANN<DescriptorType> descriptor_kdtree;
    descriptor_kdtree.setInputCloud (target);

    // Find the index of the best match for each keypoint, and store it in "correspondences_out"
    const int k = 1;
    std::vector<int> k_indices (k);
    std::vector<float> k_squared_distances (k);
    for (int i = 0; i < static_cast<int> (source->size ()); ++i) {
        descriptor_kdtree.nearestKSearch (*source, i, k, k_indices, k_squared_distances);
        correspondences[i] = k_indices[0];
    }
    cout << "OK" << endl;
}

void shot_detector::filterCorrespondences()
{
    cerr << "correspondence rejection..." << std::endl;
    std::vector<std::pair<unsigned, unsigned> > correspondences;
    for (unsigned cIdx = 0; cIdx < source2target_.size (); ++cIdx)
        if (target2source_[source2target_[cIdx]] == static_cast<int> (cIdx))
            correspondences.push_back(std::make_pair(cIdx, source2target_[cIdx]));

    std::cerr << correspondences.size() << std::endl;
    correspondences_->resize (correspondences.size());
    for (unsigned cIdx = 0; cIdx < correspondences.size(); ++cIdx) {
        (*correspondences_)[cIdx].index_query = correspondences[cIdx].first;
        (*correspondences_)[cIdx].index_match = correspondences[cIdx].second;
    }

    pcl::registration::CorrespondenceRejectorSampleConsensus<PointType> rejector;
    rejector.setInputSource(model_keypoints);
    rejector.setInputTarget(scene_keypoints);
    rejector.setInputCorrespondences(correspondences_);
    rejector.getCorrespondences(*correspondences_);
    cout << "OK" << endl;
}

void shot_detector::loadParameters()
{
    double model_ss;
    double scene_ss;
    double rf_rad;
    double descr_rad;
    double cg_size;
    double cg_thresh;
    double corr_dist;
    double voxel_sample;
    double icp_corr_distance;
    int icp_max_iter;
    double ran_inlier_dist;
    int ran_corr_random;
    int ran_sample_num;
    double ran_sim_thresh;
    double ran_max_corr_dist;
    int ran_max_iter;

    nh.param<double>("/detect_objects_pcl/model_sample", model_ss, .01);
    nh.param<double>("/detect_objects_pcl/scene_sample", scene_ss, .01);
    nh.param<double>("/detect_objects_pcl/descriptor_rad", descr_rad, .06);
    nh.param<double>("/detect_objects_pcl/rf_rad", rf_rad, .015);
    nh.param<double>("/detect_objects_pcl/cg_size", cg_size, .04);
    nh.param<double>("/detect_objects_pcl/cg_thresh", cg_thresh, 8.0);
    nh.param<double>("/detect_objects_pcl/corr_dist", corr_dist, .75);
    nh.param<double>("/detect_objects_pcl/voxel_leaf", voxel_sample, .01);
    nh.param<double>("/detect_objects_pcl/icp_correlation_dist", icp_corr_distance, .05);
    nh.param<int>("/detect_objects_pcl/icp_max_iter", icp_max_iter, 100);
    nh.param<double>("/detect_objects_pcl/ran_inlier_dist", ran_inlier_dist, 0.25);
    nh.param<int>("/detect_objects_pcl/ran_corr_random", ran_corr_random, 2.0);
    nh.param<int>("/detect_objects_pcl/ran_sample_num", ran_sample_num, 5.0);
    nh.param<double>("/detect_objects_pcl/ran_sim_thresh", ran_sim_thresh, 0.6);
    nh.param<double>("/detect_objects_pcl/ran_max_corr_dist", ran_max_corr_dist, .25);
    nh.param<int>("/detect_objects_pcl/ran_max_iter", ran_max_iter, 3000);
    model_ss_ = static_cast<float>(model_ss);
    scene_ss_ = static_cast<float>(scene_ss);
    rf_rad_ = static_cast<float>(rf_rad);
    descr_rad_ = static_cast<float>(descr_rad);
    cg_size_ = static_cast<float>(cg_size);
    cg_thresh_ = static_cast<float>(cg_thresh);
    corr_dist_ = static_cast<float>(corr_dist);
    voxel_sample_ = static_cast<float>(voxel_sample);
    icp_corr_distance_ = icp_corr_distance;
    icp_max_iter_ = icp_max_iter;
    ran_inlier_dist_ = static_cast<float>(ran_inlier_dist);
    ran_corr_random_ = ran_corr_random;
    ran_sim_thresh_ = static_cast<float>(ran_sim_thresh);
    ran_sample_num_ = ran_sample_num;
    ran_max_corr_dist_ = static_cast<float>(ran_corr_random);
    ran_max_iter_ = ran_max_iter;
}

void shot_detector::PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &data_msg)
{
    data = true;
    activated = true;
    depth_msg = *data_msg;
}


void shot_detector::calcNormals(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<NormalType>::Ptr normals)
{
    norm_est.setInputCloud(cloud);
    norm_est.compute(*normals);
}

void shot_detector::calcSHOTDescriptors(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr keypoints
                                        , pcl::PointCloud<NormalType>::Ptr normals, pcl::PointCloud<DescriptorType>::Ptr descriptors)
{
    //descr_est.setInputCloud (keypoints);
    descr_est.setInputCloud(cloud);
    descr_est.setInputNormals (normals);
    descr_est.setSearchSurface (cloud);
    descr_est.compute (*descriptors);
}

/*void shot_detector::calcPFHRGBDescriptors(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr keypoints
                                          , pcl::PointCloud<NormalType>::Ptr normals,
                                          pcl::PointCloud<DescriptorType>::Ptr descriptors)
{
    pfhrgbEstimation.setInputCloud (keypoints);
   // Provide the point cloud with normals
   pfhrgbEstimation.setInputNormals(normals);
   // Use the same KdTree from the normal estimation
   pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
   pfhrgbEstimation.setSearchMethod (tree);
   pfhrgbEstimation.setRadiusSearch (0.1);
   //pfhrgbEstimation.setKSearch (100);
   pfhrgbEstimation.compute (*descriptors);
}*/


void shot_detector::sampleKeypoints(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr keypoints, float sample_size)
{
    pcl::PointCloud<int> sampled_indices;
    uniform_sampling.setInputCloud (cloud);
    uniform_sampling.setRadiusSearch (sample_size);
    uniform_sampling.compute (sampled_indices);
    pcl::copyPointCloud (*cloud, sampled_indices.points, *keypoints);
}

void shot_detector::compare(pcl::PointCloud<DescriptorType>::Ptr model_descriptions, pcl::PointCloud<DescriptorType>::Ptr scene_descriptions)
{
    model_scene_corrs->clear();
    pcl::KdTreeFLANN<DescriptorType> match_search;
    match_search.setInputCloud (model_descriptions);
    std::cerr << scene_descriptions->size() << " and  " << model_descriptions->size() << std::endl;
    model_good_keypoints_indices.clear();
    scene_good_keypoints_indices.clear();

    //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
    for (size_t i = 0; i < scene_descriptions->size (); ++i) {
        std::vector<int> neigh_indices (1);
        std::vector<float> neigh_sqr_dists (1);
        if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0])) { //skipping NaNs
            continue;
        }
        int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
        if(found_neighs == 1 && neigh_sqr_dists[0] < corr_dist_) { //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
            pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
            model_scene_corrs->push_back (corr);
        }
    }
    pcl::copyPointCloud (*model_keypoints, model_good_keypoints_indices, *model_good_kp);
    pcl::copyPointCloud (*scene_keypoints, scene_good_keypoints_indices, *scene_good_kp);
    std::cerr << "Correspondences found: " << model_scene_corrs->size () << std::endl;

}

void shot_detector::groupCorrespondences()
{
    //
    //  Compute (Keypoints) Reference Frames only for Hough
    //

    pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
    pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch (rf_rad_);

    rf_est.setInputCloud (model_keypoints);
    rf_est.setInputNormals (model_normals);
    rf_est.setSearchSurface (model);
    rf_est.compute (*model_rf);

    rf_est.setInputCloud (scene_keypoints);
    rf_est.setInputNormals (scene_normals);
    rf_est.setSearchSurface (scene);
    rf_est.compute (*scene_rf);

    //  Clustering
    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
    clusterer.setHoughBinSize (cg_size_);
    clusterer.setHoughThreshold (cg_thresh_);
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);

    clusterer.setInputCloud (model_keypoints);
    clusterer.setInputRf (model_rf);
    clusterer.setSceneCloud (scene_keypoints);
    clusterer.setSceneRf (scene_rf);
    clusterer.setModelSceneCorrespondences (model_scene_corrs);

    //clusterer.cluster (clustered_corrs);
    clusterer.recognize (rototranslations, clustered_corrs);
    /*gc_clusterer.setGCSize (cg_size_);
    gc_clusterer.setGCThreshold (cg_thresh_);

    gc_clusterer.setInputCloud (model_keypoints);
    gc_clusterer.setSceneCloud (scene_keypoints);
    gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

    gc_clusterer.cluster (clustered_corrs);
    gc_clusterer.recognize (rototranslations, clustered_corrs);*/
}

void shot_detector::visualizeCorrespondences()
{
    std::cerr << "Model instances found: " << rototranslations.size () << std::endl;
    for (size_t i = 0; i < rototranslations.size (); ++i) {
        std::cerr << "\n    Instance " << i + 1 << ":" << std::endl;
        std::cerr << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

        // Print the rotation matrix and translation vector
        Eigen::Matrix3f rotation = rototranslations[i].block<3, 3>(0, 0);
        Eigen::Vector3f translation = rototranslations[i].block<3, 1>(0, 3);

        printf ("\n");
        printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0, 0), rotation (0, 1), rotation (0, 2));
        printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1, 0), rotation (1, 1), rotation (1, 2));
        printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2, 0), rotation (2, 1), rotation (2, 2));
        printf ("\n");
        printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
    }

    //
    //  Visualization
    //
    pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
    viewer.addCoordinateSystem (1.0);
    viewer.addPointCloud (scene, "scene_cloud");

    pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

    // if (show_correspondences_ || show_keypoints_)
    // {
    //  We are translating the model so that it doesn't end in the middle of the scene representation
    pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1, 0, 0), Eigen::Quaternionf (1, 0, 0, 0));
    pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1, 0, 0), Eigen::Quaternionf (1, 0, 0, 0));

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
    viewer.addPointCloud (off_scene_model, "off_scene_model");
    // }

    //if (show_keypoints_)
    //  {
    /*pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
        viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");*/

    /* pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
     viewer.addPointCloud (off_scene_model_keypoints, "off_scene_model_keypoints");
     viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");*/
    // }
    pcl::PointCloud<PointType>::Ptr scene_corr (new pcl::PointCloud<PointType> ());
    for (int idx = 0; idx < model_scene_corrs->size(); ++idx) {
        PointType temp = scene_keypoints->at(model_scene_corrs->at(idx).index_match);
        scene_corr->push_back(temp);
    }
    pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_corr_color_handler (scene_corr, 255, 0, 0);
    viewer.addPointCloud (scene_corr, scene_corr_color_handler, "scene_corr");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_corr");
    for (size_t i = 0; i < rototranslations.size (); ++i) {
        pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
        pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);

        std::stringstream ss_cloud;
        ss_cloud << "instance" << i;

        pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
        viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());

        //if (show_correspondences_)
        //{
        pcl::PointCloud<PointType>::Ptr scene_corr (new pcl::PointCloud<PointType> ());
        for (size_t j = 0; j < clustered_corrs[i].size (); ++j) {
            std::stringstream ss_line;
            ss_line << "correspondence_line" << i << "_" << j;
            PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
            PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

            //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
            viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
        }
        // }
    }

    while (!viewer.wasStopped ()) {
        viewer.spinOnce ();
    }
}

void shot_detector::visualizeICP()
{
    std::cerr << "verifying" << std::endl;
    if (rototranslations.size () <= 0) {
        cerr << "*** No instances found! ***" << endl;
        return;
    } else {
        cerr << "Recognized Instances: " << rototranslations.size () << endl << endl;
    }

    /**
       * Generates clouds for each instances found
       */
    std::vector<pcl::PointCloud<PointType>::ConstPtr> instances;

    for (size_t i = 0; i < rototranslations.size (); ++i) {
        pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
        pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);
        instances.push_back (rotated_model);
    }
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > final_transforms;
    /**
       * ICP
       */
    std::vector<pcl::PointCloud<PointType>::ConstPtr> registered_instances;
    if (true) {
        cerr << "--- ICP ---------" << endl;

        for (size_t i = 0; i < rototranslations.size (); ++i) {
            pcl::IterativeClosestPoint<PointType, PointType> icp;
            icp.setMaximumIterations (icp_max_iter_);
            icp.setMaxCorrespondenceDistance (icp_corr_distance_);
            icp.setInputTarget (scene);
            icp.setInputSource (instances[i]);
            pcl::PointCloud<PointType>::Ptr registered (new pcl::PointCloud<PointType>);
            icp.align (*registered);
            icp.setMaxCorrespondenceDistance (.01);
            icp.align (*registered);
            icp.setMaxCorrespondenceDistance (.005);
            icp.align (*registered);
            registered_instances.push_back (registered);
            std::cerr << rototranslations[i] << std::endl;
            final_transforms.push_back(icp.getFinalTransformation());
            std::cerr << "answer" << icp.getFinalTransformation()* rototranslations[i] << std::endl;
            cerr << "Instance " << i << std::endl;
            if (icp.hasConverged ()) {
                cerr << "Aligned!" << endl;
            } else {
                cerr << "Not Aligned!" << endl;
            }
        }

        cerr << "-----------------" << endl << endl;
    }

    /**
       * Hypothesis Verification
       */
    cerr << "--- Hypotheses Verification ---" << endl;
    std::vector<bool> hypotheses_mask;  // Mask Vector to identify positive hypotheses

    pcl::GlobalHypothesesVerification<PointType, PointType> GoHv;

    GoHv.setSceneCloud (scene);  // Scene Cloud
    GoHv.addModels (registered_instances, true);  //Models to verify

    GoHv.setInlierThreshold (hv_inlier_th_);
    GoHv.setOcclusionThreshold (hv_occlusion_th_);
    GoHv.setRegularizer (hv_regularizer_);
    GoHv.setRadiusClutter (hv_rad_clutter_);
    GoHv.setClutterRegularizer (hv_clutter_reg_);
    GoHv.setDetectClutter (hv_detect_clutter_);
    GoHv.setRadiusNormals (hv_rad_normals_);

    GoHv.verify ();
    GoHv.getMask (hypotheses_mask);  // i-element TRUE if hvModels[i] verifies hypotheses

    for (int i = 0; i < hypotheses_mask.size (); i++) {
        if (hypotheses_mask[i]) {
            cerr << "Instance " << i << " is GOOD! <---" << endl;
        } else {
            cerr << "Instance " << i << " is bad!" << endl;
        }
    }
    cerr << "-------------------------------" << endl;

    /**
       *  Visualization
       */
    pcl::visualization::PCLVisualizer viewer ("Hypotheses Verification");
    viewer.addPointCloud (scene, "scene_cloud");
    std::cerr << scene->points[1].x << scene->points[1].y << scene->points[1].z << std::endl;
    viewer.addCoordinateSystem (1.0);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_cloud");

    /* pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
      pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

      pcl::PointCloud<PointType>::Ptr off_model_good_kp (new pcl::PointCloud<PointType> ());
      pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1, 0, 0), Eigen::Quaternionf (1, 0, 0, 0));
      pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1, 0, 0), Eigen::Quaternionf (1, 0, 0, 0));
      pcl::transformPointCloud (*model_good_kp, *off_model_good_kp, Eigen::Vector3f (-1, 0, 0), Eigen::Quaternionf (1, 0, 0, 0));*/

    /* //if (show_keypoints_)
      {
        CloudStyle modelStyle = style_white;
        pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, modelStyle.r, modelStyle.g, modelStyle.b);
        viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, modelStyle.size, "off_scene_model");
      }

     // if (show_keypoints_)
      {
        CloudStyle goodKeypointStyle = style_violet;
        pcl::visualization::PointCloudColorHandlerCustom<PointType> model_good_keypoints_color_handler (off_model_good_kp, goodKeypointStyle.r, goodKeypointStyle.g,
                                                                                                        goodKeypointStyle.b);
        viewer.addPointCloud (off_model_good_kp, model_good_keypoints_color_handler, "model_good_keypoints");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, goodKeypointStyle.size, "model_good_keypoints");

        pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_good_keypoints_color_handler (scene_good_kp, goodKeypointStyle.r, goodKeypointStyle.g,
                                                                                                        goodKeypointStyle.b);
        viewer.addPointCloud (scene_good_kp, scene_good_keypoints_color_handler, "scene_good_keypoints");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, goodKeypointStyle.size, "scene_good_keypoints");
      }
    */
    for (size_t i = 0; i < instances.size (); ++i) {
        std::stringstream ss_instance;
        ss_instance << "instance_" << i;
        /*
        CloudStyle clusterStyle = style_red;
        pcl::visualization::PointCloudColorHandlerCustom<PointType> instance_color_handler (instances[i], clusterStyle.r, clusterStyle.g, clusterStyle.b);
        viewer.addPointCloud (instances[i], instance_color_handler, ss_instance.str ());
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, clusterStyle.size, ss_instance.str ());
        */
        CloudStyle registeredStyles = hypotheses_mask[i] ? style_green : style_cyan;
        if(hypotheses_mask[i]) {
            std::cerr << final_transforms[i] << std::endl;
            ss_instance << "_registered" << endl;
            pcl::visualization::PointCloudColorHandlerCustom<PointType> registered_instance_color_handler (registered_instances[i], registeredStyles.r,
                    registeredStyles.g, registeredStyles.b);
            viewer.addPointCloud (registered_instances[i], registered_instance_color_handler, ss_instance.str ());
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, registeredStyles.size, ss_instance.str ());
        }
    }

    while (!viewer.wasStopped ()) {
        viewer.spinOnce ();
    }
}


double shot_detector::computeCloudResolution(const pcl::PointCloud<PointType>::Ptr cloud)
{
    double resolution = 0.0;
    int numberOfPoints = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> squaredDistances(2);
    pcl::search::KdTree<PointType> tree;
    tree.setInputCloud(cloud);

    for (size_t i = 0; i < cloud->size(); ++i) {
        if (! pcl_isfinite((*cloud)[i].x))
            continue;

        // Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
        if (nres == 2) {
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
    ros::init (argc, argv, "apc_object_detection");
    shot_detector detector;
    // detector.processImage();
    // Spin
    std::cerr << "ros start" << std::endl;
    //detector.processImage();
    ros::spin();
    std::cerr << "End" << std::endl;
    return 0;
}

