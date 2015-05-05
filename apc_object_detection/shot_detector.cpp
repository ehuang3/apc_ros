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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/recognition/hv/hv_go.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

shot_detector::shot_detector()
{
    data=false;
    activated=false;
    std::cerr  << "Starting" << std::endl;
    loadParameters();
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
    //pcl::PointCloud<FeatureType>::Ptr model_descriptors_ (new pcl::PointCloud<FeatureType> ());
    //pcl::PointCloud<FeatureType>::Ptr scene_descriptors_ (new pcl::PointCloud<FeatureType> ());
    model=model_;
    model_keypoints=model_keypoints_;
    scene =scene_;
    scene_keypoints=scene_keypoints_;
    model_normals =model_normals_;
    scene_normals =scene_normals_;
    model_descriptors =model_descriptors_;
    scene_descriptors =scene_descriptors_;
    model_scene_corrs=model_scene_corrs_;
    model_good_kp=model_good_kp_;
    scene_good_kp=scene_good_kp_;
    objects=objects_;
    pcl::CorrespondencesPtr correspondences (new pcl::Correspondences);
    correspondences_=correspondences;


    std::string filename="/home/niko/projects/apc/catkin/src/apc_object_detection/optimized_poisson_textured_mesh.ply";
     vtkSmartPointer<vtkPLYReader> reader =
      vtkSmartPointer<vtkPLYReader>::New();

  vtkSmartPointer<vtkPolyData> data=vtkSmartPointer<vtkPolyData>::New();
    reader->SetFileName ( filename.c_str() );
      reader->Update();
      data=reader->GetOutput();

    pcl::io::vtkPolyDataToPointCloud(data,*model);

    std::cerr << model->size() << std::endl;
    std::cerr  << "Model created" << std::endl;

    nh=ros::NodeHandle("apc_object_detection");
    kinect=nh.subscribe("/camera/depth_registered/points", 1, &shot_detector::PointCloudCallback,this);
    //kinect = nh.subscribe<sensor_msgs::PointCloud2ConstPtr>
      //      ("/camera/depth/points", 1,boost::bind(&shot_detector::PointCloudCallback,this, _1) );

    pcl::PointCloud<PointType>::Ptr model_filter (new pcl::PointCloud<PointType> ());
    voxelFilter(model,model_filter,voxel_sample_);
    model=model_filter;
            std::cerr << computeCloudResolution(model) << std::endl;
    norm_est.setKSearch (10);
    descr_est.setRadiusSearch (descr_rad_);
    downsample(model,model_keypoints,model_ss_);
    calcNormals(model,model_normals);
    calcSHOTDescriptors(model,model_keypoints,model_normals,model_descriptors);
    //calcPFHRGBDescriptors(model,model_keypoints,model_normals,model_descriptors);
    std::cerr << "Model stuff calculated" <<std::endl;
}

void shot_detector::processImage()
{
    if(data==true){
        convertMsg(depth_msg,scene);
        pcl::PointCloud<PointType>::Ptr scene_filter (new pcl::PointCloud<PointType> ());
        filter(scene,scene_filter);
        voxelFilter(scene_filter,scene,voxel_sample_);
        scene_filter.reset();
        norm_est.setInputCloud(scene);
        std::cerr << computeCloudResolution(scene) << std::endl;
        std::cerr << "input" << scene->size() << std::endl;;
        std::cerr << "normals" << std::endl;
        std::cerr << scene_ss_ << std::endl;
            descr_est.setRadiusSearch (descr_rad_);
        downsample(scene,scene_keypoints,scene_ss_);
        calcNormals(scene,scene_normals);
               // std::cerr << scene_keypoints.size() << " and " << model_keypoints.size() << std::endl;
        std::cerr << "downsample" << std::endl;
        calcSHOTDescriptors(scene,scene_keypoints,scene_normals,scene_descriptors);
        //calcPFHRGBDescriptors(scene,scene_keypoints,scene_normals,scene_descriptors);
        std::cerr << "descriptors" << std::endl;
        //compare(model_descriptors,scene_descriptors);
        std::cerr << scene_descriptors->size() << " and  "<< model_descriptors->size() << std::endl;
        findCorrespondences (model_descriptors, scene_descriptors, source2target_);
        findCorrespondences (scene_descriptors, model_descriptors, target2source_);

        filterCorrespondences ();
        model_scene_corrs=correspondences_;
        if(model_scene_corrs->size ()!=0){
        houghGrouping();
        output();
        outwithver();
        }
    }
}
void shot_detector::extractClusters()
{
    if(data==true)
    {
        convertMsg(depth_msg,scene);
        pcl::PointCloud<PointType>::Ptr scene_filter (new pcl::PointCloud<PointType> ());
        filter(scene,scene_filter);
        voxelFilter(scene_filter,scene,voxel_sample_);
        scene_filter.reset();

        planarExtraction( scene,objects);
    }
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
    for (int i = 0; i < static_cast<int> (source->size ()); ++i)
    {
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
    for (unsigned cIdx = 0; cIdx < correspondences.size(); ++cIdx)
    {
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
    nh.param<double>("/apc_object_detection/model_sample",model_ss,.01);
    nh.param<double>("/apc_object_detection/scene_sample",scene_ss,.01);
    nh.param<double>("/apc_object_detection/descriptor_rad",descr_rad,.06);
    nh.param<double>("/apc_object_detection/rf_rad",rf_rad,.015);
    nh.param<double>("/apc_object_detection/cg_size",cg_size,.04);
    nh.param<double>("/apc_object_detection/cg_thresh",cg_thresh,8.0);
    nh.param<double>("/apc_object_detection/corr_dist",corr_dist,.75);
    nh.param<double>("/apc_object_detection/voxel_leaf",voxel_sample,.01);
    model_ss_=static_cast<float>(model_ss);
    scene_ss_=static_cast<float>(scene_ss);
    rf_rad_=static_cast<float>(rf_rad);
    descr_rad_=static_cast<float>(descr_rad);
    cg_size_=static_cast<float>(cg_size);
    cg_thresh_=static_cast<float>(cg_thresh);
    corr_dist_=static_cast<float>(corr_dist);
    voxel_sample_=static_cast<float>(voxel_sample);
}

void shot_detector::PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &data_msg)
{
    data=true;
    activated=true;
    depth_msg=*data_msg;
}

void shot_detector::convertMsg(sensor_msgs::PointCloud2 data_msg, pcl::PointCloud<PointType>::Ptr cloud)
{
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(data_msg, pcl_pc);
    std::cerr << "convert" << std::endl;
    pcl::PointCloud<PointType>::Ptr image_cloud(new pcl::PointCloud<PointType> ());
    pcl::fromPCLPointCloud2(pcl_pc, *image_cloud);
    std::cerr << image_cloud->size() << std::endl;
    scene=pcl::PointCloud<PointType>::Ptr (image_cloud);
}

void shot_detector::calcNormals(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<NormalType>::Ptr normals)
{
    norm_est.setInputCloud(cloud);
    norm_est.compute(*normals);
}

void shot_detector::calcSHOTDescriptors(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr keypoints
                                    , pcl::PointCloud<NormalType>::Ptr normals, pcl::PointCloud<DescriptorType>::Ptr descriptors)
{
    descr_est.setInputCloud (keypoints);
    descr_est.setInputNormals (normals);
    descr_est.setSearchSurface (cloud);
    descr_est.compute (*descriptors);
}

void shot_detector::calcPFHRGBDescriptors(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr keypoints
                                          , pcl::PointCloud<NormalType>::Ptr normals,
                                          pcl::PointCloud<FeatureType>::Ptr descriptors)
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
}


void shot_detector::downsample(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr keypoints,float sample_size)
{
    pcl::PointCloud<int> sampled_indices;
    uniform_sampling.setInputCloud (cloud);
    uniform_sampling.setRadiusSearch (sample_size);
    uniform_sampling.compute (sampled_indices);
    pcl::copyPointCloud (*cloud, sampled_indices.points, *keypoints);
}

void shot_detector::compare(pcl::PointCloud<FeatureType>::Ptr model_descriptions, pcl::PointCloud<FeatureType>::Ptr scene_descriptions)
{
    model_scene_corrs->clear();
    pcl::KdTreeFLANN<FeatureType> match_search;
    match_search.setInputCloud (model_descriptions);
    std::cerr << scene_descriptions->size() << " and  "<< model_descriptions->size() << std::endl;
    model_good_keypoints_indices.clear();
    scene_good_keypoints_indices.clear();

    //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
    for (size_t i = 0; i < scene_descriptions->size (); ++i)
    {
      std::vector<int> neigh_indices (1);
      std::vector<float> neigh_sqr_dists (1);
      if (!pcl_isfinite (scene_descriptions->at (i).histogram[0])) //skipping NaNs
      {
        continue;
      }
      int found_neighs = match_search.nearestKSearch (scene_descriptions->at (i), 1, neigh_indices, neigh_sqr_dists);
      if(found_neighs == 1 && neigh_sqr_dists[0] < corr_dist_) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
      {
          std::cerr << "YAHTAAAAAAAAAAAAAAAAAAAAAH" << std::endl;
        pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
        model_scene_corrs->push_back (corr);
        model_good_keypoints_indices.push_back (corr.index_query);
              scene_good_keypoints_indices.push_back (corr.index_match);
      }
    }
    pcl::copyPointCloud (*model_keypoints, model_good_keypoints_indices, *model_good_kp);
      pcl::copyPointCloud (*scene_keypoints, scene_good_keypoints_indices, *scene_good_kp);
    std::cerr << "Correspondences found: " << model_scene_corrs->size () << std::endl;

}

void shot_detector::houghGrouping()
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

        //gc_clusterer.cluster (clustered_corrs);
        gc_clusterer.recognize (rototranslations, clustered_corrs);*/
}

void shot_detector::output()
{
    std::cerr << "Model instances found: " << rototranslations.size () << std::endl;
      for (size_t i = 0; i < rototranslations.size (); ++i)
      {
        std::cerr << "\n    Instance " << i + 1 << ":" << std::endl;
        std::cerr << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

        // Print the rotation matrix and translation vector
        Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
        Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

        printf ("\n");
        printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
        printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
        printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
        printf ("\n");
        printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
      }

      //
      //  Visualization
      //
      pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
      viewer.addPointCloud (scene, "scene_cloud");

      pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
      pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

    // if (show_correspondences_ || show_keypoints_)
     // {
        //  We are translating the model so that it doesn't end in the middle of the scene representation
        pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
        pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

        pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
        viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
     // }

      //if (show_keypoints_)
    //  {
        /*pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
        viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");*/

        pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
        viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
     // }
                pcl::PointCloud<PointType>::Ptr scene_corr (new pcl::PointCloud<PointType> ());
                for (int idx=0;idx< model_scene_corrs->size();++idx)
                {
                    PointType temp =scene_keypoints->at(model_scene_corrs->at(idx).index_match);
                    scene_corr->push_back(temp);
                }
        pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_corr_color_handler (scene_corr,255, 0, 0);
        viewer.addPointCloud (scene_corr, scene_corr_color_handler, "scene_corr");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_corr");
      for (size_t i = 0; i < rototranslations.size (); ++i)
      {
        pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
        pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);

        std::stringstream ss_cloud;
        ss_cloud << "instance" << i;

        pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
        viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());

        //if (show_correspondences_)
        //{
        pcl::PointCloud<PointType>::Ptr scene_corr (new pcl::PointCloud<PointType> ());
          for (size_t j = 0; j < clustered_corrs[i].size (); ++j)
          {
            std::stringstream ss_line;
            ss_line << "correspondence_line" << i << "_" << j;
            PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
            PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

            //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
            viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
          }
       // }
      }

      while (!viewer.wasStopped ())
      {
        viewer.spinOnce ();
      }
}

void shot_detector::outwithver()
{
    std::cerr << "verifying" << std::endl;
    if (rototranslations.size () <= 0)
      {
        cerr << "*** No instances found! ***" << endl;
        return;
      }
      else
      {
        cerr << "Recognized Instances: " << rototranslations.size () << endl << endl;
      }

      /**
       * Generates clouds for each instances found
       */
      std::vector<pcl::PointCloud<PointType>::ConstPtr> instances;

      for (size_t i = 0; i < rototranslations.size (); ++i)
      {
        pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
        pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);
        instances.push_back (rotated_model);
      }

      /**
       * ICP
       */
      std::vector<pcl::PointCloud<PointType>::ConstPtr> registered_instances;
      if (true)
      {
        cerr << "--- ICP ---------" << endl;

        for (size_t i = 0; i < rototranslations.size (); ++i)
        {
          pcl::IterativeClosestPoint<PointType, PointType> icp;
          icp.setMaximumIterations (icp_max_iter_);
          icp.setMaxCorrespondenceDistance (icp_corr_distance_);
          icp.setInputTarget (scene);
          icp.setInputSource (instances[i]);
          pcl::PointCloud<PointType>::Ptr registered (new pcl::PointCloud<PointType>);
          icp.align (*registered);
          registered_instances.push_back (registered);
          cerr << "Instance " << i << " ";
          if (icp.hasConverged ())
          {
            cerr << "Aligned!" << endl;
          }
          else
          {
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

      for (int i = 0; i < hypotheses_mask.size (); i++)
      {
        if (hypotheses_mask[i])
        {
          cerr << "Instance " << i << " is GOOD! <---" << endl;
        }
        else
        {
          cerr << "Instance " << i << " is bad!" << endl;
        }
      }
      cerr << "-------------------------------" << endl;

      /**
       *  Visualization
       */
      pcl::visualization::PCLVisualizer viewer ("Hypotheses Verification");
      viewer.addPointCloud (scene, "scene_cloud");

      pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
      pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

      pcl::PointCloud<PointType>::Ptr off_model_good_kp (new pcl::PointCloud<PointType> ());
      pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1, 0, 0), Eigen::Quaternionf (1, 0, 0, 0));
      pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1, 0, 0), Eigen::Quaternionf (1, 0, 0, 0));
      pcl::transformPointCloud (*model_good_kp, *off_model_good_kp, Eigen::Vector3f (-1, 0, 0), Eigen::Quaternionf (1, 0, 0, 0));

      //if (show_keypoints_)
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

      for (size_t i = 0; i < instances.size (); ++i)
      {
        std::stringstream ss_instance;
        ss_instance << "instance_" << i;

        CloudStyle clusterStyle = style_red;
        pcl::visualization::PointCloudColorHandlerCustom<PointType> instance_color_handler (instances[i], clusterStyle.r, clusterStyle.g, clusterStyle.b);
        viewer.addPointCloud (instances[i], instance_color_handler, ss_instance.str ());
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, clusterStyle.size, ss_instance.str ());

        CloudStyle registeredStyles = hypotheses_mask[i] ? style_green : style_cyan;
        ss_instance << "_registered" << endl;
        pcl::visualization::PointCloudColorHandlerCustom<PointType> registered_instance_color_handler (registered_instances[i], registeredStyles.r,
                                                                                                       registeredStyles.g, registeredStyles.b);
        viewer.addPointCloud (registered_instances[i], registered_instance_color_handler, ss_instance.str ());
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, registeredStyles.size, ss_instance.str ());
      }

      while (!viewer.wasStopped ())
      {
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

void shot_detector::voxelFilter(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr filtered_cloud, float sample_size)
{
    voxel_filter.setLeafSize(sample_size,sample_size,sample_size);
    voxel_filter.setInputCloud(cloud);
    voxel_filter.filter(*filtered_cloud);
}


void shot_detector::planarExtraction(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr clustered_objects)
{
    // Objects for storing the point clouds.
    pcl::PointCloud<PointType>::Ptr plane(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr convexHull(new pcl::PointCloud<PointType>);


    // Get the plane model, if present.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<PointType> segmentation;
    segmentation.setInputCloud(cloud);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(0.01);
    segmentation.setOptimizeCoefficients(true);
    pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
    segmentation.segment(*planeIndices, *coefficients);

    if (planeIndices->indices.size() == 0)
        std::cerr << "Could not find a plane in the scene." << std::endl;
    else
    {
        // Copy the points of the plane to a new cloud.
        pcl::ExtractIndices<PointType> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(planeIndices);
        extract.filter(*plane);

        // Retrieve the convex hull.
        pcl::ConvexHull<PointType> hull;
        hull.setInputCloud(plane);
        // Make sure that the resulting hull is bidimensional.
        hull.setDimension(2);
        hull.reconstruct(*convexHull);

        // Redundant check.
        if (hull.getDimension() == 2)
        {
            // Prism object.
            pcl::ExtractPolygonalPrismData<PointType> prism;
            prism.setInputCloud(cloud);
            prism.setInputPlanarHull(convexHull);
            prism.setHeightLimits(0.0f, 0.20f);
            pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);

            prism.segment(*objectIndices);

            // Get and show all points retrieved by the hull.
            extract.setIndices(objectIndices);
            extract.filter(*clustered_objects);
            pcl::visualization::CloudViewer viewerObjects("Objects on table");
            viewerObjects.showCloud(clustered_objects);
            while (!viewerObjects.wasStopped())
            {
                // Do nothing but wait.
            }
        }
        else std::cerr << "The chosen hull is not planar." << std::endl;
    }
}

void shot_detector::filter(pcl::PointCloud<PointType>::Ptr scene, pcl::PointCloud<PointType>::Ptr filtered_scene)
{
    pcl::PassThrough<PointType> pass;
    pass.setInputCloud (scene);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.5);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*filtered_scene);

}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "apc_object_detection");
    shot_detector detector;
      ros::Rate r(10);
  // Spin
    int i=0;
    std::cerr << "ros start" << std::endl;
  while(ros::ok)
  {
    ros::spinOnce();
    std::cerr << "Spin" << std::endl;
    detector.extractClusters();
    i++;
    if (detector.activated==true || i==40)
        return 0;
    r.sleep();
  }
  std::cerr << "End" << std::endl;
}
