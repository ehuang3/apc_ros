///*
//#include <ros/ros.h>
//#include <std_msgs/String.h>
//#include <sensor_msgs/Image.h>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>
//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/highgui/highgui.hpp>


//#include <cctype>
//#include <stdio.h>
//#include <string.h>
//#include <time.h>

//#ifndef _CRT_SECURE_NO_WARNINGS
//# define _CRT_SECURE_NO_WARNINGS
//#endif
//using namespace cv;
//using namespace std;

//Mat rgb_data;
//Mat ir_data;
//bool data_incoming;
//void RGBCallback(const sensor_msgs::ImageConstPtr& msg)
//{
//    std::cerr << "RGB works" << std::endl;
//    cv_bridge::CvImagePtr cv_ptr;
//    try
//    {
//      cv_ptr = cv_bridge::toCvCopy(msg);

//      rgb_data=cv_ptr->image;
//    }
//    catch (cv_bridge::Exception& e)
//    {
//      ROS_ERROR("cv_bridge exception: %s", e.what());
//      return;
//    }
//    data_incoming=true;
//}

//void IRCallback(const sensor_msgs::ImageConstPtr& msg)
//{
//    cv_bridge::CvImagePtr cv_ptr;
//    try
//    {
//      cv_ptr = cv_bridge::toCvCopy(msg);
//      ir_data==cv_ptr->image;
//    }
//    catch (cv_bridge::Exception& e)
//    {
//      ROS_ERROR("cv_bridge exception: %s", e.what());
//      return;
//    }
//}


//class Settings
//{
//public:
//    Settings() : goodInput(false) {}
//    enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
//    enum InputType {INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST};

//    void write(FileStorage& fs) const                        //Write serialization for this class
//    {
//        fs << "{" << "BoardSize_Width"  << boardSize.width
//           << "BoardSize_Height" << boardSize.height
//           << "Square_Size"         << squareSize
//           << "Calibrate_Pattern" << patternToUse
//           << "Calibrate_NrOfFrameToUse" << nrFrames
//           << "Calibrate_FixAspectRatio" << aspectRatio
//           << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
//           << "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

//           << "Write_DetectedFeaturePoints" << bwritePoints
//           << "Write_extrinsicParameters"   << bwriteExtrinsics
//           << "Write_outputFileName"  << outputFileName

//           << "Show_UndistortedImage" << showUndistorsed

//           << "Input_FlipAroundHorizontalAxis" << flipVertical
//           << "Input_Delay" << delay
//           << "Input" << input
//           << "}";
//    }
//    void read(const FileNode& node)                          //Read serialization for this class
//    {
//        node["BoardSize_Width" ] >> boardSize.width;
//        node["BoardSize_Height"] >> boardSize.height;
//        node["Calibrate_Pattern"] >> patternToUse;
//        node["Square_Size"]  >> squareSize;
//        node["Calibrate_NrOfFrameToUse"] >> nrFrames;
//        node["Calibrate_FixAspectRatio"] >> aspectRatio;
//        node["Write_DetectedFeaturePoints"] >> bwritePoints;
//        node["Write_extrinsicParameters"] >> bwriteExtrinsics;
//        node["Write_outputFileName"] >> outputFileName;
//        node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
//        node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
//        node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
//        node["Show_UndistortedImage"] >> showUndistorsed;
//        node["Input"] >> input;
//        node["Input_Delay"] >> delay;
//        interprate();
//    }
//    void interprate()
//    {
//        goodInput = true;
//        if (boardSize.width <= 0 || boardSize.height <= 0)
//        {
//            cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << endl;
//            goodInput = false;
//        }
//        if (squareSize <= 10e-6)
//        {
//            cerr << "Invalid square size " << squareSize << endl;
//            goodInput = false;
//        }

//        inputType=IMAGE_LIST;

//        flag = 0;
//        if(calibFixPrincipalPoint) flag |= CV_CALIB_FIX_PRINCIPAL_POINT;
//        if(calibZeroTangentDist)   flag |= CV_CALIB_ZERO_TANGENT_DIST;
//        if(aspectRatio)            flag |= CV_CALIB_FIX_ASPECT_RATIO;


//        calibrationPattern = NOT_EXISTING;
//        if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = CHESSBOARD;
//        if (!patternToUse.compare("CIRCLES_GRID")) calibrationPattern = CIRCLES_GRID;
//        if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID")) calibrationPattern = ASYMMETRIC_CIRCLES_GRID;
//        if (calibrationPattern == NOT_EXISTING)
//        {
//            cerr << " Inexistent camera calibration mode: " << patternToUse << endl;
//            goodInput = false;
//        }
//        atImageList = 0;

//    }
//    Mat nextImage(bool ir)
//    {
//        Mat result;
//        ros::spinOnce();
//        if(ir==true)
//            result=ir_data;
//        else
//            result=rgb_data;
//        return result;
//    }
//public:
//    Size boardSize;            // The size of the board -> Number of items by width and height
//    Pattern calibrationPattern;// One of the Chessboard, circles, or asymmetric circle pattern
//    float squareSize;          // The size of a square in your defined unit (point, millimeter,etc).
//    int nrFrames;              // The number of frames to use from the input for calibration
//    float aspectRatio;         // The aspect ratio
//    int delay;                 // In case of a video input
//    bool bwritePoints;         //  Write detected feature points
//    bool bwriteExtrinsics;     // Write extrinsic parameters
//    bool calibZeroTangentDist; // Assume zero tangential distortion
//    bool calibFixPrincipalPoint;// Fix the principal point at the center
//    bool flipVertical;          // Flip the captured images around the horizontal axis
//    string outputFileName;      // The name of the file where to write
//    bool showUndistorsed;       // Show undistorted images after calibration
//    string input;               // The input ->



//    int cameraID;
//    vector<string> imageList;
//    int atImageList;
//    VideoCapture inputCapture;
//    InputType inputType;
//    bool goodInput;
//    int flag;

//private:
//    string patternToUse;

//};
//static void read(const FileNode& node, Settings& x, const Settings& default_value = Settings())
//{
//    if(node.empty())
//        x = default_value;
//    else
//        x.read(node);
//}
//enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

//bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,
//                           vector<vector<Point2f> > imagePoints );
//int main(int argc, char* argv[])
//{
//    // Initialize ROS
//    ros::init (argc, argv, "apc_calibration");
//    ros::NodeHandle nh=ros::NodeHandle("apc_opencv_calibration");
//    ros::Subscriber rgb=nh.subscribe("/kinect2/rgb/image", 1, &RGBCallback);
//    ros::Subscriber ir=nh.subscribe("/kinect2/ir/image", 1, &IRCallback);

//    ros::Rate r(10);
//    // Spin
//    int i=0;
//    Settings s;
//    const string inputSettingsFile = argc > 1 ? argv[1] : "/home/hubo/catkin_ws/src/apc_ros/apc_calibration/default.xml";
//    FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
//    if (!fs.isOpened())
//    {
//        cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
//        return -1;
//    }
//    fs["Settings"] >> s;
//    fs.release();                                       // close Settings file

//    if (!s.goodInput)
//    {
//        cout << "Invalid input detected. Application stopping. " << endl;
//        return -1;
//    }


//    std::cerr << "ros start" << std::endl;
//    vector<vector<Point2f> > imagePoints_rgb;
//    vector<vector<Point2f> > imagePoints_ir;
//    Mat cameraMatrix, distCoeffs;
//    Size imageSize;
//    int mode = CAPTURING;
//    clock_t prevTimestamp = 0;
//    const Scalar RED(0,0,255), GREEN(0,255,0);
//    const char ESC_KEY = 27;
//    while(ros::ok)
//    {
//        std::cerr << "Spincool" << mode << std::endl;
//        ros::spinOnce();
//        if(data_incoming==true){
//            Mat view_rgb;
//            Mat view_ir;
//            bool blinkOutput = false;

//            view_rgb = s.nextImage(false);
//            view_ir=s.nextImage(true);

//            //-----  If no more image, or got enough, then stop calibration and show result -------------
//            if( mode == CAPTURING && imagePoints_rgb.size()>=50 )
//            {
//                if( runStereoCalibrationAndSaves, imageSize,  cameraMatrix, distCoeffs, imagePoints_rgb))
//                    mode = CALIBRATED;
//                else
//                    mode = DETECTION;
//            }


//            imageSize = view_rgb.size();  // Format input image.
//            imageSize_ir = view_ir.size();
//            vector<Point2f> pointBuf_rgb;
//            vector<Point2f> pointBuf_ir;

//            bool found;
//            bool found_ir;
//                found = findChessboardCorners( view_rgb, s.boardSize, pointBuf_rgb,
//                                               CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
//                found_ir=findChessboardCorners( view_ir, s.boardSize, pointBuf_ir,
//                                                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

//            if ( found)                // If done with success,
//            {
//                // improve the found corners' coordinate accuracy for chessboard
//                if( s.calibrationPattern == Settings::CHESSBOARD)
//                {
//                    Mat viewGray;
//                    cvtColor(view_rgb, viewGray, COLOR_BGR2GRAY);
//                    cornerSubPix( viewGray, pointBuf_rgb, Size(11,11),
//                                  Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
//                }
//                if ( found_ir)                // If done with success,
//                {
//                    // improve the found corners' coordinate accuracy for chessboard
//                    if( s.calibrationPattern == Settings::CHESSBOARD)
//                    {
//                        Mat viewGray;
//                        cvtColor(view_ir, viewGray, COLOR_BGR2GRAY);
//                        cornerSubPix( viewGray, pointBuf_ir, Size(11,11),
//                                      Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
//                    }
//                }
//            }
//            if(found && found_ir)
//            {
//                if( mode == CAPTURING &&  // For camera only take new samples after delay time
//                        (!s.inputCapture.isOpened() || clock() - prevTimestamp > s.delay*1e-3*CLOCKS_PER_SEC) )
//                {
//                    imagePoints_rgb.push_back(pointBuf_rgb);
//                    imagePoints_ir.push_back(pointBuf_ir);
//                    prevTimestamp = clock();
//                    blinkOutput = data_incoming;//s.inputCapture.isOpened();
//                }

//                // Draw the corners.
//                drawChessboardCorners( view_rgb, s.boardSize, Mat(pointBuf_rgb), found );
//                drawChessboardCorners( view_ir, s.boardSize, Mat(pointBuf_ir), found_ir );
//            }

//            //----------------------------- Output Text ------------------------------------------------
//            string msg = (mode == CAPTURING) ? "100/100" :
//                                               mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
//            int baseLine = 0;
//            Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
//            Point textOrigin(view_rgb.cols - 2*textSize.width - 10, view_rgb.rows - 2*baseLine - 10);

//            if( mode == CAPTURING )
//            {
//                if(s.showUndistorsed)
//                    msg = format( "%d/%d Undist", (int)imagePoints_rgb.size(), s.nrFrames );
//                else
//                    msg = format( "%d/%d", (int)imagePoints_rgb.size(), s.nrFrames );
//            }

//            putText( view_rgb, msg, textOrigin, 1, 1, mode == CALIBRATED ?  GREEN : RED);

//            if( blinkOutput )
//                bitwise_not(view_rgb, view_rgb);

//            //------------------------- Video capture  output  undistorted ------------------------------
//            if( mode == CALIBRATED && s.showUndistorsed )
//            {
//                Mat temp = view_rgb.clone();
//                undistort(temp, view_rgb, cameraMatrix, distCoeffs);
//            }

//            //------------------------------ Show image and check for input commands -------------------
//            imshow("Image View", view_rgb);
//            char key = (char)waitKey(data_incoming ? 50 : s.delay);

//            if( key  == ESC_KEY )
//                break;

//            if( key == 'u' && mode == CALIBRATED )
//                s.showUndistorsed = !s.showUndistorsed;

//            if( data_incoming && key == 'g' )
//            {
//                mode = CAPTURING;
//                imagePoints_rgb.clear();
//            }


//        }
//        r.sleep();
//    }
//        // -----------------------Show the undistorted image for the image list ------------------------
//        if( s.inputType == Settings::IMAGE_LIST && s.showUndistorsed )
//        {
//            Mat view, rview, map1, map2;
//            initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
//                                    getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
//                                    imageSize, CV_16SC2, map1, map2);

//            for(int i = 0; i < (int)s.imageList.size(); i++ )
//            {
//                view = imread(s.imageList[i], 1);
//                if(view.empty())
//                    continue;
//                remap(view, rview, map1, map2, INTER_LINEAR);
//                imshow("Image View", rview);
//                char c = (char)waitKey();
//                if( c  == ESC_KEY || c == 'q' || c == 'Q' )
//                    break;
//            }
//        }


//        return 0;

//}
//static double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
//                                         const vector<vector<Point2f> >& imagePoints,
//                                         const vector<Mat>& rvecs, const vector<Mat>& tvecs,
//                                         const Mat& cameraMatrix , const Mat& distCoeffs,
//                                         vector<float>& perViewErrors)
//{
//    vector<Point2f> imagePoints2;
//    int i, totalPoints = 0;
//    double totalErr = 0, err;
//    perViewErrors.resize(objectPoints.size());

//    for( i = 0; i < (int)objectPoints.size(); ++i )
//    {
//        projectPoints( Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
//                       distCoeffs, imagePoints2);
//        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);

//        int n = (int)objectPoints[i].size();
//        perViewErrors[i] = (float) std::sqrt(err*err/n);
//        totalErr        += err*err;
//        totalPoints     += n;
//    }

//    return std::sqrt(totalErr/totalPoints);
//}

//static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
//                                     Settings::Pattern patternType /*= Settings::CHESSBOARD*/)
//{
//    corners.clear();

//    switch(patternType)
//    {
//    case Settings::CHESSBOARD:
//    case Settings::CIRCLES_GRID:
//        for( int i = 0; i < boardSize.height; ++i )
//            for( int j = 0; j < boardSize.width; ++j )
//                corners.push_back(Point3f(float( j*squareSize ), float( i*squareSize ), 0));
//        break;

//    case Settings::ASYMMETRIC_CIRCLES_GRID:
//        for( int i = 0; i < boardSize.height; i++ )
//            for( int j = 0; j < boardSize.width; j++ )
//                corners.push_back(Point3f(float((2*j + i % 2)*squareSize), float(i*squareSize), 0));
//        break;
//    default:
//        break;
//    }
//}

//static bool runCalibration( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
//                            vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
//                            vector<float>& reprojErrs,  double& totalAvgErr)
//{

//    cameraMatrix = Mat::eye(3, 3, CV_64F);
//    if( s.flag & CV_CALIB_FIX_ASPECT_RATIO )
//        cameraMatrix.at<double>(0,0) = 1.0;

//    distCoeffs = Mat::zeros(8, 1, CV_64F);

//    vector<vector<Point3f> > objectPoints(1);
//    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);

//    objectPoints.resize(imagePoints.size(),objectPoints[0]);

//    //Find intrinsic and extrinsic camera parameters
//    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
//                                 distCoeffs, rvecs, tvecs, s.flag|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);

//    cout << "Re-projection error reported by calibrateCamera: "<< rms << endl;

//    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

//    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
//                                            rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);
//    std::cerr << totalAvgErr << std::endl;

//    return ok;
//}

//// Print camera parameters to the output file
//static void saveCameraParams( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
//                              const vector<Mat>& rvecs, const vector<Mat>& tvecs,
//                              const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
//                              double totalAvgErr )
//{
//    FileStorage fs( s.outputFileName, FileStorage::WRITE );

//    time_t tm;
//    time( &tm );
//    struct tm *t2 = localtime( &tm );
//    char buf[1024];
//    strftime( buf, sizeof(buf)-1, "%c", t2 );

//    fs << "calibration_Time" << buf;

//    if( !rvecs.empty() || !reprojErrs.empty() )
//        fs << "nrOfFrames" << (int)std::max(rvecs.size(), reprojErrs.size());
//    fs << "image_Width" << imageSize.width;
//    fs << "image_Height" << imageSize.height;
//    fs << "board_Width" << s.boardSize.width;
//    fs << "board_Height" << s.boardSize.height;
//    fs << "square_Size" << s.squareSize;

//    if( s.flag & CV_CALIB_FIX_ASPECT_RATIO )
//        fs << "FixAspectRatio" << s.aspectRatio;

//    if( s.flag )
//    {
//        sprintf( buf, "flags: %s%s%s%s",
//                 s.flag & CV_CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "",
//                 s.flag & CV_CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "",
//                 s.flag & CV_CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "",
//                 s.flag & CV_CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "" );
//        cvWriteComment( *fs, buf, 0 );

//    }

//    fs << "flagValue" << s.flag;

//    fs << "Camera_Matrix" << cameraMatrix;
//    fs << "Distortion_Coefficients" << distCoeffs;

//    fs << "Avg_Reprojection_Error" << totalAvgErr;
//    if( !reprojErrs.empty() )
//        fs << "Per_View_Reprojection_Errors" << Mat(reprojErrs);

//    if( !rvecs.empty() && !tvecs.empty() )
//    {
//        CV_Assert(rvecs[0].type() == tvecs[0].type());
//        Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
//        for( int i = 0; i < (int)rvecs.size(); i++ )
//        {
//            Mat r = bigmat(Range(i, i+1), Range(0,3));
//            Mat t = bigmat(Range(i, i+1), Range(3,6));

//            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
//            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
//            //*.t() is MatExpr (not Mat) so we can use assignment operator
//            r = rvecs[i].t();
//            t = tvecs[i].t();
//        }
//        cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
//        fs << "Extrinsic_Parameters" << bigmat;
//    }

//    if( !imagePoints.empty() )
//    {
//        Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
//        for( int i = 0; i < (int)imagePoints.size(); i++ )
//        {
//            Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
//            Mat imgpti(imagePoints[i]);
//            imgpti.copyTo(r);
//        }
//        fs << "Image_points" << imagePtMat;
//    }
//}

//bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,vector<vector<Point2f> > imagePoints )
//{
//    vector<Mat> rvecs, tvecs;
//    vector<float> reprojErrs;
//    double totalAvgErr = 0;

//    bool ok = runCalibration(s,imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs,
//                             reprojErrs, totalAvgErr);
//    cout << (ok ? "Calibration succeeded" : "Calibration failed")
//         << ". avg re projection error = "  << totalAvgErr ;

//    if( ok )
//        saveCameraParams( s, imageSize, cameraMatrix, distCoeffs, rvecs ,tvecs, reprojErrs,
//                          imagePoints, totalAvgErr);
//    return ok;
//}

//bool runStereoCalibrationAndSave(Settings& s, Size imageSize_rgb, Mat&  cameraMatrix_rgb,
//                                 Mat& distCoeffs_rgb,vector<vector<Point2f> > imagePoints_rgb,Size imageSize_ir, Mat&  cameraMatrix_ir,
//                                 Mat& distCoeffs_ir,vector<vector<Point2f> > imagePoints_ir)
//{
//    vector<Mat> rvecs, tvecs,Evecs,Fvecs;
//    cameraMatrix = Mat::eye(3, 3, CV_64F);
//    if( s.flag & CV_CALIB_FIX_ASPECT_RATIO )
//        cameraMatrix.at<double>(0,0) = 1.0;

//    distCoeffs = Mat::zeros(8, 1, CV_64F);

//    vector<vector<Point3f> > objectPoints(1);
//    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);

//    objectPoints.resize(imagePoints.size(),objectPoints[0]);
//    double err=stereoCalibrate( objectPoints, imagePoints_rgb, imagePoints2_ir,
//                     cameraMatrix_rgb, distCoeffs_rgb, cameraMatrix_ir,
//                    distCoeffs_ir,imageSize_rgb, rvecs, tvecs, Evecs, Fvecs);
//}
int main(int argc, char* argv[])
{
    return 0;

}
