#include <jni.h>
#include <string>
#include "System.h"
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <time.h>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include "Plane.h"
#include "Process.h"
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include <android/log.h>
#define  LOG_TAG    "native-dev"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#define LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)

#define BOWISBIN

#ifdef BOWISBIN
ORB_SLAM3::System* SLAM=nullptr;
#endif
std::chrono::steady_clock::time_point t0;
double tframe=0;


cv::Mat Plane2World=cv::Mat::eye(4,4,CV_32F);
cv::Mat Marker2World=cv::Mat::eye(4,4,CV_32F);
cv::Mat centroid;

bool load_as_text(ORB_SLAM3::ORBVocabulary* voc, const std::string infile) {
    clock_t tStart = clock();
    bool res = voc->loadFromTextFile(infile);
    LOGE("Loading fom text: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
    return res;
}

void save_as_binary(ORB_SLAM3::ORBVocabulary* voc, const std::string outfile) {
    clock_t tStart = clock();
    voc->saveToBinaryFile(outfile);
    LOGE("Saving as binary: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

void txt_2_bin(){
    ORB_SLAM3::ORBVocabulary* voc = new ORB_SLAM3::ORBVocabulary();

    load_as_text(voc, "/storage/emulated/0/SLAM/VOC/ORBvoc.txt");
    save_as_binary(voc, "/storage/emulated/0/SLAM/VOC/ORBvoc.bin");
}

cv::Point2f Camera2Pixel(cv::Mat poseCamera,cv::Mat mk){
    return Point2f(
            poseCamera.at<float>(0,0)/poseCamera.at<float>(2,0)*mk.at<float>(0,0)+mk.at<float>(0,2),
            poseCamera.at<float>(1,0)/poseCamera.at<float>(2,0)*mk.at<float>(1,1)+mk.at<float>(1,2)
    );
}

extern "C"
JNIEXPORT jfloatArray JNICALL
Java_com_vslam_orbslam3_vslamactivity_VslamActivity_CVTest(JNIEnv *env, jobject instance, jlong matAddr) {
#ifndef BOWISBIN
    if(tframe == 0)
    txt_2_bin();
    tframe++;
#else

    if(!SLAM)
    { 
        SLAM = new ORB_SLAM3::System("/storage/emulated/0/SLAM/VOC/ORBvoc.bin","/storage/emulated/0/SLAM/Calibration/PARAconfig.yaml",ORB_SLAM3::System::MONOCULAR,false);
       //imageScale = SLAM->GetImageScale();
       
    }
    cv::Mat *pMat = (cv::Mat*)matAddr;
    cv::Mat pose;

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    tframe  = std::chrono::duration_cast < std::chrono::duration < double >> (t1 - t0).count();
    // Pass the image to the SLAM system
    cout << "tframe = " << tframe << endl;
    clock_t start,end;
    start=clock();
    Sophus::SE3f Tcw_SE3f = SLAM->TrackMonocular(*pMat,tframe); // TODO change to monocular_inertial
    Eigen::Matrix4f Tcw_Matrix = Tcw_SE3f.matrix();
    cv::eigen2cv(Tcw_Matrix, pose);
    end = clock();
    LOGI("Get Pose Use Time=%f\n",((double)end-start)/CLOCKS_PER_SEC);

    static bool instialized =false;
    static bool markerDetected =false;
    if(SLAM->MapChanged()){
        instialized = false;
        markerDetected =false;
    }

    if(!pose.empty()){
        cv::Mat rVec;
        cv::Rodrigues(pose.colRange(0, 3).rowRange(0, 3), rVec);
        cv::Mat tVec = pose.col(3).rowRange(0, 3);
        const vector<ORB_SLAM3::MapPoint*> vpMPs = SLAM->mpTracker->mpAtlas->GetAllMapPoints();//所有的地图点
        const vector<ORB_SLAM3::MapPoint*> vpTMPs = SLAM->GetTrackedMapPoints();
        vector<cv::KeyPoint> vKPs = SLAM->GetTrackedKeyPointsUn();
        if (vpMPs.size() > 0) {
            std::vector<cv::Point3f> allmappoints;
            for (size_t i = 0; i < vpMPs.size(); i++) {
                if (vpMPs[i]) {
                    //cv::Point3f pos = cv::Point3f(vpMPs[i]->GetWorldPos());
                    cv::Point3f pos;
                    Mat mpos;// = Mat(pos);
                    cv::eigen2cv(vpMPs[i]->GetWorldPos(),mpos);
                    pos.x = mpos.at<float>(0);
                    pos.y = mpos.at<float>(1);
                    pos.z = mpos.at<float>(2) ;
                    allmappoints.push_back(pos);
                }
            }
        
            std::vector<cv::Point2f> projectedPoints;
            cv::projectPoints(allmappoints, rVec, tVec, SLAM->mpTracker->mK, SLAM->mpTracker->mDistCoef, projectedPoints);
            for (size_t j = 0; j < projectedPoints.size(); ++j) {
                cv::Point2f r1 = projectedPoints[j];
                if(r1.x <640 && r1.x> 0 && r1.y >0 && r1.y <480)
                    cv::circle(*pMat, cv::Point(r1.x, r1.y), 2, cv::Scalar(0, 255, 0), 1, 8);
            }

            if(instialized == false){
                Plane mplane;
                cv::Mat tempTpw,rpw,rwp,tpw,twp;
                LOGE("Detect  Plane");
                tempTpw = mplane.DetectPlane(pose,vpTMPs,50);
                if(!tempTpw.empty()){
                    LOGE("Find  Plane");
                    rpw = tempTpw.rowRange(0,3).colRange(0,3);
                    for(int row = 0 ; row < 4;row++){
                        LOGE(" tempTpw %f %f %f %f",tempTpw.at<float>(row,0),tempTpw.at<float>(row,1),tempTpw.at<float>(row,2),tempTpw.at<float>(row,3));
                    }
                    tpw = tempTpw.col(3).rowRange(0,3);
                    rwp = rpw.t();
                    twp = -rwp*tpw;
                    rwp.copyTo(Plane2World.rowRange(0,3).colRange(0,3));
                    for(int row = 0 ; row < 3;row++){
                        LOGE(" rwp %f %f %f",rwp.at<float>(row,0),rwp.at<float>(row,1),rwp.at<float>(row,2));
                    }
                    twp.copyTo(Plane2World.col(3).rowRange(0,3));
                    for(int row = 0 ; row < 4;row++){
                        LOGE(" Plane2World %f %f %f %f",Plane2World.at<float>(row,0),Plane2World.at<float>(row,1),Plane2World.at<float>(row,2),Plane2World.at<float>(row,3));
                    }
                    centroid = mplane.o;
                    LOGE("Centroid is %f %f %f",mplane.o.at<float>(0,0),mplane.o.at<float>(1,0),mplane.o.at<float>(2,0));
                    instialized = true;
                    LOGE("Find  Plane");
                    Plane2World =tempTpw;
                }

            }else{
                cv::Mat Plane2Camera = pose*Plane2World;

                vector<cv::Point3f> drawPoints(8);
                drawPoints[0] = cv::Point3f(0,0,0);
                drawPoints[1] = cv::Point3f(0.3,0.0,0.0);
                drawPoints[2] = cv::Point3f(0.0,0,0.3);
                drawPoints[3] = cv::Point3f(0.0,0.3,0);
                drawPoints[4] =cv::Point3f(0,0.3,0.3);
                drawPoints[5] =cv::Point3f(0.3,0.3,0.3);
                drawPoints[6] =cv::Point3f(0.3,0,0.3);
                drawPoints[7] =cv::Point3f(0.3,0.3,0);

                for(int row = 0 ; row < 4;row++){
                    LOGE(" Plane2Camera %f %f %f %f",Plane2Camera.at<float>(row,0),Plane2Camera.at<float>(row,1),Plane2Camera.at<float>(row,2),Plane2Camera.at<float>(row,3));
                }
                cv::Mat Rcp ,Tcp;
                cv::Rodrigues(Plane2Camera.rowRange(0,3).colRange(0,3),Rcp);
                LOGE(" rwp %f %f %f",Rcp.at<float>(0,0),Rcp.at<float>(1,0),Rcp.at<float>(2,2));

                Tcp = Plane2Camera.col(3).rowRange(0,3);
                LOGE("Tcp %f %f %f",Tcp.at<float>(0,0),Tcp.at<float>(1,0),Tcp.at<float>(2,0));
                cv::projectPoints(drawPoints, Rcp, Tcp, SLAM->mpTracker->mK, SLAM->mpTracker->mDistCoef, projectedPoints);

            }




        }
    }

    switch(SLAM->GetTrackingState()) {
        case -1: {cv::putText(*pMat, "SYSTEM NOT READY", cv::Point(0,400), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,0),2); }break;
        case 0:  {cv::putText(*pMat, "NO IMAGES YET", cv::Point(0,400), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,0),2); }break;
        case 1:  {cv::putText(*pMat, "SLAM NOT INITIALIZED", cv::Point(0,400), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,0),2); }break;
        case 2:  {cv::putText(*pMat, "SLAM ON", cv::Point(0,400), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0),2); break;}
        case 3:  {cv::putText(*pMat, "SLAM LOST", cv::Point(0,400), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,0), 2); break;}
        default:break;
    }

    // Stop all threads
    //SLAM->Shutdown();
    // Save camera trajectory
    //SLAM->SaveKeyFrameTrajectoryTUM("/storage/emulated/0/SLAM/Trajectory/KeyFrameTrajectory.txt");

    cv::Mat ima=pose;
    jfloatArray resultArray = env->NewFloatArray(ima.rows * ima.cols);
    jfloat *resultPtr;

    resultPtr = env->GetFloatArrayElements(resultArray, 0);
    for (int i = 0; i < ima.rows; i++)
        for (int j = 0; j < ima.cols; j++) {
            float tempdata=ima.at<float>(i,j);
            resultPtr[i * ima.rows + j] =tempdata;
        }
    env->ReleaseFloatArrayElements(resultArray, resultPtr, 0);
   return resultArray;
#endif
}

