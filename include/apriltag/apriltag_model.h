//
// Created by leo on 2022/1/12.
//

#ifndef APRILTAG_MODEL_APRILTAG_MODEL_H
#define APRILTAG_MODEL_APRILTAG_MODEL_H

#include "opencv2/opencv.hpp"
#include "iostream"
extern "C" {
#include "apriltag/apriltag_pose.h"
#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/tag36h10.h"
#include "apriltag/tag25h9.h"
#include "apriltag/tag16h5.h"
#include "apriltag/tagCircle21h7.h"
#include "apriltag/tagCircle49h12.h"
#include "apriltag/tagCustom48h12.h"
#include "apriltag/tagStandard41h12.h"
#include "apriltag/tagStandard52h13.h"
#include "apriltag/common/getopt.h"
}

class Apriltag_model
{
private:
    apriltag_detector_t *td;
    apriltag_family_t *tf = NULL;
    std::string famname;
    cv::Mat intrisicMat; // Intrisic matrix
    cv::Mat distCoeffs;   // Distortion vector
    std::vector<cv::Point3f> objectPoints;
public:
    Apriltag_model(std::string famname, double decimate=1.0, double blur=0.0, int threads_num = 4, bool refine_edges=true);
    ~Apriltag_model();
    void run_frame(cv::Mat frame, double tag_size,double fx, double fy, double cx, double cy, bool pose_mea);

};



#endif //APRILTAG_MODEL_APRILTAG_MODEL_H
