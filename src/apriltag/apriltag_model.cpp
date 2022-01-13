//
// Created by leo on 2022/1/12.
//
#include "apriltag/apriltag_model.h"

Apriltag_model::Apriltag_model(std::string famname, double decimate, double blur, int threads_num, bool refine_edges) {
    this->famname=famname;
    if (famname=="tag36h11") {
        tf = tag36h11_create();
    } else if (famname=="tag36h10") {
        tf = tag36h10_create();
    } else if (famname=="tag25h9") {
        tf = tag25h9_create();
    } else if (famname=="tag16h5") {
        tf = tag16h5_create();
    } else if (famname=="tagCircle21h7") {
        tf = tagCircle21h7_create();
    } else if (famname=="tagCircle49h12") {
        tf = tagCircle49h12_create();
    } else if (famname=="tagStandard41h12") {
        tf = tagStandard41h12_create();
    } else if (famname=="tagStandard52h13") {
        tf = tagStandard52h13_create();
    } else if (famname=="tagCustom48h12") {
        tf = tagCustom48h12_create();
    } else {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        exit(-1);
    }

    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = decimate;
    td->quad_sigma = blur;
    td->nthreads = threads_num;
    td->debug = 0;
    td->refine_edges = refine_edges;


    cv::Mat intrisicMat(3, 3, cv::DataType<float>::type); // Intrisic matrix
    intrisicMat.at<float>(0, 0) = 3801.9172357035968;
    intrisicMat.at<float>(1, 0) = 0;
    intrisicMat.at<float>(2, 0) = 0;

    intrisicMat.at<float>(0, 1) = 0;
    intrisicMat.at<float>(1, 1) = 3790.1487533583263;
    intrisicMat.at<float>(2, 1) = 0;

    intrisicMat.at<float>(0, 2) = 1859.3812994964376;
    intrisicMat.at<float>(1, 2) = 1028.2183378519671;
    intrisicMat.at<float>(2, 2) = 1;
    this->intrisicMat=intrisicMat;

    cv::Mat distCoeffs(5, 1, cv::DataType<float>::type);   // Distortion vector
//    distCoeffs.at<float>(0) = 0.01357555;
//    distCoeffs.at<float>(1) = 0.25292517;
//    distCoeffs.at<float>(2) = 0.00191167;
//    distCoeffs.at<float>(3) = -0.01010199;
//    distCoeffs.at<float>(4) = -1.62974334;
    distCoeffs.at<float>(0) = 0;
    distCoeffs.at<float>(1) = 0;
    distCoeffs.at<float>(2) = 0;
    distCoeffs.at<float>(3) = 0;
    distCoeffs.at<float>(4) = 0;
    this->distCoeffs=distCoeffs;


}

Apriltag_model::~Apriltag_model() {
    if (famname == "tag36h11") {
        tag36h11_destroy(tf);
    } else if (famname == "tag36h10") {
        tag36h10_destroy(tf);
    } else if (famname == "tag25h9") {
        tag25h9_destroy(tf);
    } else if (famname == "tag16h5") {
        tag16h5_destroy(tf);
    } else if (famname == "tagCircle21h7") {
        tagCircle21h7_destroy(tf);
    } else if (famname == "tagCircle49h12") {
        tagCircle49h12_destroy(tf);
    } else if (famname == "tagStandard41h12") {
        tagStandard41h12_destroy(tf);
    } else if (famname == "tagStandard52h13") {
        tagStandard52h13_destroy(tf);
    } else if (famname == "tagCustom48h12") {
        tagCustom48h12_destroy(tf);
    }
}
void Apriltag_model::run_frame(cv::Mat frame, double tag_size, double fx, double fy, double cx, double cy, bool pose_mea=true) {
    cv::Mat gray;
    cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // Make an image_u8_t header for the Mat data
    image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data};

    zarray_t *detections = apriltag_detector_detect(td, &im);

    objectPoints={cv::Point3f(-tag_size/2, -tag_size/2, 0),
                  cv::Point3f(-tag_size/2, tag_size/2, 0),
                  cv::Point3f(tag_size/2, tag_size/2, 0),
                  cv::Point3f(tag_size/2, -tag_size/2, 0),
                  cv::Point3f(-tag_size/2, -tag_size/2, -tag_size),
                  cv::Point3f(-tag_size/2, tag_size/2, -tag_size),
                  cv::Point3f(tag_size/2, tag_size/2, -tag_size),
                  cv::Point3f(tag_size/2, -tag_size/2, -tag_size)};

    // Draw detection outlines
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);


        line(frame, cv::Point(det->p[0][0], det->p[0][1]),
             cv::Point(det->p[1][0], det->p[1][1]),
             cv::Scalar(0, 0xff, 0), 4);
        line(frame, cv::Point(det->p[0][0], det->p[0][1]),
             cv::Point(det->p[3][0], det->p[3][1]),
             cv::Scalar(0, 0, 0xff), 4);
        line(frame, cv::Point(det->p[1][0], det->p[1][1]),
             cv::Point(det->p[2][0], det->p[2][1]),
             cv::Scalar(0xff, 0, 0), 4);
        line(frame, cv::Point(det->p[2][0], det->p[2][1]),
             cv::Point(det->p[3][0], det->p[3][1]),
             cv::Scalar(0xff, 0, 0), 4);

        std::stringstream ss;
        ss << det->id;

        cv::String text = ss.str();
        int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
        double fontscale = 4.0;
        int baseline;
        cv::Size textsize = getTextSize(text, fontface, fontscale, 2,
                                        &baseline);
        putText(frame, text, cv::Point(det->c[0] - textsize.width / 2,
                                       det->c[1] + textsize.height / 2),
                fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);


        if (pose_mea)
        {
            apriltag_detection_info_t info;
            info.det = det;
            info.tagsize = tag_size;
            info.fx = fx;
            info.fy = fy;
            info.cx = cx;
            info.cy = cy;

// Then call estimate_tag_pose.
            apriltag_pose_t pose;
            double err = estimate_tag_pose(&info, &pose);

            cv::Mat rotation_matrix = (cv::Mat_<double>(3, 3) << pose.R->data[0], pose.R->data[1], pose.R->data[2], pose.R->data[3], pose.R->data[4], pose.R->data[5], pose.R->data[6], pose.R->data[7], pose.R->data[8]);
            cv::Mat rVec;
            cv::Rodrigues(rotation_matrix, rVec);

            cv::Mat tVec(3, 1, cv::DataType<float>::type); // Translation vector
            tVec.at<float>(0) = (float)pose.t->data[0];
            tVec.at<float>(1) = (float)pose.t->data[1];
            tVec.at<float>(2) = (float)pose.t->data[2];


            std::vector<cv::Point2f> projectedPoints;
            cv::projectPoints(objectPoints, rVec, tVec, intrisicMat, distCoeffs, projectedPoints);

            for (int i =0;i<4;i++)
            {
                cv::line(frame, projectedPoints[i], projectedPoints[i+4], cv::Scalar(255 ,0,0), 3);
            }
            std::vector<std::vector<cv::Point>> draw_bottom={{projectedPoints[0],projectedPoints[1],projectedPoints[2], projectedPoints[3]}};
            std::vector<std::vector<cv::Point>> draw_top={{projectedPoints[4],projectedPoints[5],projectedPoints[6], projectedPoints[7]}};

            cv::drawContours(frame, draw_bottom, -1, cv::Scalar(0, 0, 255), 3);

            cv::drawContours(frame, draw_top, -1, cv::Scalar(255, 192,0), -1);

        }
    }
    apriltag_detections_destroy(detections);
}