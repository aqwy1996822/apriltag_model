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
            info.tagsize = 0.0624;
            info.fx = 3801.9172357035968;
            info.fy = 3790.1487533583263;
            info.cx = 1859.3812994964376;
            info.cy = 1028.2183378519671;

// Then call estimate_tag_pose.
            apriltag_pose_t pose;
            double err = estimate_tag_pose(&info, &pose);
            std::cout << pose.R->data[0] <<" "<< pose.R->data[1] <<" "<< pose.R->data[2] <<" "<< pose.R->data[3] <<" "<< std::endl;
        }
    }
    apriltag_detections_destroy(detections);
}