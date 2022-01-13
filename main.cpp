#include <iostream>
#include "apriltag/apriltag_model.h"


int main(int argc, char *argv[])
{
    double tag_size = 0.625;
    double fx = 3801.9172357035968;
    double fy = 3790.1487533583263;
    double cx = 1859.3812994964376;
    double cy = 1028.2183378519671;
    std::string famname = "tag36h10";
    Apriltag_model apriltag_model(famname);
    cv::Mat frame, resize_frame;

    cv::VideoCapture cap("../video2.mp4");
    cv::VideoWriter writer("../result.mp4", cv::VideoWriter::fourcc('m','p','4','v'), 30, cv::Size(cap.get(cv::CAP_PROP_FRAME_WIDTH), cap.get(cv::CAP_PROP_FRAME_HEIGHT)), true);

    for (int i =0; i<cap.get(cv::CAP_PROP_FRAME_COUNT); i++)
    {
        cap.read(frame);
        apriltag_model.run_frame(frame, tag_size, fx, fy, cx, cy,true);
        cv::resize(frame, resize_frame, cv::Size(0,0), 0.5, 0.5);
        cv::imshow("frame", resize_frame);
        writer.write(frame);
        cv::waitKey(1);
    }

    cap.release();
    writer.release();
    return 0;
}