#include <iostream>
#include "apriltag/apriltag_model.h"


int main(int argc, char *argv[])
{
    getopt_t *getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 1, "Reduce output");
    getopt_add_string(getopt, 'f', "family", "tag36h10", "Tag family to use");
    getopt_add_int(getopt, 't', "threads", "8", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "2.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");

    if (!getopt_parse(getopt, argc, argv, 1) ||
            getopt_get_bool(getopt, "help")) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }
    std::string famname = "tag36h10";
    Apriltag_model apriltag_model(famname);
    cv::Mat frame, resize_frame;
    // Initialize camera
    cv::VideoCapture cap("../test.mp4");
    cv::VideoWriter writer("../result.mp4", cv::VideoWriter::fourcc('m','p','4','v'), 30, cv::Size(cap.get(cv::CAP_PROP_FRAME_WIDTH), cap.get(cv::CAP_PROP_FRAME_HEIGHT)), true);
    if (!cap.isOpened()) {
        std::cerr << "Couldn't open video capture device" << std::endl;
        return -1;
    }
    for (int i =0; i<cap.get(cv::CAP_PROP_FRAME_COUNT); i++)
    {
        cap.read(frame);
        apriltag_model.run_frame(frame, 0.625, 3801.9172357035968, 3790.1487533583263, 1859.3812994964376, 1028.2183378519671,
                                 true);
        cv::resize(frame, resize_frame, cv::Size(0,0), 0.5, 0.5);
        cv::imshow("frame", resize_frame);
        writer.write(frame);
        cv::waitKey(1);
    }

    cap.release();
    writer.release();

    getopt_destroy(getopt);

    return 0;
}