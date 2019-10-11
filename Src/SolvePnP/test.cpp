#include "PnpSolver.h"

const char* keys  =
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }";

int main(int argc, char *argv[]) {
    CommandLineParser parser(argc, argv, keys);
    int cam_id = parser.get<int>("ci");

    VideoCapture cp;
    cp.open(cam_id);

    PnpSolver test_solver(5, 7, 29, 19.5, 0, "../Calib/InParams/in.param");

    while (cp.grab()) {
        cv::Mat frame;
        cp.retrieve(frame);
        cv::Mat R, t;
        test_solver.Solve(frame, R, t);
    }
    return 0;
}
