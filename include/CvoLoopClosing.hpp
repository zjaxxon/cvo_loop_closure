#pragma once

#include <vector>
#include <DBoW3/DBoW3.h>
#include "KeyFrame.hpp"
#include <UnifiedCvo-0.1/cvo/CvoGPU.hpp>
#include <UnifiedCvo-0.1/utils/Calibration.hpp>


namespace cvo {
class CvoLoopClosing {
public:
    CvoLoopClosing(DBoW3::Database* pDB, int numFrame);

    bool detect_loop(const cv::Mat& kf, int id);

    void print_loop();

    // bool print_loop(unsigned int id);

private:
    DBoW3::Database* db;
    int numFrame_;
    int frameGap_; // We consider frames within this range as too close
    int numDetect_;
    std::unordered_map<int, std::vector<int>> loopsDetect;
};
} //namespace cvo