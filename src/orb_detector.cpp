#include "orb_detector.h"
#include <opencv2/imgproc.hpp>

namespace fast_livo2 {

ORBDetector::ORBDetector(int nfeatures,
                         float scaleFactor,
                         int nlevels,
                         int edgeThreshold,
                         int firstLevel,
                         int WTA_K,
                         cv::ORB::ScoreType scoreType,
                         int patchSize,
                         int fastThreshold)
    : nfeatures_(nfeatures)
    , scale_factor_(scaleFactor)
    , nlevels_(nlevels)
    , edge_threshold_(edgeThreshold)
    , first_level_(firstLevel)
    , WTA_K_(WTA_K)
    , score_type_(scoreType)
    , patch_size_(patchSize)
    , fast_threshold_(fastThreshold) {
    
    recreateDetector();
}

void ORBDetector::recreateDetector() {
    detector_ = cv::ORB::create(
        nfeatures_,
        scale_factor_,
        nlevels_,
        edge_threshold_,
        first_level_,
        WTA_K_,
        score_type_,
        patch_size_,
        fast_threshold_
    );
}

void ORBDetector::detectAndCompute(const cv::Mat& img,
                                   std::vector<cv::KeyPoint>& keypoints,
                                   cv::Mat& descriptors) {
    if (img.empty()) {
        keypoints.clear();
        descriptors = cv::Mat();
        return;
    }
    
    // Convert to grayscale if needed
    cv::Mat gray;
    if (img.channels() == 3) {
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    } else if (img.channels() == 4) {
        cv::cvtColor(img, gray, cv::COLOR_BGRA2GRAY);
    } else {
        gray = img;
    }
    
    // Detect and compute
    detector_->detectAndCompute(gray, cv::noArray(), keypoints, descriptors);
}

cv::Ptr<cv::DescriptorMatcher> ORBDetector::createMatcher() {
    // Use Brute Force matcher with Hamming distance for binary descriptors
    return cv::BFMatcher::create(cv::NORM_HAMMING, true); // crossCheck = true
}

void ORBDetector::setParameter(const std::string& param, double value) {
    bool needs_recreate = false;
    
    if (param == "nfeatures") {
        nfeatures_ = static_cast<int>(value);
        needs_recreate = true;
    } else if (param == "scale_factor") {
        scale_factor_ = static_cast<float>(value);
        needs_recreate = true;
    } else if (param == "nlevels") {
        nlevels_ = static_cast<int>(value);
        needs_recreate = true;
    } else if (param == "edge_threshold") {
        edge_threshold_ = static_cast<int>(value);
        needs_recreate = true;
    } else if (param == "fast_threshold") {
        fast_threshold_ = static_cast<int>(value);
        needs_recreate = true;
    }
    
    if (needs_recreate) {
        recreateDetector();
    }
}

void ORBDetector::setNFeatures(int nfeatures) {
    nfeatures_ = nfeatures;
    recreateDetector();
}

void ORBDetector::setScaleFactor(float scaleFactor) {
    scale_factor_ = scaleFactor;
    recreateDetector();
}

void ORBDetector::setNLevels(int nlevels) {
    nlevels_ = nlevels;
    recreateDetector();
}

void ORBDetector::setFastThreshold(int threshold) {
    fast_threshold_ = threshold;
    recreateDetector();
}

} // namespace fast_livo2 