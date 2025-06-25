#ifndef DIRECT_METHOD_ADAPTER_H
#define DIRECT_METHOD_ADAPTER_H

#include "feature_detector.h"

namespace fast_livo2 {

/**
 * @brief Adapter for the original direct photometric method
 * 
 * This adapter allows the original direct method to work within
 * the new modular feature detector framework. It doesn't actually
 * detect features but provides compatibility.
 */
class DirectMethodAdapter : public AbstractFeatureDetector {
public:
    DirectMethodAdapter() = default;
    
    // Implement abstract interface
    void detectAndCompute(const cv::Mat& img, 
                         std::vector<cv::KeyPoint>& keypoints,
                         cv::Mat& descriptors) override {
        // Direct method doesn't use traditional keypoints/descriptors
        // This is just a placeholder for compatibility
        keypoints.clear();
        descriptors = cv::Mat();
    }
    
    cv::Ptr<cv::DescriptorMatcher> createMatcher() override {
        // Direct method doesn't use descriptor matching
        return cv::Ptr<cv::DescriptorMatcher>();
    }
    
    std::string getFeatureType() const override { return "DIRECT"; }
    
    int getDescriptorSize() const override { return 0; } // No descriptors
    
    int getDescriptorType() const override { return CV_32F; } // Placeholder
    
    bool isBinaryDescriptor() const override { return false; }
    
    double getMatchingThreshold() const override { return 0.0; } // Not used
    
    void setParameter(const std::string& param, double value) override {
        // Direct method parameters are handled elsewhere
    }
};

} // namespace fast_livo2

#endif // DIRECT_METHOD_ADAPTER_H 