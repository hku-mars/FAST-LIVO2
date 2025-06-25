#ifndef ORB_DETECTOR_H
#define ORB_DETECTOR_H

#include "feature_detector.h"
#include <opencv2/features2d.hpp>

namespace fast_livo2 {

/**
 * @brief ORB feature detector implementation
 * 
 * Implements the AbstractFeatureDetector interface using ORB
 * (Oriented FAST and Rotated BRIEF) features.
 */
class ORBDetector : public AbstractFeatureDetector {
private:
    cv::Ptr<cv::ORB> detector_;
    int nfeatures_;
    float scale_factor_;
    int nlevels_;
    int edge_threshold_;
    int first_level_;
    int WTA_K_;
    cv::ORB::ScoreType score_type_;
    int patch_size_;
    int fast_threshold_;
    
public:
    /**
     * @brief Constructor with ORB parameters
     */
    ORBDetector(int nfeatures = 1000,
                float scaleFactor = 1.2f,
                int nlevels = 8,
                int edgeThreshold = 31,
                int firstLevel = 0,
                int WTA_K = 2,
                cv::ORB::ScoreType scoreType = cv::ORB::HARRIS_SCORE,
                int patchSize = 31,
                int fastThreshold = 20);
    
    // Implement abstract interface
    void detectAndCompute(const cv::Mat& img, 
                         std::vector<cv::KeyPoint>& keypoints,
                         cv::Mat& descriptors) override;
    
    cv::Ptr<cv::DescriptorMatcher> createMatcher() override;
    
    std::string getFeatureType() const override { return "ORB"; }
    
    int getDescriptorSize() const override { return 32; } // 32 bytes for ORB
    
    int getDescriptorType() const override { return CV_8U; }
    
    bool isBinaryDescriptor() const override { return true; }
    
    double getMatchingThreshold() const override { return 50.0; } // Hamming distance
    
    void setParameter(const std::string& param, double value) override;
    
    // ORB-specific methods
    void setNFeatures(int nfeatures);
    void setScaleFactor(float scaleFactor);
    void setNLevels(int nlevels);
    void setFastThreshold(int threshold);
    
private:
    void recreateDetector();
};

} // namespace fast_livo2

#endif // ORB_DETECTOR_H 