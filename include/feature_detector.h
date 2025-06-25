#ifndef FEATURE_DETECTOR_H
#define FEATURE_DETECTOR_H

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <memory>
#include <vector>
#include <string>

namespace fast_livo2 {

/**
 * @brief Abstract base class for feature detectors
 * 
 * This class provides a unified interface for different feature detection
 * methods (ORB, SIFT, SURF, etc.) allowing runtime switching between them.
 */
class AbstractFeatureDetector {
public:
    virtual ~AbstractFeatureDetector() = default;
    
    /**
     * @brief Detect keypoints and compute descriptors
     * @param img Input image
     * @param keypoints Output detected keypoints
     * @param descriptors Output computed descriptors
     */
    virtual void detectAndCompute(const cv::Mat& img, 
                                  std::vector<cv::KeyPoint>& keypoints,
                                  cv::Mat& descriptors) = 0;
    
    /**
     * @brief Create a matcher suitable for this feature type
     * @return Pointer to the appropriate descriptor matcher
     */
    virtual cv::Ptr<cv::DescriptorMatcher> createMatcher() = 0;
    
    /**
     * @brief Get the feature type name
     * @return String identifier for the feature type
     */
    virtual std::string getFeatureType() const = 0;
    
    /**
     * @brief Get descriptor size in bytes
     * @return Size of a single descriptor
     */
    virtual int getDescriptorSize() const = 0;
    
    /**
     * @brief Get descriptor type (CV_8U for ORB, CV_32F for SIFT/SURF)
     * @return OpenCV type of the descriptor
     */
    virtual int getDescriptorType() const = 0;
    
    /**
     * @brief Check if descriptors are binary (for matching strategy)
     * @return True if binary descriptors (like ORB), false if floating point
     */
    virtual bool isBinaryDescriptor() const = 0;
    
    /**
     * @brief Get recommended matching threshold for this feature type
     * @return Distance threshold for matching
     */
    virtual double getMatchingThreshold() const = 0;
    
    /**
     * @brief Set detector parameters at runtime
     * @param param Parameter name
     * @param value Parameter value
     */
    virtual void setParameter(const std::string& param, double value) {}
};

} // namespace fast_livo2

#endif // FEATURE_DETECTOR_H 