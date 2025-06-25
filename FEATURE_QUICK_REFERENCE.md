# FAST-LIVO2 Feature Detector Quick Reference

## Checklist for Adding a New Feature Detector

- [ ] Create header file inheriting from `AbstractFeatureDetector`
- [ ] Implement the 4 required virtual methods
- [ ] Add enum value to `FeatureType` in `feature_factory.h`
- [ ] Add case to `FeatureFactory::createDetector()`
- [ ] Add string mapping to `stringToFeatureType()`
- [ ] Add source file to CMakeLists.txt
- [ ] Add ROS parameters to launch file
- [ ] Test with sample data

## Required Methods to Implement

```cpp
class YourDetector : public AbstractFeatureDetector {
public:
    // 1. Detect keypoints and compute descriptors
    void detectAndCompute(const cv::Mat& img, 
                         std::vector<cv::KeyPoint>& keypoints,
                         cv::Mat& descriptors) override;
    
    // 2. Create appropriate matcher for your descriptor type
    cv::Ptr<cv::DescriptorMatcher> createMatcher() override;
    
    // 3. Return feature type name (used for logging)
    std::string getFeatureType() const override;
    
    // 4. Return descriptor dimension
    int getDescriptorSize() const override;
};
```

## Matcher Types by Descriptor

| Descriptor Type | Matcher | Norm Type |
|----------------|---------|-----------|
| Binary (ORB, AKAZE) | BFMatcher | cv::NORM_HAMMING |
| Float (SIFT, SURF) | BFMatcher | cv::NORM_L2 |
| Deep Features | BFMatcher or Custom | cv::NORM_L2 |

## Common Parameters

```yaml
# In launch file or config
your_detector:
  max_features: 1000      # Maximum keypoints to detect
  threshold: 0.01         # Detection threshold
  scale_factor: 1.2       # Pyramid scale factor
  nlevels: 8              # Pyramid levels
  
  # Matching parameters
  matching:
    distance_threshold: 50.0  # For binary: 0-100, For float: 0-1
    ratio_test: 0.8          # Lowe's ratio test threshold
    cross_check: true        # Bidirectional matching
```

## Memory Management

```cpp
// ALWAYS use shared_ptr for Features
auto ftr = std::make_shared<Feature>(...);

// NEVER use raw pointers
Feature* ftr = new Feature(...);  // DON'T DO THIS!
```

## Testing Commands

```bash
# Quick test with your detector
roslaunch fast_livo mapping_avia.launch feature_type:=YOUR_DETECTOR

# Compare performance
rosrun fast_livo test_feature_detector YOUR_DETECTOR test_image.jpg

# Benchmark detection time
rosrun fast_livo benchmark_features --detector YOUR_DETECTOR --dataset /path/to/images
```

## Debug Outputs

Add these to your detector for debugging:

```cpp
void YourDetector::detectAndCompute(...) {
    // ... detection code ...
    
    ROS_INFO("[%s] Detected %zu keypoints in %.3f ms", 
             getFeatureType().c_str(), 
             keypoints.size(), 
             detection_time_ms);
    
    // Optional: Save debug image
    if (debug_mode_) {
        cv::Mat vis;
        cv::drawKeypoints(img, keypoints, vis);
        cv::imwrite("debug_" + getFeatureType() + ".jpg", vis);
    }
}
```

## Common Pitfalls

1. **Wrong descriptor type in Feature struct**
   ```cpp
   // Make sure to set the correct type
   ftr->feature_type_ = getFeatureType();
   ```

2. **Forgetting to clone descriptors**
   ```cpp
   // Clone when storing to avoid dangling references
   stored_descriptor = descriptor.clone();
   ```

3. **Not handling empty detection**
   ```cpp
   if (keypoints.empty()) {
       ROS_WARN("No features detected!");
       return;
   }
   ```

4. **Mismatched descriptor format**
   ```cpp
   // Ensure correct type (CV_8U for binary, CV_32F for float)
   CV_Assert(descriptors.type() == CV_32F || 
             descriptors.type() == CV_8U);
   ```

## Performance Tips

- Pre-allocate vectors when possible
- Use OpenCV's parallel_for_ for large images
- Implement pyramid detection efficiently
- Cache commonly used computations
- Profile with `cv::TickMeter`

## Example Minimal Implementation

```cpp
// minimal_detector.cpp
#include "minimal_detector.h"

MinimalDetector::MinimalDetector(const ros::NodeHandle& nh) {
    nh.param<int>("minimal/max_features", max_features_, 500);
    detector_ = cv::ORB::create(max_features_);
}

void MinimalDetector::detectAndCompute(const cv::Mat& img, 
                                      std::vector<cv::KeyPoint>& keypoints,
                                      cv::Mat& descriptors) {
    detector_->detectAndCompute(img, cv::noArray(), keypoints, descriptors);
}

cv::Ptr<cv::DescriptorMatcher> MinimalDetector::createMatcher() {
    return cv::BFMatcher::create(cv::NORM_HAMMING, true);
}

std::string MinimalDetector::getFeatureType() const { 
    return "MINIMAL"; 
}

int MinimalDetector::getDescriptorSize() const { 
    return 32; // ORB descriptor size
}
```

That's it! Your custom feature detector is ready to use in FAST-LIVO2. 