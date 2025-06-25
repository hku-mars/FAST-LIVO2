# Adding Custom Visual Keypoint Features to FAST-LIVO2

This guide explains how to integrate your own custom visual feature detectors into the FAST-LIVO2 hybrid visual-inertial-LiDAR odometry system.

## Table of Contents
1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Step-by-Step Implementation](#step-by-step-implementation)
4. [Example: Adding SuperPoint](#example-adding-superpoint)
5. [Configuration](#configuration)
6. [Testing](#testing)
7. [Troubleshooting](#troubleshooting)

## Overview

FAST-LIVO2 has been enhanced with a modular feature detection system that allows you to easily add custom keypoint detectors. The system supports:
- ORB (default)
- SIFT
- SURF
- AKAZE
- Custom detectors (your implementation)

The architecture uses an abstract interface pattern, making it straightforward to add new feature types without modifying the core tracking logic.

## Architecture

The feature detection system consists of:

```
AbstractFeatureDetector (interface)
    ├── ORBDetector
    ├── SIFTDetector  
    ├── SURFDetector
    ├── AKAZEDetector
    └── YourCustomDetector
```

Key components:
- **AbstractFeatureDetector**: Base interface defining required methods
- **FeatureFactory**: Creates detector instances based on configuration
- **Feature struct**: Stores keypoint data with flexible descriptor support

## Step-by-Step Implementation

### 1. Create Your Detector Header

Create a new header file in `src/FAST-LIVO2/include/`:

```cpp
// your_detector.h
#ifndef YOUR_DETECTOR_H
#define YOUR_DETECTOR_H

#include "feature_detector.h"
#include <opencv2/opencv.hpp>

class YourDetector : public AbstractFeatureDetector {
private:
    // Your detector-specific members
    cv::Ptr<cv::Feature2D> detector_;
    int max_features_;
    float threshold_;
    
public:
    YourDetector(const ros::NodeHandle& nh);
    ~YourDetector() override = default;
    
    void detectAndCompute(const cv::Mat& img, 
                         std::vector<cv::KeyPoint>& keypoints,
                         cv::Mat& descriptors) override;
    
    cv::Ptr<cv::DescriptorMatcher> createMatcher() override;
    
    std::string getFeatureType() const override { return "YOUR_DETECTOR"; }
    
    int getDescriptorSize() const override;
};

#endif
```

### 2. Implement Your Detector

Create the implementation in `src/FAST-LIVO2/src/`:

```cpp
// your_detector.cpp
#include "your_detector.h"

YourDetector::YourDetector(const ros::NodeHandle& nh) {
    // Read parameters from ROS
    nh.param<int>("your_detector/max_features", max_features_, 1000);
    nh.param<float>("your_detector/threshold", threshold_, 0.01f);
    
    // Initialize your detector
    // Example: detector_ = cv::YourFeature2D::create(max_features_, threshold_);
}

void YourDetector::detectAndCompute(const cv::Mat& img, 
                                   std::vector<cv::KeyPoint>& keypoints,
                                   cv::Mat& descriptors) {
    // Implement your detection and description extraction
    // Example:
    // detector_->detectAndCompute(img, cv::noArray(), keypoints, descriptors);
    
    // Or custom implementation:
    // 1. Detect keypoints
    // 2. Extract descriptors
    // 3. Fill keypoints and descriptors
}

cv::Ptr<cv::DescriptorMatcher> YourDetector::createMatcher() {
    // Return appropriate matcher for your descriptor type
    
    // For binary descriptors (like ORB):
    // return cv::BFMatcher::create(cv::NORM_HAMMING, true);
    
    // For float descriptors (like SIFT):
    // return cv::BFMatcher::create(cv::NORM_L2, true);
    
    // For learned descriptors, you might need custom matching
}

int YourDetector::getDescriptorSize() const {
    // Return the size of your descriptor
    // e.g., 128 for SIFT, 256 for SuperPoint, etc.
    return 128;
}
```

### 3. Add to Feature Factory

Update `src/FAST-LIVO2/include/feature_factory.h`:

```cpp
enum class FeatureType {
    ORB,
    SIFT,
    SURF,
    AKAZE,
    YOUR_DETECTOR,  // Add your type
    DIRECT
};
```

Update `src/FAST-LIVO2/src/feature_factory.cpp`:

```cpp
#include "your_detector.h"  // Add include

std::unique_ptr<AbstractFeatureDetector> 
FeatureFactory::createDetector(FeatureType type, const ros::NodeHandle& nh) {
    switch(type) {
        // ... existing cases ...
        
        case FeatureType::YOUR_DETECTOR:
            return std::make_unique<YourDetector>(nh);
            
        default:
            ROS_ERROR("Unknown feature type!");
            return nullptr;
    }
}
```

Also add to the string conversion function:
```cpp
FeatureType stringToFeatureType(const std::string& type_str) {
    // ... existing mappings ...
    if (type_str == "YOUR_DETECTOR") return FeatureType::YOUR_DETECTOR;
    // ...
}
```

### 4. Update CMakeLists.txt

Add your source file to `src/FAST-LIVO2/CMakeLists.txt`:

```cmake
add_library(vio
    src/vio.cpp
    src/visual_point.cpp
    src/frame.cpp
    src/feature_factory.cpp
    src/orb_detector.cpp
    src/your_detector.cpp  # Add this line
)
```

## Example: Adding SuperPoint

Here's a concrete example of adding the SuperPoint deep learning feature detector:

### superpoint_detector.h
```cpp
#ifndef SUPERPOINT_DETECTOR_H
#define SUPERPOINT_DETECTOR_H

#include "feature_detector.h"
#include <torch/script.h>

class SuperPointDetector : public AbstractFeatureDetector {
private:
    torch::jit::script::Module model_;
    float conf_threshold_;
    int nms_radius_;
    
public:
    SuperPointDetector(const ros::NodeHandle& nh);
    
    void detectAndCompute(const cv::Mat& img, 
                         std::vector<cv::KeyPoint>& keypoints,
                         cv::Mat& descriptors) override;
    
    cv::Ptr<cv::DescriptorMatcher> createMatcher() override {
        // SuperPoint uses L2 distance for matching
        return cv::BFMatcher::create(cv::NORM_L2, true);
    }
    
    std::string getFeatureType() const override { return "SUPERPOINT"; }
    
    int getDescriptorSize() const override { return 256; }
    
private:
    torch::Tensor nms(const torch::Tensor& scores, int radius);
};

#endif
```

### superpoint_detector.cpp (key parts)
```cpp
SuperPointDetector::SuperPointDetector(const ros::NodeHandle& nh) {
    std::string model_path;
    nh.param<std::string>("superpoint/model_path", model_path, "");
    nh.param<float>("superpoint/conf_threshold", conf_threshold_, 0.015f);
    nh.param<int>("superpoint/nms_radius", nms_radius_, 4);
    
    // Load the TorchScript model
    try {
        model_ = torch::jit::load(model_path);
        model_.eval();
    } catch (const c10::Error& e) {
        ROS_ERROR("Error loading SuperPoint model: %s", e.what());
    }
}

void SuperPointDetector::detectAndCompute(const cv::Mat& img, 
                                         std::vector<cv::KeyPoint>& keypoints,
                                         cv::Mat& descriptors) {
    // Convert image to tensor
    cv::Mat img_float;
    img.convertTo(img_float, CV_32F, 1.0/255.0);
    
    auto tensor = torch::from_blob(img_float.data, 
                                  {1, 1, img.rows, img.cols}, 
                                  torch::kFloat32).clone();
    
    // Run inference
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(tensor);
    
    auto output = model_.forward(inputs).toTuple();
    auto scores = output->elements()[0].toTensor();
    auto desc = output->elements()[1].toTensor();
    
    // Apply NMS and extract keypoints
    auto nms_scores = nms(scores, nms_radius_);
    
    // Convert to OpenCV format
    // ... (implementation details)
}
```

## Configuration

### Launch File Parameters

Add parameters to your launch file (e.g., `mapping_avia.launch`):

```xml
<!-- Feature Detection Parameters -->
<param name="laserMapping/feature_type" type="string" value="YOUR_DETECTOR"/>
<param name="laserMapping/use_hybrid" type="bool" value="true"/>

<!-- Your Detector Specific Parameters -->
<param name="laserMapping/your_detector/max_features" type="int" value="1000"/>
<param name="laserMapping/your_detector/threshold" type="double" value="0.01"/>

<!-- For learned features like SuperPoint -->
<param name="laserMapping/superpoint/model_path" type="string" 
       value="$(find fast_livo)/models/superpoint_v1.pt"/>
<param name="laserMapping/superpoint/conf_threshold" type="double" value="0.015"/>
```

### Config File Structure

Create a config file `config/your_detector_config.yaml`:

```yaml
# Your Detector Configuration
your_detector:
  max_features: 1000
  threshold: 0.01
  scale_factor: 1.2
  nlevels: 8
  
  # Matching parameters
  matching:
    ratio_test: 0.8
    cross_check: true
    distance_threshold: 50.0
```

## Testing

### 1. Unit Test

Create a test file `test/test_your_detector.cpp`:

```cpp
#include <gtest/gtest.h>
#include "your_detector.h"

TEST(YourDetectorTest, BasicDetection) {
    ros::NodeHandle nh;
    YourDetector detector(nh);
    
    // Load test image
    cv::Mat img = cv::imread("test_image.jpg", cv::IMREAD_GRAYSCALE);
    
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    
    detector.detectAndCompute(img, keypoints, descriptors);
    
    // Verify detection
    EXPECT_GT(keypoints.size(), 0);
    EXPECT_EQ(descriptors.rows, keypoints.size());
    EXPECT_EQ(descriptors.cols, detector.getDescriptorSize());
}
```

### 2. Integration Test

Test with FAST-LIVO2:

```bash
# Launch with your detector
roslaunch fast_livo mapping_avia.launch feature_type:=YOUR_DETECTOR

# Play test bag
rosbag play your_test.bag
```

### 3. Performance Comparison

Compare against other detectors:

```bash
# Create a script to test different detectors
for detector in ORB SIFT YOUR_DETECTOR; do
    roslaunch fast_livo mapping_avia.launch feature_type:=$detector
    # Record metrics
done
```

## Troubleshooting

### Common Issues

1. **Descriptor Type Mismatch**
   - Ensure your matcher type matches descriptor type (Hamming for binary, L2 for float)
   
2. **Memory Issues**
   - Use `std::shared_ptr<Feature>` to avoid memory leaks
   - Limit maximum features to prevent excessive memory usage

3. **Performance Problems**
   - Profile your detector with `cv::TickMeter`
   - Consider GPU acceleration for deep learning models
   - Implement keypoint response for better selection

4. **Integration Errors**
   ```cpp
   // Wrong - raw pointer
   Feature* ftr = new Feature(...);
   
   // Correct - shared pointer
   auto ftr = std::make_shared<Feature>(...);
   ```

### Debug Tips

1. **Enable Debug Output**
   ```cpp
   ROS_INFO("Detected %zu keypoints with %s", 
            keypoints.size(), getFeatureType().c_str());
   ```

2. **Visualize Results**
   ```cpp
   cv::Mat vis;
   cv::drawKeypoints(img, keypoints, vis);
   cv::imshow("Detections", vis);
   ```

3. **Check Descriptor Validity**
   ```cpp
   CV_Assert(!descriptors.empty());
   CV_Assert(descriptors.type() == CV_32F || 
             descriptors.type() == CV_8U);
   ```

## Best Practices

1. **Parameter Tuning**
   - Start with conservative settings
   - Gradually increase feature count
   - Balance between quality and speed

2. **Descriptor Design**
   - Keep descriptors compact (128-256 dims)
   - Ensure rotation invariance if needed
   - Consider illumination robustness

3. **Integration Guidelines**
   - Follow the existing code style
   - Add comprehensive error handling
   - Document your parameters
   - Include example configurations

4. **Performance Optimization**
   - Use OpenCV's optimized functions
   - Implement multi-scale detection efficiently
   - Consider SIMD optimizations
   - Profile before optimizing

## Conclusion

Adding custom features to FAST-LIVO2 is straightforward thanks to the modular architecture. Follow this guide, and you'll have your custom detector integrated in no time. Remember to test thoroughly and optimize for your specific use case.

For questions or contributions, please refer to the main FAST-LIVO2 repository. 