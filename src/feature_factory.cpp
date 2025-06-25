#include "feature_factory.h"
#include "orb_detector.h"
#include "direct_method_adapter.h"
#include <stdexcept>
#include <algorithm>

namespace fast_livo2 {

std::unique_ptr<AbstractFeatureDetector> 
FeatureFactory::createDetector(FeatureType type, const ros::NodeHandle& nh) {
    switch (type) {
        case FeatureType::ORB:
            return createORBDetector(nh);
        
        case FeatureType::SIFT:
            return createSIFTDetector(nh);
        
        case FeatureType::SURF:
            return createSURFDetector(nh);
        
        case FeatureType::AKAZE:
            return createAKAZEDetector(nh);
        
        case FeatureType::DIRECT:
            return std::make_unique<DirectMethodAdapter>();
        
        default:
            throw std::runtime_error("Unknown feature type requested");
    }
}

FeatureType FeatureFactory::stringToFeatureType(const std::string& type_str) {
    std::string upper_str = type_str;
    std::transform(upper_str.begin(), upper_str.end(), upper_str.begin(), ::toupper);
    
    if (upper_str == "ORB") {
        return FeatureType::ORB;
    } else if (upper_str == "SIFT") {
        return FeatureType::SIFT;
    } else if (upper_str == "SURF") {
        return FeatureType::SURF;
    } else if (upper_str == "AKAZE") {
        return FeatureType::AKAZE;
    } else if (upper_str == "DIRECT") {
        return FeatureType::DIRECT;
    } else {
        throw std::runtime_error("Unknown feature type: " + type_str);
    }
}

std::string FeatureFactory::featureTypeToString(FeatureType type) {
    switch (type) {
        case FeatureType::ORB:
            return "ORB";
        case FeatureType::SIFT:
            return "SIFT";
        case FeatureType::SURF:
            return "SURF";
        case FeatureType::AKAZE:
            return "AKAZE";
        case FeatureType::DIRECT:
            return "DIRECT";
        default:
            return "UNKNOWN";
    }
}

bool FeatureFactory::isFeatureTypeAvailable(FeatureType type) {
    switch (type) {
        case FeatureType::ORB:
        case FeatureType::AKAZE:
        case FeatureType::DIRECT:
            return true;  // Always available in OpenCV
        
        case FeatureType::SIFT:
        case FeatureType::SURF:
            // These might require opencv_contrib
            // TODO: Add proper checks for availability
            return false;
        
        default:
            return false;
    }
}

std::vector<std::string> FeatureFactory::getAvailableFeatureTypes() {
    std::vector<std::string> available;
    
    for (auto type : {FeatureType::ORB, FeatureType::SIFT, FeatureType::SURF, 
                      FeatureType::AKAZE, FeatureType::DIRECT}) {
        if (isFeatureTypeAvailable(type)) {
            available.push_back(featureTypeToString(type));
        }
    }
    
    return available;
}

std::unique_ptr<AbstractFeatureDetector> 
FeatureFactory::createORBDetector(const ros::NodeHandle& nh) {
    // Read ORB parameters from ROS
    int nfeatures = nh.param<int>("orb/nfeatures", 1000);
    float scale_factor = nh.param<float>("orb/scale_factor", 1.2f);
    int nlevels = nh.param<int>("orb/nlevels", 8);
    int edge_threshold = nh.param<int>("orb/edge_threshold", 31);
    int first_level = nh.param<int>("orb/first_level", 0);
    int WTA_K = nh.param<int>("orb/WTA_K", 2);
    int score_type = nh.param<int>("orb/score_type", cv::ORB::HARRIS_SCORE);
    int patch_size = nh.param<int>("orb/patch_size", 31);
    int fast_threshold = nh.param<int>("orb/fast_threshold", 20);
    
    return std::make_unique<ORBDetector>(
        nfeatures, scale_factor, nlevels, edge_threshold,
        first_level, WTA_K, static_cast<cv::ORB::ScoreType>(score_type),
        patch_size, fast_threshold
    );
}

std::unique_ptr<AbstractFeatureDetector> 
FeatureFactory::createSIFTDetector(const ros::NodeHandle& nh) {
    // TODO: Implement SIFT detector when available
    throw std::runtime_error("SIFT detector not yet implemented");
}

std::unique_ptr<AbstractFeatureDetector> 
FeatureFactory::createSURFDetector(const ros::NodeHandle& nh) {
    // TODO: Implement SURF detector when available
    throw std::runtime_error("SURF detector not yet implemented");
}

std::unique_ptr<AbstractFeatureDetector> 
FeatureFactory::createAKAZEDetector(const ros::NodeHandle& nh) {
    // TODO: Implement AKAZE detector
    throw std::runtime_error("AKAZE detector not yet implemented");
}

} // namespace fast_livo2 