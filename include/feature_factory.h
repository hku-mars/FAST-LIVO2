#ifndef FEATURE_FACTORY_H
#define FEATURE_FACTORY_H

#include <memory>
#include <string>
#include <ros/ros.h>
#include "feature_detector.h"

namespace fast_livo2 {

/**
 * @brief Enumeration of supported feature types
 */
enum class FeatureType {
    ORB,
    SIFT,
    SURF,
    AKAZE,
    DIRECT  // Original direct photometric method
};

/**
 * @brief Factory class for creating feature detectors
 * 
 * This factory creates appropriate feature detector instances
 * based on the requested type and ROS parameters.
 */
class FeatureFactory {
public:
    /**
     * @brief Create a feature detector of the specified type
     * @param type The type of feature detector to create
     * @param nh ROS node handle for reading parameters
     * @return Unique pointer to the created detector
     */
    static std::unique_ptr<AbstractFeatureDetector> 
    createDetector(FeatureType type, const ros::NodeHandle& nh);
    
    /**
     * @brief Convert string to FeatureType enum
     * @param type_str String representation of feature type
     * @return Corresponding FeatureType enum value
     * @throws std::runtime_error if string is not recognized
     */
    static FeatureType stringToFeatureType(const std::string& type_str);
    
    /**
     * @brief Convert FeatureType enum to string
     * @param type FeatureType enum value
     * @return String representation of the feature type
     */
    static std::string featureTypeToString(FeatureType type);
    
    /**
     * @brief Check if a feature type is available/supported
     * @param type The feature type to check
     * @return True if the feature type is available
     */
    static bool isFeatureTypeAvailable(FeatureType type);
    
    /**
     * @brief Get list of all available feature types
     * @return Vector of available feature type strings
     */
    static std::vector<std::string> getAvailableFeatureTypes();

private:
    // Private constructor to prevent instantiation
    FeatureFactory() = default;
    
    /**
     * @brief Create ORB detector with ROS parameters
     */
    static std::unique_ptr<AbstractFeatureDetector> 
    createORBDetector(const ros::NodeHandle& nh);
    
    /**
     * @brief Create SIFT detector with ROS parameters
     */
    static std::unique_ptr<AbstractFeatureDetector> 
    createSIFTDetector(const ros::NodeHandle& nh);
    
    /**
     * @brief Create SURF detector with ROS parameters
     */
    static std::unique_ptr<AbstractFeatureDetector> 
    createSURFDetector(const ros::NodeHandle& nh);
    
    /**
     * @brief Create AKAZE detector with ROS parameters
     */
    static std::unique_ptr<AbstractFeatureDetector> 
    createAKAZEDetector(const ros::NodeHandle& nh);
};

} // namespace fast_livo2

#endif // FEATURE_FACTORY_H 