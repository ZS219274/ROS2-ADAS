#ifndef PLANNING_YAML_HELPER_H
#define PLANNING_YAML_HELPER_H

#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>

namespace Planning {

/**
 * @brief A helper class for safely reading YAML values with default fallbacks
 */
class YamlHelper {
public:
    /**
     * @brief Get a value from YAML node with a default fallback
     * @tparam T Type of the value to get
     * @param node YAML node to read from
     * @param key Key to look for
     * @param default_value Default value if key is not found or conversion fails
     * @return Value from YAML or default value
     */
    template<typename T>
    static T get_value(const YAML::Node& node, const std::string& key, const T& default_value) {
        try {
            const YAML::Node& sub_node = node[key];
            if (sub_node) {
                return sub_node.as<T>();
            } else {
                RCLCPP_WARN(rclcpp::get_logger("config"), 
                           "Key '%s' not found, using default value", key.c_str());
                return default_value;
            }
        } catch (const YAML::Exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("config"), 
                       "Failed to parse key '%s': %s, using default value", 
                       key.c_str(), e.what());
            return default_value;
        }
    }

    /**
     * @brief Check if a key exists in the YAML node
     * @param node YAML node to check
     * @param key Key to look for
     * @return True if key exists, false otherwise
     */
    static bool has_key(const YAML::Node& node, const std::string& key) {
        try {
            return node[key].IsDefined();
        } catch (const YAML::Exception&) {
            return false;
        }
    }
};

} // namespace Planning

#endif // PLANNING_YAML_HELPER_H