#ifndef PHOTOGRAMMETRY_CAMERA_TYPE_HPP
#define PHOTOGRAMMETRY_CAMERA_TYPE_HPP

#include "utils/know_enum.hpp"

namespace photogrammetry {
namespace camera {
enum class kCameraType {
  CameraSimplePinholeModel,
  CameraSimpleRadialDistortionPinholeModel,
  CameraRadialDistortionPinholeModel,
  CameraRadialTangentialDistortionPinholeModel,
  CameraBrownConradyDistortionPinholeModel,
  CameraFisheyeModel
};

/**
 * @brief Input a kCameraType value, return true if the value is valid, otherwise return false.
 * @param kCameraType type
 * @return true or false
 */
bool is_camera_type_valid(kCameraType const &type) {
  return type >= kCameraType::CameraSimplePinholeModel && type <= kCameraType::CameraFisheyeModel;
}

/**
 * @brief Get the camera type name from the enum value
 * @param kCameraType type
 * @return std::string
 */
std::string get_camera_type_name(kCameraType const &type) {
  return photogrammetry::utils::get_enum_name(type);
}

/**
 * @brief Determine whether the camera model name is valid
 * @param kCameraType type
 * @return true or false
 */
bool is_camera_model_name_valid(std::string const &camera_model_name) {
  auto const camera_type = photogrammetry::utils::enum_from_name<kCameraType>(camera_model_name);
  return is_camera_type_valid(camera_type);
}

/**
 * @brief Get the camera type from the camera model name
 * @param kCameraType type
 * @return kCameraType
 */
kCameraType get_camera_type_from_camera_model_name(std::string const &camera_model_name) {
  return photogrammetry::utils::enum_from_name<kCameraType>(camera_model_name);
}
} // namespace camera
} // namespace photogrammetry

#endif // PHOTOGRAMMETRY_CAMERA_TYPE_HPP
