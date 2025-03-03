#ifndef PHOTOGRAMMETRY_CAMERA_PARAMETRES_HPP
#define PHOTOGRAMMETRY_CAMERA_PARAMETRES_HPP

#include "core/eigen_types.hpp"
#include <sstream>
#include <string>
#include <vector>

namespace photogrammetry {
namespace camera {

// camera model type, 'PINHOLE_CAMERA_START' And 'PINHOLE_CAMERA_END' are
// reserved for pinhole camera model.
enum class CameraModelType {
  NONE = 0,
  PINHOLE_CAMERA_START = 1,
  PINHOLE_CAMERA,         // No distortion
  PINHOLE_CAMERA_RADIAL1, // radial distortion K1
  PINHOLE_CAMERA_RADIAL3, // radial distortion K1,K2,K3
  PINHOLE_CAMERA_BROWN,   // radial distortion K1,K2,K3, tangential distortion T1,T2
  PINHOLE_CAMERA_END,
};

static inline bool isPinhole(CameraModelType model_type) {
  return model_type > CameraModelType::PINHOLE_CAMERA_START &&
         model_type < CameraModelType::PINHOLE_CAMERA_END;
}

// Used to control which camera parameter must be
// considered as variable of held constant for non linear refinement
enum class IntrinsicParameterType : int {
  NONE = 1, // All parameters will be held constant
  ADJUST_FOCAL_LENGTH = 2,
  ADJUST_PRINCIPAL_POINT = 4,
  ADJUST_DISTORTION = 8,
  ADJUST_ALL = ADJUST_FOCAL_LENGTH | ADJUST_PRINCIPAL_POINT | ADJUST_DISTORTION
};

struct CameraParams {
  CameraModelType type_;
  CameraParams(const CameraModelType &type) : type_(type) {}
};

struct PinholeCameraInitParams : public CameraParams {
  // 内参
  double fx = 0.0, fy = 0.0;
  double cx = 0.0, cy = 0.0;
  std::vector<double> distortion;

  PinholeCameraInitParams(const CameraModelType &type) : CameraParams(type) {}
};

class PinholeIntrinsicParams {
protected:
  double fx_, fy_, cx_, cy_;
  std::vector<double> distortion_;

public:
  PinholeIntrinsicParams() noexcept = default;
  bool InitCamera(const CameraParams *params) {
    if (isPinhole(params->type_)) {
      const auto &pinhole_params = static_cast<const PinholeCameraInitParams *>(params);
      this->SetFocalLengthX(pinhole_params->fx);
      this->SetFocalLengthY(pinhole_params->fy);
      this->SetPrincipalPointX(pinhole_params->cx);
      this->SetPrincipalPointY(pinhole_params->cy);
      this->SetDistortion(pinhole_params->distortion);
      delete params;
      return true;
    }
    return false;
  }
  inline double MeanFocalLength() const { return (fx_ + fy_) / 2.0; }
  inline double FocalLengthX() const { return fx_; }
  inline double FocalLengthY() const { return fy_; }
  inline void SetFocalLength(const double focal_length) { fx_ = fy_ = focal_length; }
  inline void SetFocalLengthX(const double focal_length_x) { fx_ = focal_length_x; }
  inline void SetFocalLengthY(const double focal_length_y) { fy_ = focal_length_y; }

  inline double PrincipalPointX() const { return cx_; }
  inline double PrincipalPointY() const { return cy_; }
  inline void SetPrincipalPoint(const double cx, const double cy) { cx_ = cx; }
  inline void SetPrincipalPointX(const double cx) { cx_ = cx; }
  inline void SetPrincipalPointY(const double cy) { cy_ = cy; }

  inline void SetDistortion(const std::vector<double> &distortion) { distortion_ = distortion; }
  inline std::vector<double> DistortionParams() const { return distortion_; }
  inline std::vector<double> &DistortionParams() { return distortion_; }

  inline const Mat33 IntrinsicMatrix() const {
    return (Mat33() << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1).finished();
  }
  inline Mat33 &IntrinsicMatrix() {
    return (Mat33() << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1).finished();
  }

  inline const Mat33 InverseIntrinsicMatrix() const { return IntrinsicMatrix().inverse(); }

  const std::string ParamsInfo() const {
    std::stringstream ss;
    ss << "Focal Length: (" << "fx:" << FocalLengthX() << ", " << "fy:" << FocalLengthY() << ")\n";
    ss << "Principal Point: (" << "cx:" << PrincipalPointX() << ", " << "cy:" << PrincipalPointY()
       << ")\n";
    return ss.str();
  }
  ~PinholeIntrinsicParams() noexcept = default;
};

/*
 * @brief 相机外参
 */
class CameraExtrinsicParams {
protected:
  /*
   * @brief 外参：旋转矩阵和平移向量
   * @note R[I | -C] 其中 t = -RC
   */
  Mat33 rotation_;
  Vec3 center_;

public:
  CameraExtrinsicParams(const Mat33 &rotation = Mat33::Identity(),
                        const Vec3 &center = Vec3::Zero())
      : rotation_(rotation), center_(center) {}

  const Mat33 &Rotation() const { return rotation_; }
  Mat33 &Rotation() { return rotation_; }
  const Vec3 &Center() const { return center_; }
  Vec3 &Center() { return center_; }

  /*
   * @brief 获取平移向量
   * @return 平移向量
   * @note t = -RC
   */
  inline Vec3 getTranslation() const { return -rotation_ * center_; }

  /*
   * @brief 获取外参矩阵
   * @return 外参矩阵
   */
  inline Mat34 getExtrinsicMatrix() const {
    return (Mat34() << rotation_, -rotation_ * center_).finished();
  }

  /*
   * @brief 获取逆外参矩阵
   * @return 逆外参矩阵
   * @note [R, t] -> [R^T, R^T * C]
   */
  inline Mat34 getInverseMatrix() const {
    return (Mat34() << rotation_.transpose(), rotation_.transpose() * center_).finished();
  }

  /*
   * @brief 相机外参矩阵乘法
   * @param other 相机外参矩阵
   * @return 相机外参矩阵
   */
  CameraExtrinsicParams operator*(const CameraExtrinsicParams &other) const {
    return CameraExtrinsicParams(rotation_ * other.rotation_, rotation_ * other.center_ + center_);
  }

  /*
   * @brief 将世界坐标系下的点转换为相机坐标系下的点
   * @param other 世界坐标系下的点
   * @return 相机坐标系下的点
   */
  template <typename T> inline typename T::PlainObject operator()(const T &other) const {
    return rotation_ * (other.colwise() - center_);
  }

  // 对 Vec3 进行偏特化
  inline Vec3 operator()(const Vec3 &other) const {
    return rotation_.transpose() * (other + center_);
  }
};

} // namespace camera

} // namespace photogrammetry

#endif // PHOTOGRAMMETRY_CAMERA_PARAMETRES_HPP
