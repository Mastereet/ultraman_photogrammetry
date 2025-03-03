#ifndef PHOTOGRAMMETRY_CAMERA_MODEL_HPP
#define PHOTOGRAMMETRY_CAMERA_MODEL_HPP

#include "camera/camera_parametres.hpp"
#include "camera/std_types.hpp"
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace photogrammetry {
namespace camera {
template <typename Derived>
class CameraModel {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CameraModel(const camera_t camera_id = UINvaliedCameraId, const size_t width = 0,
              const size_t height = 0)
      : camera_id_(camera_id), width_(width), height_(height) {}

  ~CameraModel() = default;

  inline bool InitCamera(const CameraParams *params) {
    return static_cast<Derived *>(this)->InitCamera(params);
  }
  inline camera_t CameraId() const { return camera_id_; }
  inline void SetCameraId(const camera_t camera_id) { camera_id_ = camera_id; }

  inline size_t width() const { return width_; }
  inline size_t height() const { return height_; }

  inline void SetWidth(const size_t width) { width_ = width; }
  inline void SetHeight(const size_t height) { height_ = height; }

  Vec2 project(const Vec3 &X, const bool ignore_distortion = false) const {
    if (static_cast<Derived *>(this)->haveDistortion() &&
        !ignore_distortion) // apply disto & intrinsics
    {
      return static_cast<Derived *>(this)->cam2ima(
          static_cast<Derived *>(this)->distort(X.hnormalized()));
    } else // apply intrinsics
    {
      return static_cast<Derived *>(this)->cam2ima(X.hnormalized());
    }
  }

  Vec2 residual(const Vec3 &X, const Vec2 &x, const bool ignore_distortion = false) const {
    const Vec2 proj = static_cast<Derived *>(this)->project(X, ignore_distortion);
    return x - proj;
  }

  // 纯虚函数：投影和反投影
  Vec2 ima2cam(const Vec2 &point2d) const { return static_cast<Derived *>(this)->ima2cam(point2d); }
  Vec2 cam2ima(const Vec2 &point2d) const { return static_cast<Derived *>(this)->cam2ima(point2d); }
  bool haveDistortion() const { return static_cast<Derived *>(this)->haveDistortion(); }

  // 获取变量参数
  std::vector<double> getVariableParams() const {
    return static_cast<Derived *>(this)->getVariableParams();
  }
  // 纯虚函数：畸变校正
  Vec2 distort(const Vec2 &point_undistorted) const {
    return static_cast<Derived *>(this)->distort(point_undistorted);
  }
  Vec2 undistort(const Vec2 &point_distorted) const {
    return static_cast<Derived *>(this)->undistort(point_distorted);
  }

  // 相机类型
  CameraModelType getType() const { return static_cast<Derived *>(this)->getType(); }
  // Get bearing vectors from image coordinates
  Mat3X operator()(const Mat2X &p) const { return static_cast<Derived *>(this)->operator()(p); }
  // get projection matrix
  Mat34 ProjectionMatrix(const ExtrinsicParams &extrinsic_params) const {
    return static_cast<Derived *>(this)->ProjectionMatrix(extrinsic_params);
  }
  // 验证相机参数是否符合模型要求
  bool VerifyModelSpecificParams() const {
    return static_cast<Derived *>(this)->VerifyModelSpecificParams();
  }

  bool updateFromVariableParams(const std::vector<double> &variable_params) {
    return static_cast<Derived *>(this)->updateFromVariableParams(variable_params);
  }
  const std::string ParamsInfo() const { return static_cast<Derived *>(this)->ParamsInfo(); }

protected:
  // 相机 ID
  camera_t camera_id_;

  // 图像宽度和高度
  size_t width_;
  size_t height_;
};

/*
 * @brief 相机模型输出流
 * @param os 输出流
 * @param model 相机模型
 * @return 输出流
 */
template <typename Derived>
std::ostream &operator<<(std::ostream &os, const CameraModel<Derived> &model) {
  os << model.ParamsInfo();
  return os;
}

} // namespace camera
} // namespace photogrammetry

#endif // PHOTOGRAMMETRY_CAMERA_MODEL_HPP
