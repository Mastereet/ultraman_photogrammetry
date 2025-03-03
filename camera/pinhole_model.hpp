#ifndef PHOTOGRAMMETRY_PINHOLE_MODEL_HPP
#define PHOTOGRAMMETRY_PINHOLE_MODEL_HPP

#include "camera/camera_model.hpp"
#include "camera/camera_parametres.hpp"
#include <cmath>

namespace photogrammetry {
namespace camera {

// 基础针孔相机模型 - 无畸变
class PinholeCameraModel : public CameraModel<PinholeCameraModel> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using CameraType = PinholeCameraModel;
  PinholeCameraModel(const camera_t camera_id, const size_t width, const size_t height,
                     const CameraParams *params)
      : CameraModel<PinholeCameraModel>(camera_id, width, height),
        intrinsic_params_(std::make_unique<PinholeIntrinsicParams>()) {
    InitCamera(params);
  }
  inline bool InitCamera(const CameraParams *params) {
    return intrinsic_params_->InitCamera(params);
  }
  // 获取内参矩阵
  const Mat33 &IntrinsicsMatrix() const { return intrinsic_params_->IntrinsicMatrix(); }
  const Mat33 &InverseIntrinsicsMatrix() const {
    return intrinsic_params_->InverseIntrinsicMatrix();
  }
  // 获取畸变参数
  const std::vector<double> &DistortionParams() const {
    return intrinsic_params_->DistortionParams();
  }

  // 图像坐标到相机坐标的转换
  Vec2 ima2cam(const Vec2 &point2d) const {
    return {
        (point2d.x() - intrinsic_params_->PrincipalPointX()) / intrinsic_params_->FocalLengthX(),
        (point2d.y() - intrinsic_params_->PrincipalPointY()) / intrinsic_params_->FocalLengthY()};
  }

  // 相机坐标到图像坐标的转换
  Vec2 cam2ima(const Vec2 &point2d) const {
    return Vec2(
        intrinsic_params_->FocalLengthX() * point2d.x() + intrinsic_params_->PrincipalPointX(),
        intrinsic_params_->FocalLengthY() * point2d.y() + intrinsic_params_->PrincipalPointY());
  }

  bool haveDistortion() const { return false; }

  CameraModelType getType() const { return CameraModelType::PINHOLE_CAMERA_START; }
  // 无畸变模型，直接返回输入点
  Vec2 distort(const Vec2 &point_undistorted) const { return point_undistorted; }

  Vec2 undistort(const Vec2 &point_distorted) const { return point_distorted; }

  Mat34 ProjectionMatrix(const CameraExtrinsicParams &extrinsic_params) const {
    return IntrinsicsMatrix() * extrinsic_params.getExtrinsicMatrix();
  }

  std::vector<double> getVariableParams() const {
    return {intrinsic_params_->FocalLengthX(), intrinsic_params_->FocalLengthY(),
            intrinsic_params_->PrincipalPointX(), intrinsic_params_->PrincipalPointY()};
  }

  bool VerifyModelSpecificParams() const { return getVariableParams().size() == 4; }

  // 从图像坐标获取射线方向
  Mat3X operator()(const Mat2X &p) const {
    return (InverseIntrinsicsMatrix() * p.colwise().homogeneous()).colwise().normalized();
  }

  bool updateFromVariableParams(const std::vector<double> &variable_params) {
    if (VerifyModelSpecificParams()) {
      intrinsic_params_->SetFocalLengthX(variable_params[0]);
      intrinsic_params_->SetFocalLengthY(variable_params[1]);
      intrinsic_params_->SetPrincipalPointX(variable_params[2]);
      intrinsic_params_->SetPrincipalPointY(variable_params[3]);
      return true;
    } else {
      std::cerr << "PinholeCameraModel updateFromVariableParams failed: "
                << "Variable params size is not equal to 4" << std::endl;
      return false;
    }
  }

  const std::string ParamsInfo() const { return intrinsic_params_->ParamsInfo(); }

protected:
  std::unique_ptr<PinholeIntrinsicParams> intrinsic_params_;
};

// 一阶径向畸变模型

// 三阶径向畸变模型

// Brown-Conrady畸变模型（径向+切向畸变）
class PinholeCameraBrown : public CameraModel<PinholeCameraBrown> {
public:
  using CameraType = PinholeCameraBrown;
  PinholeCameraBrown(const camera_t camera_id, const size_t width, const size_t height,
                     const CameraParams *params)
      : CameraModel<PinholeCameraBrown>(camera_id, width, height),
        intrinsic_params_(std::make_unique<PinholeIntrinsicParams>()) {
    InitCamera(params);
  }
  // 初始化相机
  inline bool InitCamera(const CameraParams *params) {
    return intrinsic_params_->InitCamera(params);
  }
  // 获取内参矩阵
  const Mat33 &IntrinsicsMatrix() const { return intrinsic_params_->IntrinsicMatrix(); }
  const Mat33 &InverseIntrinsicsMatrix() const {
    return intrinsic_params_->InverseIntrinsicMatrix();
  }
  // 获取畸变参数
  const std::vector<double> &DistortionParams() const {
    return intrinsic_params_->DistortionParams();
  }

  CameraModelType getType() const { return CameraModelType::PINHOLE_CAMERA_BROWN; }

  // 图像坐标到相机坐标的转换
  Vec2 ima2cam(const Vec2 &point2d) const {
    return {
        (point2d.x() - intrinsic_params_->PrincipalPointX()) / intrinsic_params_->FocalLengthX(),
        (point2d.y() - intrinsic_params_->PrincipalPointY()) / intrinsic_params_->FocalLengthY()};
  }

  // 相机坐标到图像坐标的转换
  Vec2 cam2ima(const Vec2 &point2d) const {
    return Vec2(
        intrinsic_params_->FocalLengthX() * point2d.x() + intrinsic_params_->PrincipalPointX(),
        intrinsic_params_->FocalLengthY() * point2d.y() + intrinsic_params_->PrincipalPointY());
  }

  bool haveDistortion() const { return true; }

  Vec2 distort(const Vec2 &p) const { return (p + DistortFunc(DistortionParams(), p)); }

  /*
   * @brief 去畸变（在归一化摄像机平面上）
   * @param point_distorted 畸变点
   * @return 去畸变后的点
   * @note Heikkila J (2000) Geometric Camera Calibration Using Circular Control Points.
   */
  Vec2 undistort(const Vec2 &point_distorted) const {
    const double epsilon = 1e-10;
    Vec2 p_u = point_distorted;

    Vec2 d = DistortFunc(DistortionParams(), p_u);
    while ((p_u + d - point_distorted).lpNorm<1>() >
           epsilon) // manhattan distance between the two points
    {
      p_u = point_distorted - d;
      d = DistortFunc(DistortionParams(), p_u);
    }

    return p_u;
  }

  Mat34 ProjectionMatrix(const CameraExtrinsicParams &extrinsic_params) const {
    return IntrinsicsMatrix() * extrinsic_params.getExtrinsicMatrix();
  }

  std::vector<double> getVariableParams() const {
    std::vector<double> params{intrinsic_params_->FocalLengthX(), intrinsic_params_->FocalLengthY(),
                               intrinsic_params_->PrincipalPointX(),
                               intrinsic_params_->PrincipalPointY()};
    params.insert(params.end(), intrinsic_params_->DistortionParams().begin(),
                  intrinsic_params_->DistortionParams().end());
    return params;
  }

  bool VerifyModelSpecificParams() const { return getVariableParams().size() == 9; }

  // 从图像坐标获取射线方向
  Mat3X operator()(const Mat2X &p) const {
    return (InverseIntrinsicsMatrix() * p.colwise().homogeneous()).colwise().normalized();
  }

  const std::string ParamsInfo() const {
    std::stringstream ss;
    ss << intrinsic_params_->ParamsInfo();
    ss << "Distortion: ";
    const auto &distortion_params = intrinsic_params_->DistortionParams();
    ss << "k1: " << distortion_params[0] << ", k2: " << distortion_params[1]
       << ", k3: " << distortion_params[2] << "\nt1: " << distortion_params[3]
       << ", t2: " << distortion_params[4] << std::endl;
    return ss.str();
  }

  bool updateFromVariableParams(const std::vector<double> &variable_params) {
    if (VerifyModelSpecificParams()) {
      intrinsic_params_->SetFocalLengthX(variable_params[0]);
      intrinsic_params_->SetFocalLengthY(variable_params[1]);
      intrinsic_params_->SetPrincipalPointX(variable_params[2]);
      intrinsic_params_->SetPrincipalPointY(variable_params[3]);
      // k1,k2,k3,t1,t2
      intrinsic_params_->SetDistortion(variable_params);
      return true;
    } else {
      std::cerr << "PinholeCameraBrown updateFromVariableParams failed: "
                << "Variable params size is not equal to 9" << std::endl;
      return false;
    }
  }

protected:
  std::unique_ptr<PinholeIntrinsicParams> intrinsic_params_;

  /*
   * @brief 畸变函数
   * @param distortions 畸变参数
   * @param point2d 归一化摄像机平面上的点
   * @return 畸变后的点
   */
  static Vec2 DistortFunc(const std::vector<double> &distortions, const Vec2 &point2d) {
    const double k1 = distortions[0];
    const double k2 = distortions[1];
    const double k3 = distortions[2];
    const double t1 = distortions[3];
    const double t2 = distortions[4];

    const double r2 = point2d(0) * point2d(0) + point2d(1) * point2d(1);
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    const double k_diff = (k1 * r2 + k2 * r4 + k3 * r6);
    const double t_x = t2 * (r2 + 2 * point2d(0) * point2d(0)) + 2 * t1 * point2d(0) * point2d(1);
    const double t_y = t1 * (r2 + 2 * point2d(1) * point2d(1)) + 2 * t2 * point2d(0) * point2d(1);
    return {point2d(0) * k_diff + t_x, point2d(1) * k_diff + t_y};
  }
};

} // namespace camera
} // namespace photogrammetry

#endif // PHOTOGRAMMETRY_PINHOLE_MODEL_HPP
