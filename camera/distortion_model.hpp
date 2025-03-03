#ifndef PHOTOGRAMMETRY_DISTORTION_MODEL_HPP
#define PHOTOGRAMMETRY_DISTORTION_MODEL_HPP

#include "core/eigen_types.hpp"

namespace photogrammetry {
namespace camera {

enum class DistortionParamsIdx {
  IDX_DISTORTION_K1 = 4,
  IDX_DISTORTION_K2 = 5,
  IDX_DISTORTION_K3 = 6,
  IDX_DISTORTION_T1 = 7,
  IDX_DISTORTION_T2 = 8,
};

} // namespace camera
} // namespace photogrammetry
#endif // PHOTOGRAMMETRY_DISTORTION_MODEL_HPP