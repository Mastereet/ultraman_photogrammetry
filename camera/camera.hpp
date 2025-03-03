#ifndef PHOTOGRAMMETRY_CAMERA_HPP
#define PHOTOGRAMMETRY_CAMERA_HPP

#include "core/eigen_types.hpp"

// camera class, Mutiple frames can be associated with a single camera
// they generally have the same intrinsic parameters(focal length, principal point, et al.),
// but different extrinsic parameters the extrinsic parameters are 
// the relative pose between the camera and the world
// the intrinsic parameters are the same for all the frames

namespace photogrammetry {
namespace camera {
class Camera {
public:


private:
};
}
} // namespace photogrammetry

#endif // PHOTOGRAMMETRY_CAMERA_HPP
