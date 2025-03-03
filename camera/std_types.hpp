#ifndef PHOTOGRAMMETRY_STD_TYPES_HPP
#define PHOTOGRAMMETRY_STD_TYPES_HPP
// std
#include <condition_variable>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace photogrammetry {

// Unique identifier for cameras.
typedef uint32_t camera_t;
// Unique identifier for images.
typedef uint32_t image_t;
// Each image pair gets a unique ID.
typedef uint64_t image_pair_t;
// Unique identifier for 2D point.
typedef uint32_t point2D_t;
// Unique identifier for 3D point.
typedef uint64_t point3D_t;

/// Portable value used to save an undefined index value
const camera_t UINvaliedCameraId = std::numeric_limits<camera_t>::max();
const image_t UINvaliedImageId = std::numeric_limits<image_t>::max();
const image_pair_t UINvaliedImagePairId =
    std::numeric_limits<image_pair_t>::max();
const point2D_t UINvaliedPoint2DId =
    std::numeric_limits<point2D_t>::max();
const point3D_t UINvaliedPoint3DId =
    std::numeric_limits<point3D_t>::max();
/// Standard Pair of camera_t
using Pair = std::pair<camera_t, camera_t>;

/**
 * @brief Standard Hash_Map class
 * @tparam K type of the keys
 * @tparam V type of the values
 */
template <typename Key, typename Value>
using Hash_Map = std::unordered_map<Key, Value>;
} // namespace photogrammetry

namespace std {
/**
 * @brief 特化 std::hash 模板类，用于计算 Pair 类型的哈希值
 * @tparam K 键类型
 * @tparam V 值类型
 */
template <> struct hash<photogrammetry::Pair> {
  size_t operator()(const photogrammetry::Pair &p) const {
    // 实现哈希函数，将 Pair 类型转换为 size_t 类型的哈希值
    size_t hash_value = 0;
    hash_value ^= std::hash<photogrammetry::camera_t>{}(p.first);
    hash_value ^= std::hash<photogrammetry::camera_t>{}(p.second);
    return hash_value;
  }
};
} // namespace std

#endif // PHOTOGRAMMETRY_STD_TYPES_HPP
