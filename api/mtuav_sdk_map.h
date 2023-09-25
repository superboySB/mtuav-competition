#ifndef MTUAV_SDK_MAP_H
#define MTUAV_SDK_MAP_H

#include <cstdint>
#include <memory>

#include "mtuav_sdk_export.h"

#if OS_LINUX
#define PACKED __attribute__((__packed__))
#elif OS_WIN
#define PACKED
#endif

namespace mtuav {
enum BlockSemantic {
  // 普通地面
  SEM_GROUND = 0,
  // 绿植区
  SEM_VEGETATION = 3,
  // 道路
  SEM_ROAD = 4,
  // 房屋
  SEM_BUILDING = 8,
  // 危险区域
  SEM_DANGEROUS = 18,
};

#if OS_WIN
#pragma pack(push,1)
#endif
// 体素信息，提供简单的esdf数据
typedef struct PACKED Voxel {
  // 当前位置距离地图实体的最小距离
  // 如果距离《=0, 则表示当前位置为地图实体，或者位于地图实体内部
  float distance;
  // reserved for future use
  uint16_t reserved;
  // ref BlockSemantic，
  // 若当前位置是地图实体，表示它语义
  // 若当前位置不是地图实体，表示正下方地图实体的语义
  uint8_t semantic;

  Voxel() : distance(0), reserved(0), semantic(0) {}
  Voxel(const Voxel& o) = default;
  Voxel& operator=(const Voxel& o) = default;
  Voxel(Voxel&& o) noexcept
      : distance(o.distance), reserved(o.reserved), semantic(o.semantic) {}
  Voxel& operator=(Voxel&& o) noexcept {
    distance = o.distance;
    reserved = o.reserved;
    semantic = o.semantic;
    return *this;
  }
} Voxel;
#if OS_WIN
#pragma pack(pop)
#endif

static_assert(sizeof(Voxel)==7, "error Voxel struct packing");

class SDK_EXPORT Map {
 public:
  static std::shared_ptr<Map> CreateMapFromFile(std::string map_path);

 public:
  virtual ~Map();

  // 获取地图的范围，单位都是米
  virtual void Range(float* min_x, float* max_x, float* min_y,
                     float* max_y, float* min_z, float* max_z) = 0;

  // 查询给定位置的体素信息
  virtual const Voxel* Query(float x, float y, float z) = 0;
};
}

#endif //MTUAV_SDK_MAP_H
