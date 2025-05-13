// include/js_lidar_crop/box.hpp (2D version with velocity)
#pragma once
#include <Eigen/Geometry>

// 2D Box with velocity for LaserScan obstacles
template<typename Vec2>
struct Box {
  int id;
  Vec2 position; // center (x,y)
  Vec2 size;     // width,height
  float yaw;                // orientation (unused for AABB)
  Vec2 velocity; // vx,vy

  Box() : id(-1), position(0,0), size(0,0), yaw(0), velocity(0,0) {}
  Box(int _id, const Vec2 &_pos, const Vec2 &_sz,
      float _yaw = 0.0f, const Vec2 &_vel = Vec2(0,0))
    : id(_id), position(_pos), size(_sz), yaw(_yaw), velocity(_vel) {}
};