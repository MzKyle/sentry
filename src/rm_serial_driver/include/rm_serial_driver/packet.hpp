// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
struct ReceivePacket
{
   uint8_t header = 0x5A;
   struct  {
    float yaw;
    float pit;
    float rol;
  } eulr; /* 四元数 */
  // uint8_t notice=2; /* 控制命令 */
  // float ball_speed; /* 子弹初速度 */
  // float chassis_speed;

  //char detect_color;  // 0-red 1-blue
  //bool reset_tracker : 1;
  //uint8_t reserved : 6;
  float current_v; // m/s
  float yaw;
  float pitch;
  float roll;
  float aim_x;
  float aim_y;
  float aim_z;
  uint16_t checksum = 0;


} __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xA5;
  struct  {
    float yaw; /* 偏航角(Yaw angle) */
    float pit; /* 俯仰角(Pitch angle) */
    float rol; /* 翻滚角(Roll angle) */
  } gimbal;
  uint8_t notice=2; /* 控制命令 */

  //bool is_fire : 1;
  //uint8_t reserved : 1;
  //float x;
  //float y;
  //float z;
  //float v_yaw;
  float pitch;
  float yaw;

  float vx;
  float vy;
  float wz;
  
  uint16_t checksum = 0;
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
