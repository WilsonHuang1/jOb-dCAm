/*********************************************************************
 *  Copyright (c) 2018-2023 Shenzhen Guangjian Technology Co.,Ltd..  *
 *                     All rights reserved.                          *
 *********************************************************************/

#ifndef DEPTRUM_STREAM_INCLUDE_STREAM_TYPES_H_
#define DEPTRUM_STREAM_INCLUDE_STREAM_TYPES_H_

// #include <stdlib.h>
// #include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
// #include <string>
#include <vector>
#include "common_types.h"

namespace deptrum {
namespace stream {

#ifdef __ANDROID__
#define CONFIG_PATH "/sdcard/deptrum/config/"
#else
#define CONFIG_PATH "./config/"
#endif

using ProgressHandler = std::function<void(int)>;
using UpgradeReadyHandler = std::function<void()>;
using HeartbeatResult = std::function<void(int)>;
using IrRawFrameHandler = std::function<void(struct IrRawFrame&)>;

struct DeviceState {
  float rgb_raw_fps;
  float depth_ir_raw_fps;
  float out_fps;
  bool start_stream;
  int error_code;
};

struct IrRawFrame {
  std::shared_ptr<uint8_t> img_raw;
  int img_raw_len = 0;
  uint32_t idx = 0;
  uint64_t timestamp = 0;  // Timestamp of the frame
};

struct HeartbeatParam {
  bool is_enabled{false};
  // Callbacks for each failure, if is_callback_every_failure is 1, then the
  // callback will be called every time the heartbeat fails, otherwise it will be
  // called only when the heartbeat fails for allowable_failures_counts.
  bool is_callback_every_failure{false};
  uint32_t timeout{500};
  uint32_t allowable_failures_counts{4};
};

enum StreamType {
  kInvalidStreamType = 0,
  kRgb,
  kIr,
  kDepth,
  kRgbd,
  kRgbIr,
  kRgbdIr,
  kDepthIr,
  kDepthIrLaser,
  kSpeckleCloud,  // One point per speckle; a speckle may occupy multiple pixels on depth map
  kPointCloud,    // One point per pixel; some pixels may be missing on depth map
  kRgbdPointCloud,
  kRgbdIrPointCloud,
  kRgbdIrFlag,
  kDepthIrFlag,
};

struct DeviceDescription {
  std::string device_name;           // device name
  std::string serial_num;            // serial number
  std::string stream_sdk_version;    // stream sdk version
  std::string rgb_firmware_version;  // rgb firmware version
  std::string ir_firmware_version;   // ir firmware version
  uint16_t pid;                      // device pid
  uint16_t vid;                      // device vid
};

typedef struct Data {
  int32_t data_len;
  const char* data;
} Data;

typedef struct SupportedInfo {
  uint8_t scan_face_mode;
  uint8_t scan_code_mode;
  uint8_t running_7x24_hours;
  Data depth_range;
  uint8_t is_support_se;
  uint8_t is_support_synced_2_img;
} SupportedInfo;

typedef enum TemperatureType {
  kTemperatureCamera = 1,
  kTemperatureVcsel,
  kTemperatureCpu,
} TemperatureType;

struct DeviceSystemInfo {
  int32_t cpu_user{0};
  int32_t cpu_system{0};
  int32_t cpu_idle{0};
  int32_t cpu_io{0};
  int32_t mem_total_kb{0};
  int32_t mem_free_kb{0};
};

struct DeviceDebugInfo {
  uint32_t watch_dog_reboot_count{0};
  uint32_t app_reboot_count{0};
  uint32_t timeout_count{0};
  uint32_t usb_connect_err_count{0};
  uint32_t i2c_transfer_err_count{0};
  uint32_t eeprom_transfer_err_count{0};
  uint32_t tof_get_frame_timeout_count{0};
  uint32_t tof_get_err_frame_count{0};
  uint32_t sniper_get_frame_err_count{0};
  uint32_t sniper_generate_err_count{0};
  std::string last_crash_time;
};

struct CameraParam {  // Camera intrinsic parameters
  float cx;           // Principal point in image, x
  float cy;           // Principal point in image, y
  float fx;           // Focal length x
  float fy;           // Focal length y
};

struct Bbox {
  int16_t x = 0;
  int16_t y = 0;
  int16_t w = 0;
  int16_t h = 0;
};

struct FaceInfo {
  Bbox bbox;
  struct Point {
    float x = 0.0f;
    float y = 0.0f;
  } face5p[5];
};

template<class T>
struct ImuInfo {
  ImuData<T> imu_data;
  int16_t temperature;
  ImuInfo() = default;
  ImuInfo(uint64_t timestmp, T gyro_x, T gyro_y, T gyro_z, T acce_x, T acce_y, T acce_z) :
      imu_data(timestmp, gyro_x, gyro_y, gyro_z, acce_x, acce_y, acce_z) {}
};
template<typename T>
struct OpaqueData {
  std::shared_ptr<T> data;
  int32_t data_len = 0;
};

enum DeptrumNebulaFrameType {
  kFrameRaw = 0,
  kFrameAe,
  kFrameStof,
  kFrameMtof,

  kFrameMtofCpu = kFrameMtof,
  kFrameMtofDsp,
};

enum DeptrumNebulaAeFrameType {
  kAeFrameStof,
  kAeFrameMtof,
};

enum DeptrumNebulaFilterLevel {
  kMtofLowLevel = 0,
  kMtofMiddleLevel,
  kMtofHighLevel,

  kStofLowLevel,
  kStofMiddleLevel,
  kStofHighLevel,

  kMtofAeLowLevel = kStofLowLevel,
  kMtofAeMiddleLevel = kStofMiddleLevel,
  kMtofAeHighLevel = kStofHighLevel,
};

enum DeptrumNebulaAeRoiMode {
  kAeRoiModeNear = 0,
  kAeRoiModeMiddle = 1,
  kAeRoiModeFar = 2,
};

struct ExtraFrameInfo {
  uint8_t ir_gain = 0;
  uint8_t ir_exp = 0;
  bool ir_exp_mode = false;
  bool is_collect_image = false;
  FaceInfo face_info;
  // debug
  OpaqueData<uint8_t> raw_frame1;  // stof s1
  OpaqueData<uint8_t> raw_frame2;  // stof/mtof s2 m2 m6
  OpaqueData<uint8_t> raw_frame3;  // stof/mtof s3 m3 m7
  OpaqueData<uint8_t> raw_frame4;  // ae a4
  uint16_t prev_exposure{0};
  uint16_t current_exposure{0};
  int32_t rgb_exposure{0};
  int32_t z_angle{0};
  int32_t y_height{0};
  uint64_t calib_timestamp = 0;
  uint64_t capture_timestamp = 0;
  float sensor_temperature{0.0f};
  float driver_temperature{0.0f};
  float rx_ntc{0.0f};
  float tx_ntc{0.0f};

  // stof debug
  OpaqueData<uint8_t> img_speckle;
  OpaqueData<uint8_t> img_depth_sniper;
  OpaqueData<uint8_t> img_depth_stof_magic;
  OpaqueData<uint8_t> img_flag;
  OpaqueData<uint8_t> ir_mask;
  OpaqueData<uint8_t> img_mtof_amplitude;

  OpaqueData<PointXyz<float>> img_depth_sniper_point_cloud;
  OpaqueData<PointXyz<float>> img_depth_stof_magic_point_cloud;
  DeptrumNebulaFrameType frame_type;
  bool ae_status{false};

  std::vector<ImuInfo<double>> imu_info_vec;

  uint16_t psensor_value[4]{0};
};

struct StreamFrame {
  int index;                   // Index of the frame
  int size;                    // Frame data size in bytes
  int cols;                    // Number of columns
  int rows;                    // Number of rows
  int bits_per_pixel;          // Number of bits per pixel
  float temperature;           // Driver temperature during this frame
  FrameType frame_type;        // Type of the frame, refer to FrameType
  FrameAttr frame_attr;        // Attributes of the frame, refer to FrameAttr
  ImageFormat image_format;    // Format of the image
  uint64_t timestamp;          // Timestamp of the frame
  std::shared_ptr<void> data;  // Pointer to the frame data
};

struct StreamFrames {
  int count;                                            // Number of frames
  std::vector<std::shared_ptr<StreamFrame>> frame_ptr;  // Pointer to the frames array
  std::shared_ptr<ExtraFrameInfo> extra_info{};         // Extra frames information
};

}  // namespace stream
}  // namespace deptrum

#endif  // DEPTRUM_STREAM_INCLUDE_STREAM_TYPES_H_
