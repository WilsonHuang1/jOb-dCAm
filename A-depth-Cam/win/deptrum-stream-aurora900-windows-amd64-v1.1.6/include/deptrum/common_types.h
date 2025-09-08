/*********************************************************************
 *  Copyright (c) 2018-2023 Shenzhen Guangjian Technology Co.,Ltd..  *
 *                     All rights reserved.                          *
 *********************************************************************/

#ifndef DEPTRUM_COMMON_TYPES_H_
#define DEPTRUM_COMMON_TYPES_H_

#include <cstdint>
#include <string>

#ifndef DEPTRUM_DLL
#if _WIN32
#ifdef DEPTRUM_EXPORT
#define DEPTRUM_DLL __declspec(dllexport)
#else
#define DEPTRUM_DLL __declspec(dllimport)
#endif  // DEPTRUM_EXPORT
#else   // Not _WIN32
#define DEPTRUM_DLL
#endif
#endif

namespace deptrum {

enum CameraComponent {
  kInvalidCameraComponent = 0,
  kIrCamera,
  kRgbCamera,
  kLaser,
  kLed,
};

enum FrameMode {
  kInvalid = 0,

  kRes320x240RgbJpeg,    // RGB frame format: 320 columns, 240 rows, pixel in bgr888
  kRes400x256RgbJpeg,    // RGB frame format: 400 columns, 256 rows, pixel in bgr888
  kRes480x640RgbJpeg,    // RGB frame format: 480 columns, 640 rows, pixel in bgr888
  kRes640x480RgbJpeg,    // RGB frame format: 640 columns, 480 rows, pixel in bgr888
  kRes800x512RgbJpeg,    // RGB frame format: 800 columns, 512 rows, pixel in bgr888
  kRes960x540RgbJpeg,    // RGB frame format: 960 columns, 540 rows, pixel in bgr888
  kRes960x1280RgbJpeg,   // RGB frame format: 960 columns, 1280 rows, pixel in bgr888
  kRes1080x1920RgbJpeg,  // RGB frame format: 1080 columns, 1920 rows, pixel in bgr888
  kRes1280x960RgbJpeg,   // RGB frame format: 1280 columns, 960 rows, pixel in bgr888
  kRes1600x1080RgbJpeg,  // RGB frame format: 1600 columns, 1080 rows, pixel in bgr888
  kRes1920x1080RgbJpeg,  // RGB frame format: 1920 columns, 1080 rows, pixel in bgr888
  kRes1600x1024RgbJpeg,  // RGB frame format: 1600 columns, 1024 rows, pixel in bgr888

  kRes320x200Ir8Bit,    // IR frame format: 320 columns, 200 rows, 8 bits per pixel
  kRes320x240Ir8Bit,    // IR frame format: 320 columns, 240 rows, 8 bits per pixel
  kRes400x640Ir8Bit,    // IR frame format: 400 columns, 640 rows, 8 bits per pixel
  kRes400x640Ir16Bit,   // IR frame format: 400 columns, 640 rows, 16 bits per pixel
  kRes480x300Ir8Bit,    // IR frame format: 480 columns, 300 rows, 8 bits per pixel
  kRes480x640Ir8Bit,    // IR frame format: 480 columns, 640 rows, 8 bits per pixel
  kRes480x640Ir16Bit,   // IR frame format: 480 columns, 640 rows, 16 bits per pixel
  kRes640x400Ir8Bit,    // IR frame format: 640 columns, 400 rows, 8 bits per pixel
  kRes640x480Ir8Bit,    // IR frame format: 640 columns, 480 rows, 8 bits per pixel
  kRes650x800Ir8Bit,    // IR frame format: 650 columns, 800 rows, 8 bits per pixel
  kRes736x480Ir8Bit,    // IR frame format: 736 columns, 480 rows, 8 bits per pixel
  kRes800x1280Ir8Bit,   // IR frame format: 960 columns, 1280 rows, 8 bits per pixel
  kRes800x1280Ir16Bit,  // IR frame format: 960 columns, 1280 rows, 16 bits per pixel

  kRes240x180Depth16Bit,   // Depth frame format: 240 columns, 180 rows, 16 bits per pixel
  kRes640x480Depth16Bit,   // Depth frame format: 640 columns, 480 rows, 16 bits per pixel
  kRes400x640Depth16Bit,   // Depth frame format: 400 columns, 640 rows, 16 bits per pixel
  kRes480x640Depth16Bit,   // Depth frame format: 480 columns, 640 rows, 16 bits per pixel
  kRes800x1280Depth16Bit,  // Depth frame format: 800 columns, 1280 rows, 16 bits per pixel

  kRes320x200RgbYuv,  // RGB frame format: 320 columns, 200 rows, pixel in yuv420
  kRes480x300RgbYuv,  // RGB frame format: 480 columns, 300 rows, pixel in yuv420
  kRes640x400RgbYuv,  // RGB frame format: 640 columns, 400 rows, pixel in yuv420
};

enum FrameDecodeMode {
  kSoftwareDecode = 0,  // CPU
  kHardwareDecode,      // GPU/VPU. If not supported, fall back to software decode.
  kNoDecode,
};

enum FrameType {
  kInvalidFrameType = 0,
  kRgbFrame,
  kIrFrame,
  kDepthFrame,
  kLaserFrame,
  kPointCloudFrame,  // Both PointCloud and SpeckleCloud use this type of frame
  kRgbdPointCloudFrame,
  kRgbdIrPointCloudFrame,
  kSemanticFrame,
  kRawFrame,
};

enum FrameAttr {
  kAttrInvalid = 0,
  kAttrLeft = 0x1,
  kAttrRight = 0x2,
  kAttrSl = 0x4,
  kAttrItof = 0x8,
  kAttrMtof = 0x10,
  kAttrStof = 0x20,
  kAttrAe = 0x40,
};

enum ImageFormat {
  kInvalidImageFormat = 0,
  kRaw8,
  kRaw10,
  kRaw12,
  kRaw16,
  kRgb888,
  kRgba,
  kYuv420Nv12,
  kYuv420Nv21,
  kJpeg,
};

struct Frame {
  int index;                 // Index of the frame
  int size;                  // Frame data size in bytes
  int cols;                  // Number of columns
  int rows;                  // Number of rows
  int bits_per_pixel;        // Number of bits per pixel
  float temperature;         // Driver temperature during this frame
  FrameType frame_type;      // Type of the frame, refer to FrameType
  ImageFormat image_format;  // Format of the image
  uint64_t timestamp;        // Timestamp of the frame
  void* data;                // Pointer to the frame data
};

struct Frames {
  int count;         // Number of frames
  Frame* frame_ptr;  // Pointer to the frames array
};

struct Version {
  uint8_t major;
  uint8_t minor;
  uint8_t revision;
  char tag[8];
};

struct DeviceVersionInfo {
  Version camera_sdk_version;    // Camera SDK version
  Version firmware_version;      // Device firmware version
  Version rgb_firmware_version;  // RGB camera firmware version
  Version calib_version;         // Calibration version
  Version depth_version;         // Depth algorithm library version
};

struct CameraComponentId {
  uint8_t device_addr = 0;
  uint16_t vid = 0;      // Vendor ID
  uint16_t pid = 0;      // Product ID
  uint16_t bcd_usb = 0;  // 0x0210 USB 3.0, 0x0200 USB 2.0, 0x0110 USB 1.1
  std::string serial_number;
  std::string name;
  std::string port_path;  // USB port path
#ifdef __ANDROID__
  int fd;
  int bus_num;
  char* usbfs;
#endif
};

struct DeviceInformation {
  std::string model;
  CameraComponentId ir_camera;
  CameraComponentId rgb_camera;
};

enum DistortionCoeffType {
  kDistortionCoeffType4 = 4,    // [k1,k2,p1,p2] or [k1,k2,k3,k4]
  kDistortionCoeffType5 = 5,    // [k1,k2,p1,p2,k3]
  kDistortionCoeffType8 = 8,    // [k1,k2,p1,p2,k3,k4,k5,k6]
  kDistortionCoeffType12 = 12,  // [k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4]
  kDistortionCoeffType14 = 14   // [k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4,τx,τy]
};

// Camera intrinsic parameters
struct Intrinsic {
  int rows;                     // Rows of the image
  int cols;                     // Columns of the image
  float focal_length[2];        // [fx, fy]: Focal length of the image plane
  float principal_point[2];     // [cx, cy]: Coordinates of the principal point of the image
  float distortion_coeffs[14];  // Pinhole or fisheye camera model distortion coefficients
                                // [k1,k2,p1,p2[,k3[,k4,k5,k6[,s1,s2,s3,s4[,τx,τy]]]]], or
                                // [k1,k2,k3,k4]
  DistortionCoeffType distortion_coeff_type;  // Distortion coefficients type

  Intrinsic() : distortion_coeff_type(kDistortionCoeffType5) {}
};

// Camera extrinsic parameters
struct Extrinsic {
  float rotation_matrix[9];     // 3x3 Rotation matrix stored in row major order
  float translation_vector[3];  // Translation vector, x,y,z (mm)
};

template<class T>
struct PointXy {
  T x, y;
  PointXy(T x, T y) : x(x), y(y) {}
  PointXy() : PointXy(0, 0) {}
};

template<class T>
struct PointXyz {
  T x, y, z;
  PointXyz(T x, T y, T z) : x(x), y(y), z(z) {}
  PointXyz() : PointXyz(0, 0, 0) {}
};

template<class T>
struct PointXyzRgb {
  T x, y, z;
  uint8_t r, g, b;
  PointXyzRgb(T x, T y, T z, uint8_t r, uint8_t g, uint8_t b) :
      x(x),
      y(y),
      z(z),
      r(r),
      g(g),
      b(b) {}
  PointXyzRgb(T x, T y, T z) : PointXyzRgb(x, y, z, 0, 0, 0) {}
  PointXyzRgb() : PointXyzRgb(0, 0, 0, 0, 0, 0) {}
};

template<class T>
struct PointXyzRgbIr {
  T x, y, z;
  uint8_t r, g, b, ir;
  PointXyzRgbIr(T x, T y, T z, uint8_t r, uint8_t g, uint8_t b, uint8_t ir) :
      x(x),
      y(y),
      z(z),
      r(r),
      g(g),
      b(b),
      ir(ir) {}
  PointXyzRgbIr(T x, T y, T z) : PointXyzRgbIr(x, y, z, 0, 0, 0, 0) {}
  PointXyzRgbIr() : PointXyzRgbIr(0, 0, 0, 0, 0, 0, 0) {}
};

template<class T>
struct ImuData {
  uint64_t timestamp;
  T gyro_x, gyro_y, gyro_z, acce_x, acce_y, acce_z;
  ImuData(uint64_t timestamp, T gyro_x, T gyro_y, T gyro_z, T acce_x, T acce_y, T acce_z) :
      timestamp(timestamp),
      gyro_x(gyro_x),
      gyro_y(gyro_y),
      gyro_z(gyro_z),
      acce_x(acce_x),
      acce_y(acce_y),
      acce_z(acce_z) {}
};

struct ImuIntrinsic {
  int imu_channels;        // gyro_x, gyro_y, gyro_z, acce_x, acce_y,acce_z
  float acce_scaling[3];   // acce_x, acce_y, acce_z
  float gyro_scaling[3];   // gyro_x, gyro_y, gyro_z
  float acce_misalign[6];  // -omega_yz, omega_zy,omega_xz,-omega_zx,-omegaxy,omega_yx
  float gyro_misalign[6];  // -omega_yz, omega_zy,omega_xz,-omega_zx,-omegaxy,omega_yx
  float acce_bias[3];      // acce_bias_x,acce_bias_y,acce_bias_z
  float gyro_bias[3];      // gyro_bias_x,gyro_bias_y,gyro_bias_z
  float temperature;       // imu temperature
  float gyro_a[9];         // Gyro, size_effect matrix, kalibr
  float gyro_c[9];         // Gyro, transformation from imu_frame to gyro_frame
  float gyro_m[9];         // Gyro, scale and misalignment matrix
  float acce_m[9];         // Acce, scale and misalignment matrix
};
}  // namespace deptrum

#endif  // DEPTRUM_COMMON_TYPES_H_
