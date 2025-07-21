#include <fstream>
#include <opencv2/opencv.hpp>
#include "functional/base.h"
#include "sample_helper.h"
#if defined(DEVICE_TYPE_NEBULA220_MIPI) || defined(DEVICE_TYPE_STELLAR400_MIPI)
#include <sys/stat.h>
#endif

int ChooceFrameMode(
    std::vector<std::tuple<FrameMode, FrameMode, FrameMode>> device_resolution_vec) {
  int num = 0;
  std::string ir_mode_str, rgb_mode_str, depth_mode_str;
  FrameMode ir_mode, rgb_mode, depth_mode;
  std::cout << "the frame mode supported: " << std::endl;
  std::cout << "     ir_mode               "
            << "     rgb_mode                "
            << "depth_mode" << std::endl;
  for (auto device_resolution : device_resolution_vec) {
    ir_mode = std::get<0>(device_resolution);
    rgb_mode = std::get<1>(device_resolution);
    depth_mode = std::get<2>(device_resolution);
    printf("[%d]: %s%30s%15s\n",
           num++,
           frame_mode_to_string_map[ir_mode].c_str(),
           frame_mode_to_string_map[rgb_mode].c_str(),
           frame_mode_to_string_map[depth_mode].c_str());
  }
  int index = -1;
  while (index < 0 || index >= device_resolution_vec.size()) {
    std::cout << "enter your choice:" << std::endl;
    std::cin >> index;
    std::cout << std::endl;
  }
  return index;
}

void SaveImage(const std::string& file_name, const char* pBuffer, int frame_length) {
  std::ofstream outfile(file_name, std::ofstream::binary);
  if (outfile) {
    outfile.write(pBuffer, frame_length);
    outfile.flush();
    outfile.close();
  }
}

void SavePointXyz(const std::string& file_name, char* pBuffer, int frame_length) {
  deptrum::PointXyz<float>* current_point = (deptrum::PointXyz<float>*) pBuffer;
  auto point_size = frame_length / sizeof(deptrum::PointXyz<float>);

  std::ofstream out(file_name, std::ios::out);
  out << "ply\n";
  out << "format ascii 1.0\n";
  out << "comment created by C++ WritePly\n";
  out << "element vertex " << point_size << "\n";
  out << "property float x\n";
  out << "property float y\n";
  out << "property float z\n";
  out << "end_header\n";

  for (size_t i = 0; i < point_size; i++) {
    out << (float) current_point[i].x << "\t" << (float) current_point[i].y << "\t"
        << (float) current_point[i].z << "\n";
  }
  out.flush();
  out.close();
}

void SavePointXyzRgb(const std::string& file_name, char* pBuffer, int frame_length) {
  deptrum::PointXyzRgb<float>* current_point = (deptrum::PointXyzRgb<float>*) pBuffer;
  auto point_size = frame_length / sizeof(deptrum::PointXyzRgb<float>);

  std::ofstream out(file_name, std::ios::out);
  out << "ply\n";
  out << "format ascii 1.0\n";
  out << "comment created by C++ WritePly\n";
  out << "element vertex " << point_size << "\n";
  out << "property float x\n";
  out << "property float y\n";
  out << "property float z\n";
  out << "property uchar red\n";
  out << "property uchar green\n";
  out << "property uchar blue\n";
  out << "end_header\n";

  for (size_t i = 0; i < point_size; i++) {
    out << (float) current_point[i].x << "\t" << (float) current_point[i].y << "\t"
        << (float) current_point[i].z << "\t " << (float) current_point[i].r << "\t"
        << (float) current_point[i].g << "\t" << (float) current_point[i].b << "\n";
  }
  out.flush();
  out.close();
}

void SavePointXyzRgbIr(const std::string& file_name, char* pBuffer, int frame_length) {
  deptrum::PointXyzRgbIr<float>* current_point = (deptrum::PointXyzRgbIr<float>*) pBuffer;
  auto point_size = frame_length / sizeof(deptrum::PointXyzRgbIr<float>);

  std::ofstream out(file_name, std::ios::out);
  out << "ply\n";
  out << "format ascii 1.0\n";
  out << "comment created by C++ WritePly\n";
  out << "element vertex " << point_size << "\n";
  out << "property float x\n";
  out << "property float y\n";
  out << "property float z\n";
  out << "property uchar red\n";
  out << "property uchar green\n";
  out << "property uchar blue\n";
  out << "property uchar ir\n";
  out << "end_header\n";

  for (size_t i = 0; i < point_size; i++) {
    out << (float) current_point[i].x << "\t" << (float) current_point[i].y << "\t"
        << (float) current_point[i].z << "\t " << (float) current_point[i].r << "\t"
        << (float) current_point[i].g << "\t" << (float) current_point[i].b << "\t"
        << (float) current_point[i].ir << "\n";
  }
  out.flush();
  out.close();
}

// frame_type 2: stof 3:mtof
int SaveOneFrame(const std::string& file_path, const StreamFrames& frames) {
// check file path is exist
#if defined(DEVICE_TYPE_NEBULA220_MIPI) || defined(DEVICE_TYPE_STELLAR400_MIPI)
  if (!file_path.empty()) {
    struct stat info;
    if (stat(file_path.c_str(), &info) != 0) {
      // 文件夹路径不存在，创建路径
      mkdir(file_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }
  }
#endif

  int32_t current_idx{0};
  for (int i = 0; i < frames.count; i++) {
    auto frame = frames.frame_ptr[i];
    if (frame->data.get() != nullptr) {
      if (frame->frame_type == FrameType::kDepthFrame) {
        std::string depth_name;
        if ((frame->frame_attr & FrameAttr::kAttrStof) != 0) {
          depth_name = file_path + "depth_stof_" + std::to_string(frame->index) + "_" +
                       std::to_string(frame->timestamp) + ".raw";
        } else if ((frame->frame_attr & FrameAttr::kAttrMtof) != 0) {
          depth_name = file_path + "depth_mtof_" + std::to_string(frame->index) + "_" +
                       std::to_string(frame->timestamp) + ".raw";
        } else {
          depth_name = file_path + "depth_" + std::to_string(frame->index) + "_" +
                       std::to_string(frame->timestamp) + ".raw";
        }
        SaveImage(depth_name, (const char*) frame->data.get(), frame->size);
        current_idx = frame->index;
      } else if (frame->frame_type == FrameType::kIrFrame && frame->size != 0) {
        // SaveImage("ir.raw", (const char*) frame->data.get(), frame->size);
        cv::Mat ir_mat = cv::Mat(frame->rows, frame->cols, CV_8UC1, (void*) frame->data.get());
        cv::imwrite(file_path + "ir_" + std::to_string(frame->index) + "_" +
                        std::to_string(frame->timestamp) + ".png",
                    ir_mat);
        current_idx = frame->index;
      } else if (frame->frame_type == FrameType::kSemanticFrame) {
        cv::Mat flag_mat = cv::Mat(frame->rows, frame->cols, CV_8UC3, (void*) frame->data.get());
        cv::imwrite(file_path + "flag_rgb_" + std::to_string(frame->index) + "_" +
                        std::to_string(frame->timestamp) + ".png",
                    flag_mat);
        // SaveImage("flag.raw", (const char*) frame->data.get(), frame->size);
      } else if (frame->frame_type == FrameType::kPointCloudFrame) {
        SavePointXyz(file_path + "point_cloud_" + std::to_string(frame->index) + "_" +
                         std::to_string(frame->timestamp) + ".ply",
                     (char*) frame->data.get(),
                     frame->size);
      } else if (frame->frame_type == FrameType::kRgbdPointCloudFrame) {
        SavePointXyzRgb(file_path + "point_cloud_rgb_" + std::to_string(frame->index) + "_" +
                            std::to_string(frame->timestamp) + ".ply",
                        (char*) frame->data.get(),
                        frame->size);
      } else if (frame->frame_type == FrameType::kRgbdIrPointCloudFrame) {
        SavePointXyzRgbIr(file_path + "point_cloud_rgbir_" + std::to_string(frame->index) + "_" +
                              std::to_string(frame->timestamp) + ".ply",
                          (char*) frame->data.get(),
                          frame->size);
      } else {
        // only nebula 200 debug image
        if (frames.extra_info->frame_type == DeptrumNebulaFrameType::kFrameStof) {
          if (frames.extra_info->img_speckle.data_len != 0) {
            cv::Mat img_mat = cv::Mat(180, 240, CV_8UC1, frames.extra_info->img_speckle.data.get());
            cv::imwrite(file_path + "speckle_" + std::to_string(frame->timestamp) + ".png",
                        img_mat);
          }
          if (frames.extra_info->img_flag.data_len != 0) {
            cv::Mat img_mat = cv::Mat(120, 248, CV_8UC1, frames.extra_info->img_flag.data.get());
            cv::imwrite(file_path + "flag_" + std::to_string(frame->timestamp) + ".png", img_mat);
          }
        }
      }
    }
  }

  if (frames.extra_info->raw_frame1.data_len != 0) {
    std::string file_name = file_path + "raw_phase_" + std::to_string(current_idx) + "_" +
                            std::to_string(frames.extra_info->current_exposure) + ".raw";
    SaveImage(file_name,
              (const char*) frames.extra_info->raw_frame1.data.get(),
              frames.extra_info->raw_frame1.data_len);
  }
  return 0;
}
