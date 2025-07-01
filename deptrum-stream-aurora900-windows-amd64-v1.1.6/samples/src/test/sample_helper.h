#ifndef SAMPLES_SRC_SAMPLE_HELPER_H_
#define SAMPLES_SRC_SAMPLE_HELPER_H_
#include <iostream>
#include <vector>
#include "deptrum/stream_types.h"

using namespace std;
using namespace deptrum;
using namespace deptrum::stream;

int ChooceFrameMode(std::vector<std::tuple<FrameMode, FrameMode, FrameMode>> device_resolution_vec);
int SaveOneFrame(const std::string& file_path, const StreamFrames& frames);

#endif  // SAMPLES_SRC_SAMPLE_HELPER_H_