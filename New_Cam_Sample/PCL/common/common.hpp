#ifndef SAMPLE_COMMON_COMMON_HPP_
#define SAMPLE_COMMON_COMMON_HPP_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#include <fstream>
#include <iterator>
#include <conio.h>
#include <process.h>

#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkOutputWindow.h>

#include "Mv3dRgbdApi.h"
#include "Mv3dRgbdDefine.h"
#include "Mv3dRgbdImgProc.h"

typedef struct _POINT_XYZ_
{
	float fX;
	float fY;
	float fZ;
}POINT_XYZ;

typedef struct _POINT_NORMAL_
{
	float fNormalX;
	float fNormalY;
	float fNormalZ;
	float fCurvature;
}POINT_NORMAL;

typedef struct _POINT_XYZBGR_
{
	float fX;
	float fY;
	float fZ;

	union {
		struct
		{
			uint8_t nB;
			uint8_t nG;
			uint8_t nR;
			uint8_t nA;
		};
		float fRgb;
	};
}POINT_XYZBGR;

typedef struct _POINT_XYZNORMALS_
{
	POINT_XYZ pPoint;
	POINT_NORMAL pNormals;
}POINT_XYZNORMALS;

typedef struct _POINT_XYZRGBNORMALS_
{
	POINT_XYZBGR pColorPoint;
	POINT_NORMAL pNormals;
}POINT_XYZRGBNORMALS;

typedef struct _RGB_TEXTURE_DATA_
{
	uint8_t nR;
	uint8_t nG;
	uint8_t nB;
}RGB_TEXTURE_DATA;

pcl::PCLPointField createPointField(std::string strName, uint32_t nOffset, uint8_t nDataType, uint32_t nCount)
{
	pcl::PCLPointField pclField;
	pclField.name = strName;
	pclField.offset = nOffset;
	pclField.datatype = nDataType;
	pclField.count = nCount;
	return pclField;
}

bool convertRGB8P_2_RGB(unsigned char* pSrcData, int nWidth, int nHeight, unsigned char* pDstData)
{
	if (nullptr == pSrcData || nullptr == pDstData)
	{
		return false;
	}

	int nStepDist = nWidth * nHeight;
	int nByteCount = 0;
	for (int nHeightIndex = 0; nHeightIndex < nHeight; ++nHeightIndex)
	{
		for (int nWidthIndex = 0; nWidthIndex < nWidth; ++nWidthIndex)
		{
			int nDataIndex = nHeightIndex * nWidth + nWidthIndex;
			pDstData[nByteCount++] = pSrcData[0 * nStepDist + nDataIndex];
			pDstData[nByteCount++] = pSrcData[1 * nStepDist + nDataIndex];
			pDstData[nByteCount++] = pSrcData[2 * nStepDist + nDataIndex];
		}
		while (nByteCount % 4)
		{
			nByteCount++;
		}
	}

	return true;
}

#ifndef ASSERT
#define ASSERT(x)   do{ \
                if(!(x)) { \
                    LOGE("Assert failed at %s:%d", __FILE__, __LINE__); \
                    LOGE("Source Code: " #x ); \
                    system("pause"); \
                    exit(0); \
                                } \
                        }while(0)
#endif

#ifndef ASSERT_OK
#define ASSERT_OK(x)    do{ \
                int err = (x); \
                if(err != MV3D_RGBD_OK) { \
                LOGE("Assert failed: error %#x at %s:%d", err, __FILE__, __LINE__); \
                    LOGE("Source Code: " #x ); \
                    system("pause"); \
                    exit(0); \
                                } \
                        }while(0)
#endif


#ifdef _WIN32
# include <windows.h>
# include <time.h>
static inline char* getLocalTime()
{
    static char local[26] = { 0 };
    SYSTEMTIME wtm;
    struct tm tm;
    GetLocalTime(&wtm);
    tm.tm_year = wtm.wYear - 1900;
    tm.tm_mon = wtm.wMonth - 1;
    tm.tm_mday = wtm.wDay;
    tm.tm_hour = wtm.wHour;
    tm.tm_min = wtm.wMinute;
    tm.tm_sec = wtm.wSecond;
    tm.tm_isdst = -1;

    strftime(local, 26, "%Y-%m-%d %H:%M:%S", &tm);

    return local;
}

#else
# include <sys/time.h>
# include <unistd.h>
static inline char* getLocalTime()
{
    static char local[26] = { 0 };
    time_t time;

    struct timeval tv;
    gettimeofday(&tv, NULL);

    time = tv.tv_sec;
    struct tm* p_time = localtime(&time);
    strftime(local, 26, "%Y-%m-%d %H:%M:%S", p_time);

    return local;
}
#endif

#define LOG(fmt,...)   printf(fmt "\n", ##__VA_ARGS__)
#define LOGD(fmt,...)  printf("(%s) " fmt "\n", getLocalTime(), ##__VA_ARGS__)
#define LOGE(fmt,...)  printf("(%s) Error: " fmt "\n", getLocalTime(), ##__VA_ARGS__)

#define MAX_IMAGE_COUNT 10

#endif