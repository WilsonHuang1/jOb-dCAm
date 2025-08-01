﻿
#pragma once

#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>
#include <map>
#include <cmath>
#include <vector>
#include <memory>
#include <iostream>
#include <algorithm>
#include <functional>

//注意：请将glad.h和glad.c文件添加到工程中
//工程依赖opengl32.lib glu32.lib glfw3.lib
//请使用VS2013版本
namespace RenderImage
{
    //像素格式
    enum RIPixelType
    {
        RIPixelType_Undefined       = 0xFFFFFFFF,
        RIPixelType_Mono8           = 17301505,
        RIPixelType_RGB8_Planar     = 35127329,
        RIPixelType_Coord3D_ABC32f  = 39846080,
        RIPixelType_Coord3D_C16     = 17825976,     
        RIPixelType_RGB8_Packed     = 35127316,
        RIPixelType_RGBD_C16        = 0x82283007,
    };

    enum ColorTableType
    {
        ColorTableHue,
        ColorTableJet,
        ColorTableClassic,
        ColorTableGrayScale,
        ColorTableBiomes,
        ColorTableCold,
        ColorTableWarm,
        ColorTableQuant,
        ColorTablePattern,
    };

    //图像信息
    struct RIFrameInfo
    {
        unsigned char* pData;           //图像数据
        unsigned short nWidth;          //宽度
        unsigned short nHeight;         //高度
        unsigned int   nFrameNum;       //帧号
        unsigned int   nFrameLength;    //数据长度
        RIPixelType    enPixelType;     //像素格式
        unsigned int   nReserved[4];    //预留字节
    };        

    //纹理图像信息
    struct RIFrameTexInfo
    {
        unsigned char* pRgbData;        //Rgb图像数据
        float*         pTexData;        //UV数据
        unsigned short nWidth;          //宽度
        unsigned short nHeight;         //高度
        unsigned int   nFrameNum;       //帧号
        RIPixelType    enPixelType;     //像素格式
        unsigned int   nReserved[4];    //预留字节
    };

    //自动管理数据
    typedef struct _AUTO_MALLOC_DATA_INFO_
    {
        _AUTO_MALLOC_DATA_INFO_()
            : pData(nullptr)
            , nDataLen(0)
            , nDataBufferSize(0)
        {}

        ~_AUTO_MALLOC_DATA_INFO_()
        {
            Release();
        }

        _AUTO_MALLOC_DATA_INFO_(const _AUTO_MALLOC_DATA_INFO_&) = delete;
        _AUTO_MALLOC_DATA_INFO_& operator=(const _AUTO_MALLOC_DATA_INFO_&) = delete;

        bool ResizeData(int nNewDataLen)
        {
            if (this->nDataBufferSize >= nNewDataLen)
            {
                ResetData();
                return true;
            }
            Release();

            this->pData = (unsigned char*)malloc(nNewDataLen);
            if (this->pData)
            {
                this->nDataBufferSize = nNewDataLen;
            }
            ResetData();

            return true;
        }

        void ResetData()
        {
            if (this->pData)
            {
                memset(this->pData, 0, this->nDataBufferSize);
            }
        }

        void Release()
        {
            if (this->pData)
            {
                free(this->pData);
                this->pData = nullptr;
                this->nDataBufferSize = 0;
                this->nDataLen = 0;
            }
        }

        unsigned char*  pData;
        int             nDataLen;
        int             nDataBufferSize;
    }AUTO_MALLOC_DATA_INFO;
}

namespace RenderImage
{
    class RenderTexture;
    struct glfw_state;
    class RenderImgWnd
    {
    public:
        
        RenderImgWnd(int width, int height, const char* title);

        ~RenderImgWnd();

        void CloseWnd();
                
        //渲染2D图像：Mono8、C16、RGB
        void RenderImage(const RIFrameInfo& frame);

        //配置数据范围
        void SetPointCloudRange(float fMinX, float fMaxX, float fMinY, float fMaxY);

        //渲染点云(仅支持RIPixelType_Coord3D_ABC32f格式)
        void RenderPointCloud(const RIFrameInfo& frame);

        //渲染纹理点云(仅支持RGB8Planar格式纹理数据及UV坐标)
        void RenderUVPointCloud(const RIFrameInfo& frame, const RIFrameTexInfo& tex, glfw_state& app_state);

        //C16深度图显示伪彩色时的颜色表
        void SetColorTable(ColorTableType enType);

        //初始化点云渲染环境，3D点云渲染之前调用一次
        void Init3DRender();

        operator bool();

    private:        
        std::shared_ptr<RenderTexture>      m_pTexture;
        std::shared_ptr<glfw_state>         m_pRenderState;
        AUTO_MALLOC_DATA_INFO               m_stPointCloudConvert;
        GLFWwindow*     win                 = nullptr;
        int             _width              = 0;
        int             _height             = 0;
        ColorTableType  m_enColorTable      = ColorTableJet;
        float            m_fMinX                = 0;
        float            m_fMaxX                = 0;
        float            m_fMinY                = 0;
        float            m_fMaxY                = 0;
        
        std::function<void(bool)>           on_left_mouse   = [](bool) {};
        std::function<void(double, double)> on_mouse_scroll = [](double, double) {};
        std::function<void(double, double)> on_mouse_move   = [](double, double) {};
        std::function<void(int)>            on_key_release  = [](int) {};
    };    
}

#define C16_INVALID_VALUE (-32768)
using namespace std;

namespace RenderImage
{    
    struct float2 { float x, y; };
    struct rect
    {
        float x, y;
        float w, h;

        // Create new rect within original boundaries with give aspect ration
        rect adjust_ratio(float2 size) const
        {
            auto H = static_cast<float>(h), W = static_cast<float>(h)* size.x / size.y;
            if (W > w)
            {
                auto scale = w / W;
                W *= scale;
                H *= scale;
            }

            return{ x + (w - W) / 2, y + (h - H) / 2, W, H };
        }
    };

    struct float3
    {
        float x, y, z;
        float & operator[](int i)
        {
            return (*(&x + i));
        }
    };
    inline float3 operator * (const float3 & a, float b) { return{ a.x*b, a.y*b, a.z*b }; }
    inline float3 operator + (const float3 & a, const float3 & b) { return{ a.x + b.x, a.y + b.y, a.z + b.z }; }

    template< typename T>
    inline T clamp_val(T val, const T& min, const T& max)
    {
        const T t = val < min ? min : val;
        return t > max ? max : t;
    }

    void set_viewport(const rect& r)
    {
        glViewport((int)r.x, (int)r.y, (int)r.w, (int)r.h);
        glLoadIdentity();
        glMatrixMode(GL_PROJECTION);
        glOrtho(0, r.w, r.h, 0, -1, +1);
    }

    bool ConvertRGB8P_2_RGB(unsigned char* pSrcData, int nWidth, int nHeight, unsigned char* pDstData);
    
    class texture
    {
    public:
        ~texture();
        void upload(const RIFrameTexInfo* frame)
        {
            if (!frame) return;

            if (!_gl_handle)
                glGenTextures(1, &_gl_handle);
            GLenum err = glGetError();

            RIPixelType format = frame->enPixelType;
            int width = frame->nWidth;
            int height = frame->nHeight;
            _stream_index = frame->nFrameNum;

            glBindTexture(GL_TEXTURE_2D, _gl_handle);

            switch (format)
            {
            case RIPixelType_RGB8_Planar:
                if (NULL == _pRgb8packedBuf)
                {
                    _pRgb8packedBuf = (unsigned char*)malloc(3 * height * width);
                    memset(_pRgb8packedBuf, 0, 3 * height * width);
                }
                if (ConvertRGB8P_2_RGB(frame->pRgbData, width, height, _pRgb8packedBuf))
                {
                    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, _pRgb8packedBuf);
                }
                else
                {
                    throw std::runtime_error("ConvertRGB8P_2_RGB failed!");
                }
                break;
            default:
                throw std::runtime_error("The requested format is not supported by this demo!");
            }

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
            glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
            glBindTexture(GL_TEXTURE_2D, 0);
        }

        void show(const rect& r, float alpha = 1.f) const
        {
            if (!_gl_handle)
                return;

            set_viewport(r);

            glBindTexture(GL_TEXTURE_2D, _gl_handle);
            glColor4f(1.0f, 1.0f, 1.0f, alpha);
            glEnable(GL_TEXTURE_2D);
            glBegin(GL_QUADS);
            glTexCoord2f(0, 0); glVertex2f(0, 0);
            glTexCoord2f(0, 1); glVertex2f(0, r.h);
            glTexCoord2f(1, 1); glVertex2f(r.w, r.h);
            glTexCoord2f(1, 0); glVertex2f(r.w, 0);
            glEnd();
            glDisable(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, 0);
        }

        GLuint get_gl_handle() { return _gl_handle; }

    private:
        GLuint          _gl_handle = 0;
        int             _stream_index{};
        unsigned char*  _pRgb8packedBuf = NULL;
    };

    texture::~texture()
    {
        if (NULL != _pRgb8packedBuf)
        {
            free(_pRgb8packedBuf);
            _pRgb8packedBuf = NULL;
        }
    }

    // Struct for managing rotation of pointcloud view
    struct glfw_state {
        glfw_state(float yaw = 15.0, float pitch = 15.0) : yaw(yaw), pitch(pitch), last_x(0.0), last_y(0.0),
            ml(false), offset_x(2.f), offset_y(2.f) {}
        double yaw;
        double pitch;
        double last_x;
        double last_y;
        bool ml;
        float offset_x;
        float offset_y;
        texture tex;
    };
                
    class ColorTable
    {
    public:

        static ColorTable& GetColorTable(ColorTableType enType);

        ColorTable(std::map<float, float3> map, int steps = 4000);

        ColorTable(const std::vector<float3>& values, int steps = 4000);

        ColorTable();

        inline float3 get(float value) const
        {
            if (_max == _min) return *_data;
            auto t = (value - _min) / (_max - _min);
            t = clamp_val(t, 0.f, 1.f);
            return _data[(int)(t * (_size - 1))];
        }

        float min_key() const { return _min; }
        float max_key() const { return _max; }

        const std::vector<float3>& get_cache() const { return _cache; }

    private:
        inline float3 lerp(const float3& a, const float3& b, float t) const
        {
            return b * t + a * (1 - t);
        }

        float3 calc(float value) const;

        void initialize(int steps);

        std::map<float, float3> _map;
        std::vector<float3> _cache;
        float _min, _max;
        size_t _size; float3* _data;
    };
        
    ////////////////////////
    // Image display code //
    ////////////////////////
    /// \brief The texture class
    bool ConvertImageData(const RIFrameInfo& stSrcImageInfo, RIFrameInfo& stDstImageInfo, ColorTableType enColorTableType);
    bool ConvertMono8_2_RGB(unsigned char* pSrcData, int nWidth, int nHeight, unsigned char* pDstData);
    bool ConvertC16_2_RGB(  unsigned char* pSrcData, int nWidth, int nHeight, unsigned char* pDstData, ColorTable& stColorTable);
    bool ConvertRGB8P_2_RGB(unsigned char* pSrcData, int nWidth, int nHeight, unsigned char* pDstData);
    bool ConvertRGB_2_RGB(  unsigned char* pSrcData, int nWidth, int nHeight, unsigned char* pDstData);
    bool ConvertRGB24D16_2_RGB(unsigned char* pSrcData, int nWidth, int nHeight, unsigned char* pDstData);
    bool NormalizeABC32f(unsigned char* pSrcData, int nDataLen, unsigned char* pDstData, float* fZRatio, float fMinX, float fMaxX, float fMinY, float fMaxY);
    //RGBD渲染使用的颜色表
    std::vector<float3> m_vecRGBDColorTable;
    float g_fContrastEnhance = 0.0;
    float g_fWeightedRatio   = 0.5;
    void InitColorTable()
    {
        if (!m_vecRGBDColorTable.empty())
        {
            return;
        }

        //构建灰度值到彩色的映射表，蓝色->红色
        float3 color;
        for (int i = 0; i < 256; ++i)
        {
            if (i >= 0 && i <= 63)
            {
                color.x = 0;
                color.y = 4 * i + 2;
                color.z = 255;
            }
            else if (i >= 64 && i <= 127)
            {
                color.x = 0;
                color.y = 255;
                color.z = 510 - 4 * i;
            }
            else if (i >= 128 && i <= 191)
            {
                color.x = 4 * i - 510;
                color.y = 255;
                color.z = 0;
            }
            else
            {
                color.x = 255;
                color.y = 1022 - 4 * i;
                color.z = 0;
            }
            m_vecRGBDColorTable.push_back(color);
        }
    }
    class RenderTexture
    {
    public:
        void upload(const RIFrameInfo& frame, ColorTableType enColorTableType)
        {
            if (!_gl_handle)
                glGenTextures(1, &_gl_handle);
            GLenum err = glGetError();

            glBindTexture(GL_TEXTURE_2D, _gl_handle);

            switch (frame.enPixelType)
            {
            case RIPixelType_RGB8_Packed:
            case RIPixelType_RGB8_Planar:
            case RIPixelType_Coord3D_C16:
            case RIPixelType_Mono8:
            case RIPixelType_RGBD_C16:
            {
                //转换为rgb图像显示
                RIFrameInfo stConvertImage = frame;
                if (!m_stConvertData.ResizeData((frame.nWidth + 3) * frame.nHeight * 3))
                {
                    return;
                }
                stConvertImage.pData = m_stConvertData.pData;
                stConvertImage.enPixelType = RIPixelType_RGB8_Packed;
                if (ConvertImageData(frame, stConvertImage, enColorTableType))
                {
                    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, stConvertImage.nWidth, stConvertImage.nHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, stConvertImage.pData);
                }                
            }                
                break;
            default:
                break;
            }

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
            glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
            glBindTexture(GL_TEXTURE_2D, 0);
        }

        void show(const rect& r, float alpha = 1.f) const
        {
            if (!_gl_handle)
                return;

            set_viewport(r);

            glBindTexture(GL_TEXTURE_2D, _gl_handle);
            glColor4f(1.0f, 1.0f, 1.0f, alpha);
            glEnable(GL_TEXTURE_2D);
            glBegin(GL_QUADS);
            glTexCoord2f(0, 0); glVertex2f(0, 0);
            glTexCoord2f(0, 1); glVertex2f(0, r.h);
            glTexCoord2f(1, 1); glVertex2f(r.w, r.h);
            glTexCoord2f(1, 0); glVertex2f(r.w, 0);
            glEnd();
            glDisable(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, 0);
        }

        GLuint get_gl_handle() { return _gl_handle; }

        void render(const RIFrameInfo& frame, const rect& rect, ColorTableType enColorTableType, float alpha = 1.f)
        {
            upload(frame, enColorTableType);
            show(rect.adjust_ratio({ (float)frame.nWidth, (float)frame.nHeight }), alpha);
        }

    private:
        GLuint          _gl_handle = 0;
        int             _stream_index{};
        AUTO_MALLOC_DATA_INFO m_stConvertData;
    };
    
    RenderImgWnd::RenderImgWnd(int width, int height, const char* title)
                  : _width(width)
                  , _height(height)
    {
        glfwInit();
        win = glfwCreateWindow(width, height, title, nullptr, nullptr);
        if (!win)
            throw std::runtime_error("Could not open OpenGL window, please check your graphic drivers or use the textual SDK tools");
        glfwMakeContextCurrent(win);

        glfwSetWindowUserPointer(win, this);
        glfwSetMouseButtonCallback(win, [](GLFWwindow* w, int button, int action, int mods)
        {
            auto s = (RenderImgWnd*)glfwGetWindowUserPointer(w);
            if (button == 0) s->on_left_mouse(action == GLFW_PRESS);
        });

        glfwSetScrollCallback(win, [](GLFWwindow* w, double xoffset, double yoffset)
        {
            auto s = (RenderImgWnd*)glfwGetWindowUserPointer(w);
            s->on_mouse_scroll(xoffset, yoffset);
        });

        glfwSetCursorPosCallback(win, [](GLFWwindow* w, double x, double y)
        {
            auto s = (RenderImgWnd*)glfwGetWindowUserPointer(w);
            s->on_mouse_move(x, y);
        });

        glfwSetKeyCallback(win, [](GLFWwindow* w, int key, int scancode, int action, int mods)
        {
            auto s = (RenderImgWnd*)glfwGetWindowUserPointer(w);
            if (0 == action) // on key release
            {
                s->on_key_release(key);
            }
        });        
    }

    RenderImgWnd::~RenderImgWnd()
    {
        glfwDestroyWindow(win);
        glfwTerminate();
    }

    void RenderImgWnd::CloseWnd()
    {
        glfwSetWindowShouldClose(win, 1);
    }

    RenderImgWnd::operator bool()
    {
        glPopMatrix();
        glfwSwapBuffers(win);

        auto res = !glfwWindowShouldClose(win);

        glfwPollEvents();
        glfwGetFramebufferSize(win, &_width, &_height);

        // Clear the framebuffer
        glClear(GL_COLOR_BUFFER_BIT);
        glViewport(0, 0, _width, _height);

        // Draw the images
        glPushMatrix();
        glfwGetWindowSize(win, &_width, &_height);
        glOrtho(0, _width, _height, 0, -1, +1);

        return res;
    }

    void RenderImgWnd::RenderImage(const RIFrameInfo& frame)
    {
        if (nullptr == m_pTexture)
        {
            m_pTexture = std::make_shared<RenderTexture>();
        }
        if (nullptr == m_pTexture)
        {
            return;
        }        
        m_pTexture->render(frame, {0, 0, (float)_width, (float)_height}, m_enColorTable);
    }

    //配置数据范围
    void RenderImgWnd::SetPointCloudRange(float fMinX, float fMaxX, float fMinY, float fMaxY)
    {
        m_fMinX = fMinX;
        m_fMaxX = fMaxX;
        m_fMinY = fMinY;
        m_fMaxY = fMaxY;
    }

    void RenderImgWnd::RenderPointCloud(const RIFrameInfo& frame)
    {
        if (frame.enPixelType != RIPixelType_Coord3D_ABC32f)
        {
            return;
        }
        if (nullptr == m_pRenderState || frame.nWidth <= 0 || frame.nHeight <= 0)
        {
            return;
        }

        if (!m_stPointCloudConvert.ResizeData(frame.nFrameLength))
        {
            return;
        }
        float fZRatio = 1.0f;
        NormalizeABC32f(frame.pData, frame.nFrameLength, m_stPointCloudConvert.pData, &fZRatio, m_fMinX, m_fMaxX, m_fMinY, m_fMaxY);

        // OpenGL commands that prep screen for the pointcloud
        glLoadIdentity();
        glPushAttrib(GL_ALL_ATTRIB_BITS);

        glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
        glClear(GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        gluPerspective(60, _width / _height, 0.01f, 10.0f);

        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

        glTranslatef(0, 0, +2.2f + m_pRenderState->offset_y * 0.05f);
        glRotated(m_pRenderState->pitch, 1, 0, 0);
        glRotated(m_pRenderState->yaw, 0, 1, 0);
        glTranslatef(0, 0, -0.5f);

        glPointSize(1.0f);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_TEXTURE_2D);
        //glBindTexture(GL_TEXTURE_2D, app_state.tex.get_gl_handle());
        float tex_border_color[] = { 0.8f, 0.8f, 0.8f, 0.8f };
        glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, tex_border_color);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, 0x812F); // GL_CLAMP_TO_EDGE
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, 0x812F); // GL_CLAMP_TO_EDGE
        glBegin(GL_POINTS);


        /* this segment actually prints the pointcloud */
        //auto vertices = points.get_vertices();              // get vertices
        //auto tex_coords = points.get_texture_coordinates(); // and texture coordinates
        float* pPoints = (float*)m_stPointCloudConvert.pData;
        ColorTable stColorTable = ColorTable::GetColorTable(m_enColorTable);
        for (int i = 0; i < frame.nFrameLength / sizeof(float3); i++)
        {
            float3 fColor = stColorTable.get(pPoints[i * 3 + 2] / fZRatio);
            glColor3f(fColor.x, fColor.y, fColor.z);

            if (pPoints[i * 3 + 2])
            {
                // upload the point and texture coordinates only for points we have depth data for                
                glVertex3fv(&(pPoints[i * 3]));
                //glTexCoord2fv(tex_coords[i]);
            }
        }

        // OpenGL cleanup
        glEnd();
        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glPopAttrib();
    }

    void RenderImgWnd::RenderUVPointCloud(const RIFrameInfo& frame, const RIFrameTexInfo& tex, glfw_state& app_state)
    {
        if (frame.enPixelType != RIPixelType_Coord3D_ABC32f)
        {
            return;
        }
        if (nullptr == m_pRenderState || frame.nWidth <= 0 || frame.nHeight <= 0)
        {
            return;
        }

        if (!m_stPointCloudConvert.ResizeData(frame.nFrameLength))
        {
            return;
        }

        float fZRatio = 1.0f;
        NormalizeABC32f(frame.pData, frame.nFrameLength, m_stPointCloudConvert.pData, &fZRatio, m_fMinX, m_fMaxX, m_fMinY, m_fMaxY);

        glLoadIdentity();
        glPushAttrib(GL_ALL_ATTRIB_BITS);

        glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
        glClear(GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        gluPerspective(60, _width / _height, 0.01f, 10.0f);

        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

        glTranslatef(0, 0, +2.2f + m_pRenderState->offset_y * 0.05f);
        glRotated(m_pRenderState->pitch, 1, 0, 0);
        glRotated(m_pRenderState->yaw, 0, 1, 0);
        glTranslatef(0, 0, -0.5f);

        glPointSize(1.0f);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, app_state.tex.get_gl_handle());
        float tex_border_color[] = { 0.8f, 0.8f, 0.8f, 0.8f };
        glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, tex_border_color);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, 0x812F); // GL_CLAMP_TO_EDGE
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, 0x812F); // GL_CLAMP_TO_EDGE
        glBegin(GL_POINTS);


        /* this segment actually prints the pointcloud */
        float* pPoints = (float*)m_stPointCloudConvert.pData;

        for (int i = 0; i < frame.nFrameLength / sizeof(float3); i++)
        {
            if (pPoints[i * 3 + 2])
            {
                // upload the point and texture coordinates only for points we have depth data for                
                glVertex3fv(&(pPoints[i * 3]));
                glTexCoord2fv(&(tex.pTexData[i * 2]));
            }
        }

        // OpenGL cleanup
        glEnd();
        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glPopAttrib();
    }

    void RenderImgWnd::SetColorTable(ColorTableType enType)
    {
        if (enType < ColorTableHue || enType > ColorTablePattern)
        {
            return;
        }
        m_enColorTable = enType;
    }

    void RenderImgWnd::Init3DRender()
    {
        if (nullptr == m_pRenderState)
        {
            m_pRenderState = std::make_shared<glfw_state>();

            this->on_left_mouse = [&](bool pressed)
            {
                m_pRenderState->ml = pressed;
            };

            this->on_mouse_scroll = [&](double xoffset, double yoffset)
            {
                m_pRenderState->offset_x -= static_cast<float>(xoffset);
                m_pRenderState->offset_y -= static_cast<float>(yoffset);
            };

            this->on_mouse_move = [&](double x, double y)
            {
                if (m_pRenderState->ml)
                {
                    m_pRenderState->yaw -= (x - m_pRenderState->last_x);
                    m_pRenderState->yaw = max(m_pRenderState->yaw, -350.0);
                    m_pRenderState->yaw = min(m_pRenderState->yaw, +350.0);
                    m_pRenderState->pitch += (y - m_pRenderState->last_y);
                    m_pRenderState->pitch = max(m_pRenderState->pitch, -350.0);
                    m_pRenderState->pitch = min(m_pRenderState->pitch, +350.0);
                }
                m_pRenderState->last_x = x;
                m_pRenderState->last_y = y;
            };

            this->on_key_release = [&](int key)
            {
                if (key == 32) // Escape
                {
                    m_pRenderState->yaw = m_pRenderState->pitch = 0;  m_pRenderState->offset_x = m_pRenderState->offset_y = 0.0;
                }
            };
        }
    }
           
    static ColorTable hue{ {
        { 255, 0, 0 },
        { 255, 255, 0 },
        { 0, 255, 0 },
        { 0, 255, 255 },
        { 0, 0, 255 },
        { 255, 0, 255 },
        { 255, 0, 0 },
        } };

    static ColorTable jet{ {
        { 0, 0, 255 },
        { 0, 255, 255 },
        { 255, 255, 0 },
        { 255, 0, 0 },
        { 50, 0, 0 },
        } };

    static ColorTable classic{ {
        { 30, 77, 203 },
        { 25, 60, 192 },
        { 45, 117, 220 },
        { 204, 108, 191 },
        { 196, 57, 178 },
        { 198, 33, 24 },
        } };

    static ColorTable grayscale{ {
        { 255, 255, 255 },
        { 0, 0, 0 },
        } };

    static ColorTable inv_grayscale{ {
        { 0, 0, 0 },
        { 255, 255, 255 },
        } };

    static ColorTable biomes{ {
        { 0, 0, 204 },
        { 204, 230, 255 },
        { 255, 255, 153 },
        { 170, 255, 128 },
        { 0, 153, 0 },
        { 230, 242, 255 },
        } };

    static ColorTable cold{ {
        { 230, 247, 255 },
        { 0, 92, 230 },
        { 0, 179, 179 },
        { 0, 51, 153 },
        { 0, 5, 15 }
        } };

    static ColorTable warm{ {
        { 255, 255, 230 },
        { 255, 204, 0 },
        { 255, 136, 77 },
        { 255, 51, 0 },
        { 128, 0, 0 },
        { 10, 0, 0 }
        } };

    static ColorTable quantized{ {
        { 255, 255, 255 },
        { 0, 0, 0 },
        }, 6 };

    static ColorTable pattern{ {
        { 255, 255, 255 },
        { 0, 0, 0 },
        { 255, 255, 255 },
        { 0, 0, 0 },
        { 255, 255, 255 },
        { 0, 0, 0 },
        { 255, 255, 255 },
        { 0, 0, 0 },
        { 255, 255, 255 },
        { 0, 0, 0 },
        { 255, 255, 255 },
        { 0, 0, 0 },
        { 255, 255, 255 },
        { 0, 0, 0 },
        { 255, 255, 255 },
        { 0, 0, 0 },
        { 255, 255, 255 },
        { 0, 0, 0 },
        { 255, 255, 255 },
        { 0, 0, 0 },
        { 255, 255, 255 },
        { 0, 0, 0 },
        { 255, 255, 255 },
        { 0, 0, 0 },
        { 255, 255, 255 },
        { 0, 0, 0 },
        { 255, 255, 255 },
        { 0, 0, 0 },
        { 255, 255, 255 },
        { 0, 0, 0 },
        { 255, 255, 255 },
        { 0, 0, 0 },
        { 255, 255, 255 },
        { 0, 0, 0 },
        { 255, 255, 255 },
        { 0, 0, 0 },
        { 255, 255, 255 },
        { 0, 0, 0 },
        { 255, 255, 255 },
        { 0, 0, 0 },
        { 255, 255, 255 },
        { 0, 0, 0 },
        { 255, 255, 255 },
        { 0, 0, 0 },
        { 255, 255, 255 },
        { 0, 0, 0 },
        { 255, 255, 255 },
        { 0, 0, 0 },
        { 255, 255, 255 },
        { 0, 0, 0 },
        } };

    ColorTable& ColorTable::GetColorTable(ColorTableType enType)
    {
        switch (enType)
        {
        case RenderImage::ColorTableHue:
            return hue;
            break;
        case RenderImage::ColorTableJet:
            return jet;
            break;
        case RenderImage::ColorTableClassic:
            return classic;
            break;
        case RenderImage::ColorTableGrayScale:
            return grayscale;
            break;
        case RenderImage::ColorTableBiomes:
            return biomes;
            break;
        case RenderImage::ColorTableCold:
            return cold;
            break;
        case RenderImage::ColorTableWarm:
            return warm;
            break;
        case RenderImage::ColorTableQuant:
            return quantized;
            break;
        case RenderImage::ColorTablePattern:
            return pattern;
            break;
        default:
            break;
        }

        return ColorTable();
    }

    ColorTable::ColorTable(std::map<float, float3> map, int steps)
        : _map(map)
    {
        initialize(steps);
    }

    ColorTable::ColorTable(const std::vector<float3>& values, int steps)
    {
        for (size_t i = 0; i < values.size(); i++)
        {
            _map[(float)i / (values.size() - 1)] = values[i];
        }
        initialize(steps);
    }

    ColorTable::ColorTable()
    {}

    float3 ColorTable::calc(float value) const
    {
        if (_map.size() == 0) return{ value, value, value };
        // if we have exactly this value in the map, just return it
        if (_map.find(value) != _map.end()) return _map.at(value);
        // if we are beyond the limits, return the first/last element
        if (value < _map.begin()->first)   return _map.begin()->second;
        if (value > _map.rbegin()->first)  return _map.rbegin()->second;

        auto lower = _map.lower_bound(value) == _map.begin() ? _map.begin() : --(_map.lower_bound(value));
        auto upper = _map.upper_bound(value);

        auto t = (value - lower->first) / (upper->first - lower->first);
        auto c1 = lower->second;
        auto c2 = upper->second;
        return lerp(c1, c2, t);
    }

    void ColorTable::initialize(int steps)
    {
        if (_map.size() == 0) return;

        _min = _map.begin()->first;
        _max = _map.rbegin()->first;

        _cache.resize(steps + 1);
        for (int i = 0; i <= steps; i++)
        {
            auto t = (float)i / steps;
            auto x = _min + t*(_max - _min);
            _cache[i] = calc(x);
        }

        // Save size and data to avoid STL checks penalties in DEBUG
        _size = _cache.size();
        _data = _cache.data();
    }

    bool ConvertImageData(const RIFrameInfo& stSrcImageInfo, RIFrameInfo& stDstImageInfo, ColorTableType enColorTableType)
    {
        if (RIPixelType_RGB8_Packed == stDstImageInfo.enPixelType)
        {
            if (RIPixelType_Mono8 == stSrcImageInfo.enPixelType)
            {
                return ConvertMono8_2_RGB(stSrcImageInfo.pData,
                                          stSrcImageInfo.nWidth,
                                          stSrcImageInfo.nHeight,
                                          stDstImageInfo.pData);
            }
            else if (RIPixelType_Coord3D_C16 == stSrcImageInfo.enPixelType)
            {
                return ConvertC16_2_RGB(stSrcImageInfo.pData,
                                        stSrcImageInfo.nWidth,
                                        stSrcImageInfo.nHeight,
                                        stDstImageInfo.pData,
                                        ColorTable::GetColorTable(enColorTableType));
            }
            else if (RIPixelType_RGB8_Planar == stSrcImageInfo.enPixelType)
            {
                return ConvertRGB8P_2_RGB(stSrcImageInfo.pData,
                                          stSrcImageInfo.nWidth,
                                          stSrcImageInfo.nHeight,
                                          stDstImageInfo.pData);
            }
            else if (RIPixelType_RGB8_Packed == stSrcImageInfo.enPixelType)
            {
                return ConvertRGB_2_RGB(stSrcImageInfo.pData,
                                        stSrcImageInfo.nWidth,
                                        stSrcImageInfo.nHeight,
                                        stDstImageInfo.pData);
            }
            else if (RIPixelType_RGBD_C16 == stSrcImageInfo.enPixelType)
            {
                return ConvertRGB24D16_2_RGB(stSrcImageInfo.pData,
                                             stSrcImageInfo.nWidth,
                                             stSrcImageInfo.nHeight,
                                             stDstImageInfo.pData);
            }
        }
        return false;
    }

    bool ConvertMono8_2_RGB(unsigned char* pSrcData, int nWidth, int nHeight, unsigned char* pDstData)
    {
        if (nullptr == pSrcData || nullptr == pDstData)
        {
            return false;
        }

        int nByteCount = 0;
        for (int nHeightIndex = 0; nHeightIndex < nHeight; ++nHeightIndex)
        {
            for (int nWidthIndex = 0; nWidthIndex < nWidth; ++nWidthIndex)
            {
                int nDataIndex = nHeightIndex * nWidth + nWidthIndex;
                pDstData[nByteCount++] = pSrcData[nDataIndex];
                pDstData[nByteCount++] = pSrcData[nDataIndex];
                pDstData[nByteCount++] = pSrcData[nDataIndex];
            }
            while (nByteCount % 4)
            {
                nByteCount++;
            }
        }

        return true;
    }

    bool ConvertC16_2_RGB(unsigned char* pSrcData, int nWidth, int nHeight, unsigned char* pDstData, ColorTable& stColorTable)
    {
        if (nullptr == pSrcData || nullptr == pDstData)
        {
            return false;
        }

        int nDepth = 0;
        int nMax = INT_MIN;
        int nMin = INT_MAX;

        short* pValue = (short*)pSrcData;
        for (int i = 0; i < nWidth * nHeight; ++i)
        {
            nDepth = pValue[i];
            if (C16_INVALID_VALUE == nDepth)
            {
                continue;
            }
            if (nDepth > nMax)
            {
                nMax = nDepth;
            }
            if (nDepth < nMin)
            {
                nMin = nDepth;
            }
        }

        int nCurRange = nMax - nMin;
        int nByteCount = 0;
        for (int nHeightIndex = 0; nHeightIndex < nHeight; ++nHeightIndex)
        {
            for (int nWidthIndex = 0; nWidthIndex < nWidth; ++nWidthIndex)
            {
                int nDataIndex = nHeightIndex * nWidth + nWidthIndex;

                nDepth = pValue[nDataIndex];
                if (C16_INVALID_VALUE == nDepth || 0 == nDepth)
                {
                    pDstData[nByteCount++] = 0;
                    pDstData[nByteCount++] = 0;
                    pDstData[nByteCount++] = 0;
                    continue;
                }
                if (nCurRange != 0)
                {
                    float fTmpPosValue = (nDepth - nMin) / (1.0 * nCurRange);
                    if (fTmpPosValue < 0)
                    {
                        fTmpPosValue = 0;
                    }
                    if (fTmpPosValue > 1)
                    {
                        fTmpPosValue = 1.0;
                    }

                    float3 stColor = stColorTable.get(fTmpPosValue);
                    pDstData[nByteCount++] = stColor.x;
                    pDstData[nByteCount++] = stColor.y;
                    pDstData[nByteCount++] = stColor.z;
                    continue;
                }
                float3 stColor = stColorTable.get(0);
                pDstData[nByteCount++] = stColor.x;
                pDstData[nByteCount++] = stColor.y;
                pDstData[nByteCount++] = stColor.z;
            }
            while (nByteCount % 4)
            {
                nByteCount++;
            }
        }

        return true;
    }

    bool ConvertRGB8P_2_RGB(unsigned char* pSrcData, int nWidth, int nHeight, unsigned char* pDstData)
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

    bool ConvertRGB_2_RGB(unsigned char* pSrcData, int nWidth, int nHeight, unsigned char* pDstData)
    {
        if (nullptr == pSrcData || nullptr == pDstData)
        {
            return false;
        }

        if (nWidth % 4 == 0)
        {
            //宽度整除4，无需转换
            memcpy(pDstData, pSrcData, nWidth * nHeight * 3);
            return true;
        }

        int nByteCount = 0;
        for (int nHeightIndex = 0; nHeightIndex < nHeight; ++nHeightIndex)
        {
            for (int nWidthIndex = 0; nWidthIndex < nWidth; ++nWidthIndex)
            {
                int nDataIndex = nHeightIndex * nWidth + nWidthIndex;
                pDstData[nByteCount++] = pSrcData[3 * nDataIndex + 0];
                pDstData[nByteCount++] = pSrcData[3 * nDataIndex + 1];
                pDstData[nByteCount++] = pSrcData[3 * nDataIndex + 2];
            }
            while (nByteCount % 4)
            {
                nByteCount++;
            }
        }

        return true;
    }

    bool CalcEqualizedHist(const unsigned char* src, unsigned int sstep, unsigned int width, unsigned int height
        , float low_thresh, float high_thresh, bool depth_only, unsigned int& min_elem, unsigned int& max_elem, std::vector<float> &equalized_hist)
    {
        if (low_thresh<0.0f || low_thresh>1.0f || high_thresh<0.0f || high_thresh>1.0f || low_thresh > high_thresh)
        {
            return 0;
        }

        vector<unsigned int> hist;
        size_t size = (1 << (8 * sizeof(unsigned short)));
        hist.resize(size);
        //赋0效率待测试
        memset(&hist[0], 0, hist.size()*sizeof(unsigned int));
        
        unsigned int total_num = 0;
        if (depth_only)
        {
            //src为16位深度图
            for (int i = 0; i < height; ++i)
            {
                const unsigned short* s = (const unsigned short*)(src + i*sstep);
                for (int j = 0; j < width; ++j)
                {
                    unsigned short val = s[j];
                    //深度图0值为无效值
                    if (0 != val)
                    {
                        ++hist[val];
                        ++total_num;
                    }
                }
            }
        }
        else
        {
            //src为RGB24D16_Packed格式的图像数据(R,G,B,D1,D2...)
            for (int i = 0; i < height; ++i)
            {
                const unsigned char* s = src + i*sstep;
                for (int j = 0; j < width; ++j)
                {
                    unsigned short val = (s[5 * j + 3]) + (s[5 * j + 4] << 8);
                    //深度图0值为无效值
                    if (0 != val)
                    {
                        ++hist[val];
                        ++total_num;
                    }
                }
            }
        }

        if (0 == total_num)
        {
            equalized_hist.resize(hist.size());
            memset(&equalized_hist[0], 0, equalized_hist.size()*sizeof(float));
            min_elem = max_elem = 0;
            return true;
        }

        equalized_hist.resize(hist.size());

        for (int i = 1; i < hist.size(); ++i)
        {
            hist[i] += hist[i - 1];
            equalized_hist[i] = 1.0f*hist[i] / total_num;
        }

        //TODO:图像平滑后是否需要设置阈值
        //在[0.01,0.99]范围内获取最大值和最小值，消除噪点影响
        int min_val = 0;
        int max_val = 0;
        for (int i = 1; i < equalized_hist.size(); ++i)
        {
            if (equalized_hist[i] >= low_thresh)
            {
                min_val = i;
                break;
            }
        }
        for (int i = equalized_hist.size() - 1; i > 0; --i)
        {
            if (equalized_hist[i] <= high_thresh)
            {
                max_val = i;
                break;
            }
        }
        if (max_val < min_val)
        {
            max_val = min_val;
        }

        min_elem = min_val;
        max_elem = max_val;

        return true;
    }

    bool ConvertRGB24D16_2_RGB(unsigned char* pSrcData, int nWidth, int nHeight, unsigned char* pDstData)
    {
        vector<float> equalized_hist;
        unsigned int min_val = 0;
        unsigned int max_val = 0;

        CalcEqualizedHist(pSrcData, nWidth * 5, nWidth, nHeight, 0.01, 0.99, false, min_val, max_val, equalized_hist);

        //初始化颜色映射表
        InitColorTable();
                
        int nCurRange  = max_val - min_val;
        int nByteCount = 0;
        for (int nHeightIndex = 0; nHeightIndex < nHeight; ++nHeightIndex)
        {
            for (int nWidthIndex = 0; nWidthIndex < nWidth; ++nWidthIndex)
            {
                int nDataIndex = nHeightIndex * nWidth + nWidthIndex;

                unsigned short* pDepthValue = (unsigned short*)(pSrcData + nDataIndex * 5 + 3);
                unsigned short nDepthValue = *pDepthValue;
                if (nCurRange != 0 && nDepthValue != 0)
                {
                    int color_index = 0;
                    if (nDepthValue <= min_val)
                    {
                        color_index = 255;
                    }
                    else if (nDepthValue >= max_val)
                    {
                        color_index = 0;
                    }
                    else
                    {
                        //综合考虑直方图均衡化和直方图正规化
                        color_index = (g_fContrastEnhance*(1 - equalized_hist[nDepthValue])
                                       + (1.0f - g_fContrastEnhance) * (max_val - nDepthValue) / nCurRange) * 255;
                    }
                    pDstData[nByteCount++] = g_fWeightedRatio * m_vecRGBDColorTable[color_index].x + (1 - g_fWeightedRatio) * pSrcData[nDataIndex * 5 + 0];
                    pDstData[nByteCount++] = g_fWeightedRatio * m_vecRGBDColorTable[color_index].y + (1 - g_fWeightedRatio) * pSrcData[nDataIndex * 5 + 1];
                    pDstData[nByteCount++] = g_fWeightedRatio * m_vecRGBDColorTable[color_index].z + (1 - g_fWeightedRatio) * pSrcData[nDataIndex * 5 + 2];
                }
                else
                {
                    //使用RGB图像颜色
                    pDstData[nByteCount++] = pSrcData[nDataIndex * 5 + 0];
                    pDstData[nByteCount++] = pSrcData[nDataIndex * 5 + 1];
                    pDstData[nByteCount++] = pSrcData[nDataIndex * 5 + 2];
                }
            }
            while (nByteCount % 4)
            {
                nByteCount++;
            }
        }

        return true;
    }

    bool NormalizeABC32f(unsigned char* pSrcData, int nDataLen, unsigned char* pDstData, float* fZRatio, float fMinX, float fMaxX, float fMinY, float fMaxY)
    {
        if (nullptr == pSrcData || nullptr == pDstData)
        {
            return false;
        }
        int nPointNum = nDataLen / (sizeof(float) * 3);
        std::vector<float> vecX(nPointNum);
        std::vector<float> vecY(nPointNum);
        std::vector<float> vecZ(nPointNum);
        float* pSrcValue = (float*)pSrcData;
        float* pDstValue = (float*)pDstData;
        for (int nPntIndex = 0; nPntIndex < nPointNum; ++nPntIndex)
        {
            vecX[nPntIndex] = pSrcValue[nPntIndex * 3 + 0];
            vecY[nPntIndex] = pSrcValue[nPntIndex * 3 + 1];
            vecZ[nPntIndex] = pSrcValue[nPntIndex * 3 + 2];
        }
        auto funcCalcDistance = [](const std::vector<float>& vec, float& fMin, float& fMax) {
            auto itMin = std::min_element(vec.begin(), vec.end());
            auto itMax = std::max_element(vec.begin(), vec.end());
            if (itMin != vec.end() && itMax != vec.end())
            {
                fMax = *itMax;
                fMin = *itMin;
            }            
        };
        float fMinZ = 0.0f, fMaxZ = 0.0f;
        if (fMinX >= fMaxX)
        {
            funcCalcDistance(vecX, fMinX, fMaxX);
        }
        if (fMinY >= fMaxY)
        {
            funcCalcDistance(vecY, fMinY, fMaxY);
        }       
        funcCalcDistance(vecZ, fMinZ, fMaxZ);
        float fDistX = fMaxX - fMinX;
        float fDistY = fMaxY - fMinY;
        float fDistZ = fMaxZ - fMinZ;
        float fMaxDist = max(max(fDistX, fDistY), fDistZ);
        if (fZRatio)
        {
            *fZRatio = fDistZ / fMaxDist;
        }
        if (fDistX > 0 && fDistY > 0 && fDistZ > 0)
        {
            for (int nPntIndex = 0; nPntIndex < nPointNum; ++nPntIndex)
            {
                pDstValue[nPntIndex * 3 + 0] = (vecX[nPntIndex] - fMinX) / fDistX * (fDistX / fMaxDist) * 2 - 1.0f;
                pDstValue[nPntIndex * 3 + 1] = (vecY[nPntIndex] - fMinY) / fDistY * (fDistY / fMaxDist) * 2 - 1.0f;
                pDstValue[nPntIndex * 3 + 2] = (vecZ[nPntIndex] - fMinZ) / fDistZ * (fDistZ / fMaxDist);
            }
            return true;
        }

        return false;
    }
};
