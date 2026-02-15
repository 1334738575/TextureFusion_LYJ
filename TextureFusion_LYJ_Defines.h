#ifndef TEXTUREFUSION_LYJ_DEFINES_H
#define TEXTUREFUSION_LYJ_DEFINES_H


#ifdef WIN32
#ifdef _MSC_VER
#define TEXTUREFUSION_LYJ_API __declspec(dllexport)
#else
#define TEXTUREFUSION_LYJ_API
#endif
#else
#define TEXTUREFUSION_LYJ_API
#endif


#include <common/BaseTriMesh.h>
#include <common/CompressedImage.h>
#include <common/Timer.h>
#include <base/PreDefine.h>
#include <IO/BaseIO.h>
#include <IO/MeshIO.h>

namespace TextureFusion_LYJ
{
    class ProBuffer
    {
    public:

        ProBuffer();
        ProBuffer(const int& _w, const int& _h, SLAM_LYJ::BaseTriMesh* _btmPtr, const float& _minD=0.01f, const float& _maxD=FLT_MAX, const float& _csTh=0.5f, const float& _detDTh=0.01f);
        ~ProBuffer();
        void updateTcw(const SLAM_LYJ::Pose3D& _Tcw);

        void init(const int& _w, const int& _h, SLAM_LYJ::BaseTriMesh* _btmPtr, const float& _minD = 0.01f, const float& _maxD = FLT_MAX, const float& _csTh = 0.5f, const float& _detDTh = 0.01f);

        int w = 0;
        int h = 0;
        SLAM_LYJ::BaseTriMesh* btmPtr = nullptr;
        Eigen::Matrix<float, 3, 4> Tcw;
        cv::Mat depthsM;
        std::vector<uint> fIds;
        std::vector<char> allVisiblePIds;
        std::vector<char> allVisibleFIds;
        float minD = 0.01f;
        float maxD = FLT_MAX;
        float csTh = 0.5f;
        float detDTh = 0.01f;

    };

    struct TextureFusionOption
    {
        int threadNum = -1;
    };
}

#endif//TextureFusion_LYJ_DEFINES_H