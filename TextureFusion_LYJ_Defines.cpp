#include "TextureFusion_LYJ_Defines.h"
        


namespace TextureFusion_LYJ
{
    ProBuffer::ProBuffer() {}
    ProBuffer::ProBuffer(const int& _w, const int& _h, COMMON_LYJ::BaseTriMesh* _btmPtr, const float& _minD, const float& _maxD, const float& _csTh, const float& _detDTh)
    {
        init(_w, _h, _btmPtr, _minD, _maxD, _csTh, _detDTh);
    }
    ProBuffer::~ProBuffer(){}
    void ProBuffer::updateTcw(const COMMON_LYJ::Pose3D& _Tcw)
    {
        _Tcw.getMatrix34f(Tcw);
    }

    void ProBuffer::init(const int& _w, const int& _h, COMMON_LYJ::BaseTriMesh* _btmPtr, const float& _minD, const float& _maxD, const float& _csTh, const float& _detDTh)
    {
        w = _w;
        h = _h;
        btmPtr = _btmPtr;
        minD = _minD;
        maxD = _maxD;
        csTh = _csTh;
        detDTh = _detDTh;
        depthsM = cv::Mat(h, w, CV_32FC1);
        fIds.assign(w * h, UINT32_MAX);
        allVisiblePIds.assign(_btmPtr->getVn(), 0);
        allVisibleFIds.assign(_btmPtr->getFn(), 0);
    }

        
}