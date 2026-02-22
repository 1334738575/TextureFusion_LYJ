#ifndef IMAGE_SELECTOR_LYJ_H
#define IMAGE_SELECTOR_LYJ_H

#include "TextureFusion_LYJ_Defines.h"
#include <CUDAInclude.h>

namespace TextureFusion_LYJ
{
    struct ImageSelectorOption
    {
        /* data */
        int threadNum = -1;
    };
    
    class ImageSelector
    {
    private:
        /* data */
    public:
        ImageSelector(/* args */);
        ~ImageSelector();

        void selectImages(COMMON_LYJ::BaseTriMesh& _btm,
            const std::vector<COMMON_LYJ::Pose3D>& _Tcws,
            const std::vector<COMMON_LYJ::PinholeCamera>& _cams,
            std::vector<COMMON_LYJ::BitFlagVec>& _imgs2fs,
            std::vector<int>& _imgInds,
            ImageSelectorOption _opt);

    private:
        bool project(std::vector<COMMON_LYJ::BitFlagVec>& _imgs2fs);
        bool select(const std::vector<COMMON_LYJ::BitFlagVec>& _imgs2fs,
        std::vector<int>& _imgInds);


    private:
        COMMON_LYJ::BaseTriMesh* btm_;
        std::vector<COMMON_LYJ::Pose3D> Tcws_;
        std::vector<COMMON_LYJ::PinholeCamera> cams_;
        ImageSelectorOption opt_;
    };
    
        
}

#endif