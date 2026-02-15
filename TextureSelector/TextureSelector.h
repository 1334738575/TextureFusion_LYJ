#ifndef TEXTURE_SELECTOR_LYJ_H
#define TEXTURE_SELECTOR_LYJ_H


#include "TextureFusion_LYJ_Defines.h"


namespace TextureFusion_LYJ
{
    struct TextureSelectorOption
    {
        /* data */
        int threadNum = -1;
    };
    
    class TextureSelector
    {
    private:
        /* data */
        SLAM_LYJ::BaseTriMesh* btm_;
        std::vector<const COMMON_LYJ::CompressedImage*> imgs_;
        std::vector<SLAM_LYJ::Pose3D> Tcws_;
        std::vector<SLAM_LYJ::PinholeCamera> cams_;
        std::vector<SLAM_LYJ::BitFlagVec> imgs2fs_;
        TextureSelectorOption opt_;

    public:
        TextureSelector(/* args */);
        ~TextureSelector();

        void selectTexture(
            SLAM_LYJ::BaseTriMesh& _btm,
            const std::vector<const COMMON_LYJ::CompressedImage*>& _imgs,
            const std::vector<SLAM_LYJ::Pose3D>& _Tcws,
            const std::vector<SLAM_LYJ::PinholeCamera>& _cams,
            const std::vector<SLAM_LYJ::BitFlagVec>& _imgs2fs,
            std::vector<int>& _fs2Tex,
            TextureSelectorOption _opt
        );
    };
    

        
}




#endif