#ifndef TEXTURE_FUSION_LYJ_H
#define TEXTURE_FUSION_LYJ_H


#include "ImageSelector/ImageSelector.h"
#include "TextureSelector/TextureSelector.h"


namespace TextureFusion_LYJ
{
    struct TextureBlock
    {
        /* data */
        int imgId = -1;
        Eigen::Vector2i leftup = Eigen::Vector2i(99999,99999);
        Eigen::Vector2i rightdown = Eigen::Vector2i::Zero();
        int w = 0;
        int h = 0;
        Eigen::Vector2i leftupFinal = Eigen::Vector2i::Zero();
        std::vector<int> fids;
    };
    
    class TextureFusioner
    {
    private:
        /* data */
    public:
        TextureFusioner(/* args */);
        ~TextureFusioner();

        void process(SLAM_LYJ::BaseTriMesh& _btm,
            const std::vector<COMMON_LYJ::CompressedImage>& _imgs,
            const std::vector<SLAM_LYJ::Pose3D>& _Tcws,
            const std::vector<SLAM_LYJ::PinholeCamera>& _cams,
            TextureFusionOption _opt
        );

    private:
        void generateBlock(SLAM_LYJ::BaseTriMesh& _btm,
            const std::vector<const COMMON_LYJ::CompressedImage*>& _imgs,
            const std::vector<SLAM_LYJ::Pose3D>& _Tcws,
            const std::vector<SLAM_LYJ::PinholeCamera>& _cams,
            const std::vector<int>& _fs2Tex,
            std::vector<TextureBlock>& _texBlocks
        );
    };
    

        
}


#endif