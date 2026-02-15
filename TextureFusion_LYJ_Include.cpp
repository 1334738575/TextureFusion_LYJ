#include "TextureFusion_LYJ_Include.h"
#include "TextureFusioner/TextureFusioner.h"

namespace TextureFusion_LYJ
{


TEXTUREFUSION_LYJ_API void print_TextureFusion_LYJ_Test()
{
    printf("Hello TextureFusion_LYJ!");
}


TEXTUREFUSION_LYJ_API void texture_fusion(
    SLAM_LYJ::BaseTriMesh& _btm,
    const std::vector<COMMON_LYJ::CompressedImage>& _imgs,
    const std::vector<SLAM_LYJ::Pose3D>& _Tcws,
    const std::vector<SLAM_LYJ::PinholeCamera>& _cams,
    TextureFusionOption _opt
)
{
    TextureFusioner textureFusioner;
    textureFusioner.process(_btm, _imgs, _Tcws, _cams, _opt);
}


}