#ifndef TEXTUREFUSION_LYJ_INCLUDE_H
#define TEXTUREFUSION_LYJ_INCLUDE_H


#include "TextureFusion_LYJ_Defines.h"
#include <stdio.h>



namespace TextureFusion_LYJ
{


TEXTUREFUSION_LYJ_API void print_TextureFusion_LYJ_Test();

TEXTUREFUSION_LYJ_API void texture_fusion(
    COMMON_LYJ::BaseTriMesh& _btm,
    const std::vector<COMMON_LYJ::CompressedImage>& _imgs,
    const std::vector<COMMON_LYJ::Pose3D>& _Tcws,
    const std::vector<COMMON_LYJ::PinholeCamera>& _cams,
    TextureFusionOption _opt
);

}

#endif//TextureFusion_LYJ_INCLUDE_H