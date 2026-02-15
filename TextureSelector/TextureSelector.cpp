#include "TextureSelector.h"


namespace TextureFusion_LYJ
{
    
    
    TextureSelector::TextureSelector(/* args */)
    {}
    TextureSelector::~TextureSelector()
    {}

    void TextureSelector::selectTexture(
        SLAM_LYJ::BaseTriMesh& _btm,
        const std::vector<const COMMON_LYJ::CompressedImage*>& _imgs,
        const std::vector<SLAM_LYJ::Pose3D>& _Tcws,
        const std::vector<SLAM_LYJ::PinholeCamera>& _cams,
        const std::vector<SLAM_LYJ::BitFlagVec>& _imgs2fs,
        std::vector<int>& _fs2Tex,
        TextureSelectorOption _opt
    )
    {
        btm_ = &_btm;
        imgs_ = _imgs;
        Tcws_ = _Tcws;
        cams_ = _cams;
        imgs2fs_ = _imgs2fs;
        opt_ = _opt;
        _fs2Tex.clear();
        int imgSz = _imgs.size();
        int fSz = btm_->getFn();

        _fs2Tex.assign(fSz, -1);
        std::map<double, int> overlaps;
        SLAM_LYJ::BitFlagVec fVisible(fSz);
        std::set<int> selectedImgInds;
        auto funcFindNext = [&]()->int
        {
            overlaps.clear();
            const auto& dPtr = fVisible.data();
            for(int i=0;i<imgSz;++i)
            {
                if(selectedImgInds.count(i))
                    continue;
                int cnt = 0;
                for (size_t j = 0; j < fSz; ++j)
                {
                    /* code */
                    if(!fVisible[j] && _imgs2fs[i][j])
                        ++cnt;
                }
                overlaps[double(cnt)/double(fSz)] = i;
            }
            if(overlaps.empty() || (overlaps.rbegin()->first <= 0))
                return -1;
            return overlaps.rbegin()->second;
        };
        
        while(1)
        {
            int imgInd = funcFindNext();
            if(imgInd == -1)
                break;
            selectedImgInds.insert(imgInd);
            for (size_t i = 0; i < fSz; ++i)
            {
                /* code */
                if(_imgs2fs[imgInd][i] && !fVisible[i])
                {
                    fVisible.setFlag(i, true);
                    _fs2Tex[i] = imgInd;
                }
            }
        }

        return;
    }
}