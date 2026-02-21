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
        std::vector<double> overlapsV(imgSz, 0);
        SLAM_LYJ::BitFlagVec fVisible(fSz);
        std::set<int> selectedImgInds;
        auto functtt = [&](uint64_t _s, uint64_t _e, uint32_t _id)
            {
                for (int i = _s; i < _e; ++i)
                {
                    if (selectedImgInds.count(i))
                        continue;
                    int cnt = 0;
                    for (size_t j = 0; j < fSz; ++j)
                    {
                        /* code */
                        if (!fVisible[j] && _imgs2fs[i][j])
                            ++cnt;
                    }
                    overlapsV[i] = double(cnt) / double(fSz);
                }
            };
        auto funcFindNext = [&]()->int
        {
            overlaps.clear();
            overlapsV.assign(imgSz, 0);
            const auto& dPtr = fVisible.data();

            if (opt_.threadNum != 1)
            {
                SLAM_LYJ::SLAM_LYJ_MATH::ThreadPool threadPool(opt_.threadNum);
                threadPool.processWithId(functtt, 0, imgSz);
            }
            else
                functtt(0, imgSz, 0);

            for (int i = 0; i < imgSz; ++i)
            {
                if (overlapsV[i] == 0)
                    continue;
                overlaps[overlapsV[i]] = i;
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