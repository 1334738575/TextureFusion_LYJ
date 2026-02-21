#include "TextureFusioner.h"
#include <common/Timer.h>
#include <IO/ColmapIO.h>


namespace TextureFusion_LYJ
{
    
    
    TextureFusioner::TextureFusioner(/* args */)
    {}
    TextureFusioner::~TextureFusioner()
    {}

    void TextureFusioner::process(SLAM_LYJ::BaseTriMesh& _btm,
        const std::vector<COMMON_LYJ::CompressedImage>& _imgs,
        const std::vector<SLAM_LYJ::Pose3D>& _Tcws,
        const std::vector<SLAM_LYJ::PinholeCamera>& _cams,
        TextureFusionOption _opt
    )
    {
        SLAM_LYJ::Timer t;
        int fSz = _btm.getFn();

        //select image
        std::vector<SLAM_LYJ::BitFlagVec> imgs2fs;
        std::vector<int> imgInds;
        ImageSelectorOption imgSelectOpt;
        imgSelectOpt.threadNum = _opt.threadNum;
        ImageSelector imgSelector;
        imgSelector.selectImages(_btm, _Tcws, _cams, imgs2fs, imgInds, imgSelectOpt);
        if(imgs2fs.empty() || imgInds.empty())
        {
            std::cout << "select image error!" << std::endl;
            return;
        }
        double selcetImageTime = t.elapsed(true);
        std::cout << "select images time: " << selcetImageTime << "ms" << std::endl;


        //select texture
        int selectedSz = imgInds.size();
        std::vector<const COMMON_LYJ::CompressedImage*> selectedImgs(selectedSz, nullptr);
        std::vector<SLAM_LYJ::Pose3D> selectedTcws(selectedSz);
        std::vector<SLAM_LYJ::PinholeCamera> selectedCams(selectedSz);
        std::vector<SLAM_LYJ::BitFlagVec> selectedImgs2fs(selectedSz);
        for(int i=0;i<selectedSz;++i)
        {
            selectedImgs[i] = &_imgs[imgInds[i]];
            selectedTcws[i] = _Tcws[imgInds[i]];
            const auto& cam = _cams[imgInds[i]];
            selectedCams[i] = cam;
            selectedImgs2fs[i] = imgs2fs[imgInds[i]];
        }
        imgs2fs.clear();
        TextureSelectorOption texSelectorOpt;
        texSelectorOpt.threadNum = _opt.threadNum;
        std::vector<int> fs2Tex;//imgid is after select order
        TextureSelector texSelector;
        texSelector.selectTexture(_btm, selectedImgs, selectedTcws, selectedCams, selectedImgs2fs, fs2Tex, texSelectorOpt);
        double selcetTextureTime = t.elapsed(true);
        std::cout << "select texture time: " << selcetTextureTime << "ms" << std::endl;
        
        //block, maybe remove after
        TextureBlock defaultBlock;
        defaultBlock.w = 50;
        defaultBlock.h = 50;
        defaultBlock.leftup = Eigen::Vector2i(0,0);
        defaultBlock.rightdown = Eigen::Vector2i(49,49);
        cv::Mat defaultTexture = cv::Mat::zeros(defaultBlock.h,defaultBlock.w,CV_8UC3);
        std::vector<TextureBlock> texBlocks;
        generateBlock(_btm, selectedImgs, selectedTcws, selectedCams, fs2Tex, texBlocks);
        texBlocks.insert(texBlocks.begin(), defaultBlock);
        std::vector<int> fs2bs(fSz, 0);
        int bSz = texBlocks.size();
        for(int i=0;i<bSz;++i)
        {
            const auto& fsInB = texBlocks[i].fids;
            for(int j=0;j<fsInB.size();++j)
            {
                fs2bs[fsInB[j]] = i;
            }
        }
        for(int i=0;i<fSz;++i)
        {
            if(fs2bs[i] == 0)
                texBlocks[0].fids.push_back(i);
        }
        double generateBlockTime = t.elapsed(true);
        std::cout << "generate block time: " << generateBlockTime << "ms" << std::endl;
        
        //obj
        std::vector<Eigen::Vector2f> uvs;
        std::vector<SLAM_LYJ::BaseTriTextureUV> triUVs;
        COMMON_LYJ::CompressedImage comImg;
        //final image
        int totalW = 0;
        int totalH = 0;
        int maxW = 0;
        int maxH = 0;
        int maxBId = -1;
        for(int i=0;i<bSz;++i)
        {
            if(texBlocks[i].w > maxW)
            {
                maxW = texBlocks[i].w;
                maxBId = i;
            }
            if(texBlocks[i].h > maxH)
            {
                maxH = texBlocks[i].h;
            }
            totalH += texBlocks[i].h;
        }
        int priorOutH = sqrt(selectedSz) * maxH;
        //std::cout << "prior out H: " << priorOutH << std::endl;
        int curC=0;
        int curR=0;
        int outW = 0;
        int outH = 0;
        for(int i=0;i<bSz;++i)
        {
            texBlocks[i].leftupFinal(0) = curC;
            texBlocks[i].leftupFinal(1) = curR;
            if(curR == 0)
                outW = curC + maxW;
            curR += texBlocks[i].h;
            if(curR > priorOutH)
            {
                curC += maxW;
                if(curR > outH)
                    outH = curR;
                curR = 0;
            }
            if(texBlocks[i].h < 0 || curR < 0)
                std::cout << "error h!" << std::endl;
        }
        cv::Mat finalMat = cv::Mat::zeros(outH, outW, CV_8UC3);
        std::vector<cv::Mat> cvImgs(selectedSz);
        for(int i=0;i<selectedSz;++i)
        {
            selectedImgs[i]->decompressCVMat(cvImgs[i]);
        }
        for(int i=0;i<bSz;++i)
        {
            cv::Rect rectSrc(texBlocks[i].leftup(0),texBlocks[i].leftup(1),texBlocks[i].w, texBlocks[i].h);
            cv::Rect rectDst(texBlocks[i].leftupFinal(0),texBlocks[i].leftupFinal(1),texBlocks[i].w, texBlocks[i].h);
            int imgId = texBlocks[i].imgId;
            cv::Mat mmm;
            if(imgId == -1)
                mmm = defaultTexture;
            else
                mmm = cvImgs[imgId](rectSrc);
            mmm.copyTo(finalMat(rectDst));
        }
        comImg.compressCVMat(finalMat);
        //cv::imwrite("D:/tmp/tf.jpg", finalMat);
        double generateImageTime = t.elapsed(true);
        std::cout << "generate image time: " << generateImageTime << "ms" << std::endl;

        //uvs, simple
        const auto& ps = _btm.getVertexs();
        const auto& fs = _btm.getFaces();
        uvs.assign(fSz * 3, Eigen::Vector2f(-1, -1));
        triUVs.resize(fSz);
        for(int i=0;i<fSz;++i)
        {
            const int& imgId = fs2Tex[i];
            const auto& bIds = fs2bs[i];
            const auto& vIds = fs[i].vId_;
            int ii = i * 3;
            if(imgId == -1)
            {
                uvs[3 * i + 0] = texBlocks[bIds].leftupFinal.cast<float>() + Eigen::Vector2f(0,0);
                uvs[3 * i + 1] = texBlocks[bIds].leftupFinal.cast<float>() + Eigen::Vector2f(0,40);
                uvs[3 * i + 2] = texBlocks[bIds].leftupFinal.cast<float>() + Eigen::Vector2f(40,0);
                for(int j=0;j<3;++j)
                {
                    uvs[3 * i + j](0) /= outW;
                    uvs[3 * i + j](1) = 1 - uvs[3 * i + j](1) / outH;
                    triUVs[i].uvId_[j] = ii + j;
                }
                continue;
            }
            const auto& Tcw = selectedTcws[imgId];
            const auto& cam = selectedCams[imgId];
            if(bIds == -1)
                continue;
            Eigen::Vector2f uv;
            for(int j=0;j<3;++j)
            {
                const auto& Pw = ps[vIds[j]];
                auto Pc = Tcw * Pw;
                cam.world2Image(Pc, uv);
                uvs[3 * i + j] = texBlocks[bIds].leftupFinal.cast<float>() + uv - texBlocks[bIds].leftup.cast<float>();
                uvs[3 * i + j](0) /= outW;
                uvs[3 * i + j](1) = 1 - uvs[3 * i + j](1) / outH;
                triUVs[i].uvId_[j] = ii + j;
            }
        }
        double generateUVTime = t.elapsed(true);
        std::cout << "generate UV time: " << generateUVTime << "ms" << std::endl;
        
        //TODO
        _btm.enableTexture();
        _btm.setTexture(uvs, triUVs, comImg);
    }

        

    void TextureFusioner::generateBlock(SLAM_LYJ::BaseTriMesh& _btm,
        const std::vector<const COMMON_LYJ::CompressedImage*>& _imgs,
        const std::vector<SLAM_LYJ::Pose3D>& _Tcws,
        const std::vector<SLAM_LYJ::PinholeCamera>& _cams,
        const std::vector<int>& _fs2Tex,
        std::vector<TextureBlock>& _texBlocks
    )
    {
        _texBlocks.clear();
        std::map<int, TextureBlock> mTexBlocks;
        int fSz = _fs2Tex.size();
        const auto& ps = _btm.getVertexs();
        const auto& fs = _btm.getFaces();
        for(int i=0;i<fSz;++i)
        {
            if(_fs2Tex[i] == -1)
                continue;
            const int imgId = _fs2Tex[i];
            const auto& Tcw = _Tcws[imgId];
            const auto& cam = _cams[imgId];
            const auto& vIds = fs[i].vId_;

            mTexBlocks[_fs2Tex[i]].fids.push_back(i);
            auto& bbb = mTexBlocks[_fs2Tex[i]];
            if(bbb.imgId == -1)
                bbb.imgId = _fs2Tex[i];
            
            Eigen::Vector2f uv;
            int ii = i * 3;
            for(int j=0;j<3;++j)
            {
                const auto& Pw = ps[vIds[j]];
                auto Pc = Tcw * Pw;
                cam.world2Image(Pc, uv);
                if(uv(0) > bbb.rightdown(0))
                    bbb.rightdown(0) = uv(0) + 1;
                if( uv(0) < bbb.leftup(0))
                    bbb.leftup(0) = uv(0) - 1;
                if(uv(1) > bbb.rightdown(1))
                    bbb.rightdown(1) = uv(1) + 1;
                if(uv(1) < bbb.leftup(1))
                    bbb.leftup(1) = uv(1) - 1;
            }
        }
        
        _texBlocks.reserve(mTexBlocks.size());
        for(const auto& ttt:mTexBlocks)
        {
            _texBlocks.push_back(ttt.second);
            _texBlocks.back().leftup(0) -= 1;
            _texBlocks.back().leftup(1) -= 1;
            _texBlocks.back().rightdown(0) += 2;
            _texBlocks.back().rightdown(1) += 2;
            if(_texBlocks.back().leftup(0) < 0)
                _texBlocks.back().leftup(0) = 0;
            if(_texBlocks.back().leftup(1) < 0)
                _texBlocks.back().leftup(0) = 0;
            if(_texBlocks.back().rightdown(0) >= _cams[ttt.first].wide())
                _texBlocks.back().rightdown(0) = _cams[ttt.first].wide()-1;
            if(_texBlocks.back().rightdown(1) >= _cams[ttt.first].height())
                _texBlocks.back().rightdown(1) = _cams[ttt.first].height()-1;
            _texBlocks.back().w = _texBlocks.back().rightdown(0) - _texBlocks.back().leftup(0) + 1;
            _texBlocks.back().h = _texBlocks.back().rightdown(1) - _texBlocks.back().leftup(1) + 1;
        }
    }


}