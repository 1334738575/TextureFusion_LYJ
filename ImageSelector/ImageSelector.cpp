#include "ImageSelector.h"

namespace TextureFusion_LYJ
{
    
    
    ImageSelector::ImageSelector(/* args */)
    {}
    ImageSelector::~ImageSelector()
    {}


    void ImageSelector::selectImages(COMMON_LYJ::BaseTriMesh& _btm,
        const std::vector<COMMON_LYJ::Pose3D>& _Tcws,
        const std::vector<COMMON_LYJ::PinholeCamera>& _cams,
        std::vector<COMMON_LYJ::BitFlagVec>& _imgs2fs,
        std::vector<int>& _imgInds,
        ImageSelectorOption _opt
    )
    {
        btm_ = &_btm;
        Tcws_ = _Tcws;
        cams_ = _cams;
        opt_ = _opt;
        _imgs2fs.clear();
        _imgInds.clear();

        if(!project(_imgs2fs))
            return;

        if(!select(_imgs2fs, _imgInds))
            return;
    }

    
    bool ImageSelector::project(std::vector<COMMON_LYJ::BitFlagVec>& _imgs2fs)
    {
        //prepare
        CUDA_LYJ::ProHandle proHandle_=nullptr;
        std::vector<CUDA_LYJ::ProjectorCache> proCaches;
        std::vector<ProBuffer> proBuffers;
        int threadNum = opt_.threadNum;
        if(threadNum <= 0)
            threadNum = std::thread::hardware_concurrency();
        int imgSz = Tcws_.size();
        int fSz = btm_->getFn();
        const auto& faces = btm_->getFaces();
        
        //init
        _imgs2fs.resize(imgSz);
        for(int i=0;i<imgSz;++i)
        {
            _imgs2fs[i].assign(fSz, false);
        }
        btm_->enableFCenters();
        btm_->calculateFCenters();
        btm_->enableFNormals();
        btm_->calculateFNormals();
        std::vector<float> camParams(4);
        camParams[0] = cams_[0].fx();
        camParams[1] = cams_[0].fy();
        camParams[2] = cams_[0].cx();
        camParams[3] = cams_[0].cy();
        int w = cams_[0].wide();
        int h = cams_[0].height();
        proHandle_ = CUDA_LYJ::initProjector(btm_->getVertexs()[0].data(), btm_->getVn(), btm_->getFCenters()[0].data(), btm_->getFNormals()[0].data(), btm_->getFaces()[0].vId_, btm_->getFn(), camParams.data(), cams_[0].wide(), cams_[0].height());
        proCaches.resize(threadNum);
        for(int i=0;i< threadNum;++i)
            proCaches[i].init(btm_->getVn(), btm_->getFn(), w, h);
        proBuffers.resize(threadNum);
        for (int i = 0; i < threadNum; ++i)
            //proBuffers[i].init(w, h, btm_);
            proBuffers[i].init(w, h, btm_, 0.01, FLT_MAX, -0.5);

        //project
        auto funcProject = [&](uint64_t _s, uint64_t _e, uint32_t _id) {
            for (int i = _s; i < _e; ++i)
            {
                //COMMON_LYJ::Timer q;
                proBuffers[_id].updateTcw(Tcws_[i]);
                CUDA_LYJ::project(proHandle_, proCaches[_id], proBuffers[_id].Tcw.data(), (float*)proBuffers[_id].depthsM.data, proBuffers[_id].fIds.data(), proBuffers[_id].allVisiblePIds.data(), proBuffers[_id].allVisibleFIds.data(), proBuffers[_id].minD, proBuffers[_id].maxD, proBuffers[_id].csTh, proBuffers[_id].detDTh);
                //double t = q.elapsed();
                //std::cout << "cost time: " << t << " ms" << std::endl;
                for (int j = 0; j < fSz; ++j)
                {
                    const auto& vIds = faces[j].vId_;
                    const auto& pValids = proBuffers[_id].allVisiblePIds;
                    if (proBuffers[_id].allVisibleFIds[j] == 1
                        && pValids[vIds[0]] == 1
                        && pValids[vIds[1]] == 1
                        && pValids[vIds[2]] == 1
                    )
                        _imgs2fs[i].setFlag(j, true);
                }

                if(false)
                {
                    //cv::Mat depthsShow(h, w, CV_8UC1);
                    //depthsShow.setTo(0);
                    //for (int r = 0; r < h; ++r)
                    //{
                	   // for (int c = 0; c < w; ++c)
                	   // {
                		  //  float d = proBuffers[_id].depthsM.at<float>(r, c);
                		  //  if (d == FLT_MAX)
                			 //   continue;
                		  //  int di = d * 10;
                		  //  depthsShow.at<uchar>(r, c) = (uchar)di;
                	   // }
                    //}
                    //cv::pyrDown(depthsShow, depthsShow);
                    //cv::pyrDown(depthsShow, depthsShow);
                    //cv::imshow("depth", depthsShow);
                    //cv::waitKey();

                    const auto& cs = btm_->getFCenters();
                    const auto& ps = btm_->getVertexs();
                    int pSz = ps.size();
                    std::vector<Eigen::Vector3f> ps1;
                    std::vector<Eigen::Vector3f> ps2;
                    const auto& pValids = proBuffers[_id].allVisiblePIds;
                    for (int j = 0; j < fSz; ++j)
                    {
                        const auto& vIds = faces[j].vId_;
                        if (proBuffers[_id].allVisibleFIds[j] == 1
                            //&& pValids[vIds[0]] == 1
                            //&& pValids[vIds[1]] == 1
                            //&& pValids[vIds[2]] == 1
                            )
                        {
                            ps1.push_back(cs[j]);
                        }
                    }
                    for (int j = 0; j < pSz; ++j)
                    {
                        if (pValids[j] == 1)
                            ps2.push_back(ps[j]);
                    }
                    COMMON_LYJ::BaseTriMesh btmTmp1;
                    btmTmp1.setVertexs(ps1);
                    COMMON_LYJ::writePLYMesh("D:/tmp/tmp1.ply", btmTmp1);
                    COMMON_LYJ::BaseTriMesh btmTmp2;
                    btmTmp2.setVertexs(ps2);
                    COMMON_LYJ::writePLYMesh("D:/tmp/tmp2.ply", btmTmp2);
                }
                continue;
            }
        };
        COMMON_LYJ::ThreadPool threadPool(threadNum);
        threadPool.processWithId(funcProject, 0, imgSz);

        //release
        if(proHandle_)
            CUDA_LYJ::release(proHandle_);
        return true;
    }


    bool ImageSelector::select(const std::vector<COMMON_LYJ::BitFlagVec>& _imgs2fs,
        std::vector<int>& _imgInds)
    {
        int imgSz = Tcws_.size();
        int fSz = btm_->getFn();

        std::map<double, int> overlaps;
        std::vector<double> overlapsV(imgSz, 0);
        COMMON_LYJ::BitFlagVec fVisible(fSz);
        int dSz = fVisible.size();
        dSz >>= 3;
        dSz += 1;
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
                COMMON_LYJ::ThreadPool threadPool(opt_.threadNum);
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
            if (overlaps.empty())
                return -1;
            double ol = overlaps.rbegin()->first;
            if(ol <= 0)
                return -1;
            int ind = overlaps.rbegin()->second;
            return ind;
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
                if(_imgs2fs[imgInd][i])
                    fVisible.setFlag(i, true);
            }
        }

        if(selectedImgInds.empty())
            return false;
        _imgInds.reserve(selectedImgInds.size());
        for(const auto& _:selectedImgInds)
            _imgInds.push_back(_);
        return true;
    }
}