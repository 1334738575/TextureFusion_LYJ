#include <iostream>
#include <TextureFusion_LYJ_Defines.h>
#include <TextureFusion_LYJ_Include.h>
#include <IO/ColmapIO.h>
#include <IO/MeshIO.h>
#include <common/Timer.h>


void testTextureFusion()
{
    COMMON_LYJ::Timer t;
    std::string pth = "D:/gsWin/gaussian-splatting/gaussian-splatting/data/mask/sparse/1";
    COMMON_LYJ::ColmapData colmapData;
    colmapData.readFromColmap(pth);
    double readColmapTime = t.elapsed(false);
    std::cout << "read colmap time: " << readColmapTime << "ms" << std::endl;
    std::vector<COMMON_LYJ::ColmapImage>& colmapImages = colmapData.images_;
    std::vector<COMMON_LYJ::ColmapCamera>& colmapCameras = colmapData.cameras_;
    std::string imgDir = "D:/gsWin/gaussian-splatting/gaussian-splatting/data/mask/images/";
    std::string btmPath = "D:/SLAM_LYJ_Packages/SLAM_LYJ_qt/tmp/fuse_unbounded_post.ply";

    int imgSz = colmapImages.size();
    // imgSz = 100;
    COMMON_LYJ::BaseTriMesh btm;
    std::vector<COMMON_LYJ::Pose3D> Tcws(imgSz);
    std::vector<COMMON_LYJ::PinholeCamera> cams(imgSz);
    std::vector<COMMON_LYJ::CompressedImage> comImgs(imgSz);
    COMMON_LYJ::readPLYMesh(btmPath, btm);
    double readMeshTime = t.elapsed(false);
    std::cout << "read mesh time: " << readMeshTime << "ms" << std::endl;
    auto& colmapCamera = colmapCameras[0];
    for (int i = 0; i < imgSz; ++i)
    {
        const auto& colmapImage = colmapImages[i];
        std::string imgName = imgDir + colmapImage.imgName;
        //std::cout << imgName << std::endl;
        comImgs[i].readJPG(imgName);
        cams[i] = COMMON_LYJ::PinholeCamera(colmapCamera.width, colmapCamera.height, colmapCamera.params);
        Eigen::Matrix3d Rcw = colmapImage.qcw.toRotationMatrix();
        Tcws[i].setR(Rcw);
        Tcws[i].sett(colmapImage.tcw);
    }
    double readImageTime = t.elapsed(false);
    std::cout << "read image time: " << readImageTime << "ms" << std::endl;
    TextureFusion_LYJ::TextureFusionOption tfOpt;
    //tfOpt.threadNum = 1;//common_lyj，里面有bug，线程数多于任务数会死循环TODO
    TextureFusion_LYJ::texture_fusion(btm, comImgs, Tcws, cams, tfOpt);
    double textueFusionTime = t.elapsed(false);
    std::cout << "texture fusion time: " << textueFusionTime << "ms" << std::endl;
    COMMON_LYJ::writeOBJMesh("D:/tmp/tf.obj", btm);
    double writeObjTime = t.elapsed(false);
    std::cout << "write obj time: " << writeObjTime << "ms" << std::endl;
    return;
}

        
int main()
{
        
    TextureFusion_LYJ::print_TextureFusion_LYJ_Test();
    testTextureFusion();
}