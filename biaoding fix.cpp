#include<iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/geometry.hpp>
#include<string>
#include<utility>
#include<cmath>

const cv::Size ban = {11,8};
const float bian = 50;
//给的参数

std::vector<std::vector<cv::Point2f>> photojiao;
std::vector<std::vector<cv::Point3f>> worldjiao;
std::vector<cv::Point3f> wj;

void worldjiaochushihua(cv::Size ban,float bian)
{
    
    for (int j=0;j<ban.height;j++)
        for (int i=0;i<ban.width;i++)
        {
            wj.emplace_back(bian*i,bian*j,0.f);

        }

};

int main(){
    cv::Mat neican,jibian,jiancetu,jctG;
    std::vector<cv::Mat> R,T;

    cv::String path = "D:/Download/calibration_images/calibration_images/image_000*.png";
    std::vector<cv::String> photos;
    cv::glob(path, photos);
    if (photos.empty())
    {
        std::cerr<<"mei you zhao dao tu pian: "<<path<<std::endl;
        return -1;
    }

    worldjiaochushihua(ban,bian);

    cv::Size imageSize;
    for(size_t i=0;i<photos.size();i++)
    {   
        std::vector<cv::Point2f> pj;
        jiancetu = cv::imread(photos[i]);
        if (jiancetu.empty())
        {
            std::cerr<<"du qu shi bai: "<<photos[i]<<std::endl;
            continue;
        }
        std::cout<<photos[i]<<std::endl;
        if (imageSize.empty()) imageSize = jiancetu.size();
        cv::cvtColor(jiancetu,jctG,cv::COLOR_BGR2GRAY);
        //转灰度
        
        bool yes = cv::findChessboardCorners(jctG,ban,pj);
        if (!yes)
        {
            std::cerr<<"zhao bu dao jiao dian, tiao guo: "<<photos[i]<<std::endl;
            continue;
        }
        cv::TermCriteria criteria = cv::TermCriteria(
                    cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,
                    40,
                    0.01);
        cv::cornerSubPix(jctG, pj, cv::Size(11,11), cv::Size(-1, -1), criteria);
        cv::drawChessboardCorners(jiancetu,ban,pj,yes);
        //找并且亚像素优化，画角点

        worldjiao.emplace_back(wj);
        photojiao.emplace_back(pj);
        
        cv::imshow("check",jiancetu);
        cv::waitKey(1);

    }

    if (worldjiao.size() < 3)
    {
        std::cerr<<"you xiao tu pian tai shao: "<<worldjiao.size()<<std::endl;
        return -1;
    }
    
    double error = cv::calibrateCamera(worldjiao, photojiao, imageSize, neican, jibian, R, T);
    std::cout<<"chongtouyingwucha(px):"<<error<<std::endl;
    std::cout<<"neicanjuzhen:"<<neican<<std::endl;
    std::cout<<"jibianxishu:"<<jibian<<std::endl;

    return 0;
}