#include<iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/geometry.hpp>
#include<string>
#include<utility>
#include<cmath>

const cv::Size ban = {11,8};
const float bian = 50e-3;
//给的参数

std::vector<std::vector<cv::Point2f>> photojiao;
std::vector<std::vector<cv::Point3f>> worldjiao;
std::vector<cv::Point3f> wj;

void worldjiaochushihua(cv::Size ban,float bian)
{
    for (int i=0;i<ban.width;i++)
        for (int j=0;j<ban.height;j++)
        {
            wj.emplace_back(bian*i,bian*j,0.f);

        }

};

int main(){
    cv::Mat neican,jibian,R,T,jiancetu,jctG;


    cv::String path = "D:/Download/calibration_images/calibration_images/image_000*.png";
    std::vector<cv::String> photos;
    cv::glob(path, photos);

    worldjiaochushihua(ban,bian);

    for(int i=0;i<std::size(photos);i++)
    {   
        std::vector<cv::Point2f> pj;
        jiancetu = cv::imread(photos[i]);
        std::cout<<photos[i]<<std::endl;
        cv::cvtColor(jiancetu,jctG,cv::COLOR_BGR2GRAY);
        //转灰度
        
        bool yes = cv::findChessboardCorners(jctG,ban,pj);
        /*cv::TermCriteria criteria = cv::TermCriteria(
                    cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,
                    40,
                    0.01);
        cv::cornerSubPix(jctG, pj, ban, cv::Size(-1, -1), criteria);*/
        cv::drawChessboardCorners(jiancetu,ban,pj,yes);
        //找并且亚像素优化，画角点

        worldjiao.emplace_back(wj);
        photojiao.emplace_back(pj);
        
        cv::imshow("check",jiancetu);
        cv::waitKey(1);

    }
    
    double error = cv::calibrateCamera(worldjiao, photojiao, cv::Size(jctG.rows, jctG.cols), neican, jibian, R, T);
    std::cout<<"chongtouyingwucha:"<<error/1e3<<std::endl;
    std::cout<<"neicanjuzhen:"<<neican<<std::endl;
    std::cout<<"jibianxishu:"<<jibian<<std::endl;

    return 0;
}