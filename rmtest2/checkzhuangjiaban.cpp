#include<iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/geometry.hpp>
#include<opencv2/video.hpp>
#include<string>
#include<utility>
#include<cmath>
#include<algorithm>




const cv::Size ban = {11,8};
const float bian = 50.0e-3;
//给的参数【标定】

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




//给定装甲板【识别】
constexpr float SMALL_ARMOR_WIDTH = 67.5f;
constexpr float LIGHTBAR_LENGTH = 27.5f;

const std::vector<cv::Point3f> SMALL_ARMOR_POINTS = {
 {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};

class Arrmor
{
    public:
    cv::Point2f jiao1;
    cv::Point2f jiao2;
    cv::Point2f jiao3;
    cv::Point2f jiao4;
    cv::Point2f cpoint;
    int nmuber;
    
    //取矩形的上边中点tp与下边中点bm
    static void getMid(const cv::Rect& rect, cv::Point2f& tp, cv::Point2f& bm)
    {
        tp.x = rect.x + rect.width/2.0f;
        tp.y = rect.y;
        bm.x = tp.x;
        bm.y = rect.y + rect.height;
    }

    void shezhiarrmor(cv::Rect a1,cv::Rect a2)
    {
        //按中心点x坐标区分左右两个矩形
        cv::Rect left = a1;
        cv::Rect right = a2;
        if(left.x + left.width/2.0 > right.x + right.width/2.0)
            std::swap(left, right);

        cv::Point2f ltop, lbottom, rtop, rbottom;
        getMid(left, ltop, lbottom);
        getMid(right, rtop, rbottom);

        jiao1 = ltop;
        jiao2 = rtop;
        jiao3 = rbottom;
        jiao4 = lbottom;

        double lcx = left.x + left.width/2.0;
        double rcx = right.x + right.width/2.0;
        double lcy = left.y + left.height/2.0;
        double rcy = right.y + right.height/2.0;
        cpoint.x = (lcx + rcx) / 2;
        cpoint.y = (lcy + rcy) / 2;
    }

};

void saimianji(const std::vector<std::vector<cv::Point>>& cs, double b,double c, std::vector<std::vector<cv::Point>>& d){
    d.clear();
    for(const std::vector<cv::Point>& contour : cs){
        if(cv::contourArea(contour) >= b&&cv::contourArea(contour) <= c){
            d.push_back(contour);
        }
    }
}

//筛选旋转矩形长短边的比值
std::vector<std::vector<cv::Point>> saibizhi(const std::vector<std::vector<cv::Point>>& cs, double low, double high){
    std::vector<std::vector<cv::Point>> c;
    for(const std::vector<cv::Point>& contour : cs){
        cv::RotatedRect rr = cv::minAreaRect(contour);
        double chang = std::max(rr.size.width, rr.size.height);
        double duan = std::min(rr.size.width, rr.size.height);
        double bili = chang / duan;
        if(bili >= low && bili <= high){
            c.push_back(contour);
        }
    }
    return c;
}

/*cv::Mat neican = (cv::Mat_<double>(3, 3) << 151.3066, 0, 511.4999999999929,
    0, 457.2498322824845, 639.5000000000018,
    0, 0, 1);
cv::Mat jibian = (cv::Mat_<double>(3, 2) << -0.0005124, -7.024551415981683e-08, 2.200896141412788e-07,
    -3.062372331788878e-06, 6.699174006374956e-12, 0);*/


int main(){



//标定
    cv::Mat neican,jibian,R,T,jiancetu,jctG;


    cv::String path = "D:/Download/calibration_images/calibration_images/image_000*.png";
    std::vector<cv::String> photos;
    cv::glob(path, photos);

    worldjiaochushihua(ban,bian);

    for(int i=0;i<std::size(photos);i++)
    {   
        std::vector<cv::Point2f> pj;

        jiancetu = cv::imread(photos[i]);
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
        

    }
    
    double error = cv::calibrateCamera(worldjiao, photojiao, cv::Size(jctG.rows, jctG.cols), neican, jibian, R, T);



//开始识别
double time1 = static_cast<double>(cv::getTickCount());//开始计时第一次

    cv::VideoCapture cap("D:/Download/testvedio.avi");



    while(true)
    {
    double time1 = static_cast<double>(cv::getTickCount());//开始计时
    cv::Mat image;
    cap.read(image);

    int wimg  = image.cols;
    int himg = image.rows;

    cv::Mat imagegau;
    cv::GaussianBlur(image,imagegau,cv::Size(5,5),0);
    cv::Mat imagehsv;
    cv::cvtColor(imagegau,imagehsv,cv::COLOR_BGR2HSV);
    //cv::cvtColor(image,imagehsv,cv::COLOR_BGR2HSV);

    cv::Scalar bluelower(80, 80,70);
    cv::Scalar blueupper(100, 255,255);
    cv::Mat mask;
    cv::inRange(imagehsv, bluelower, blueupper, mask);
    //掩膜，二值化

    std::vector<std::vector<cv::Point>> cs;
    cv::findContours(mask, cs, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    //查找轮廓

    std::vector<std::vector<cv::Point>> acs;
    saimianji(cs, 30, 200 ,acs);
    //过滤无关小轮廓

    acs = saibizhi(acs, 3, 10);
    //筛选合适形态

    cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);
    for(int i=0;i<acs.size();i++)
    {
        cv::Rect rect = cv::boundingRect(acs[i]);
        cv::rectangle(mask, rect, cv::Scalar(0, 255, 0), 3);
    }
    //外接矩形

    std::vector<Arrmor> arrmors;
    for(int i=0;i<acs.size();i++)
    {
        cv::Rect rect1 = cv::boundingRect(acs[i]);
        for(int j=0;j<acs.size();j++)
        {
            if(i==j) continue;//两两配对
                cv::Rect rect2 = cv::boundingRect(acs[j]);

                if(rect1.height > 1.5*rect1.width && rect2.height > 1.5*rect2.width)//灯条再筛选一遍长宽
                {
                    double chang1 = std::max(rect1.width, rect1.height);
                    double chang2 = std::max(rect2.width, rect2.height);
                    if(abs(chang1 - chang2)<chang1&&abs(chang1 - chang2)<chang2)//两灯条长度差不能太大
                    {
                        double c1x = rect1.x + rect1.width/2.0;
                        double c1y = rect1.y + rect1.height/2.0;
                        double c2x = rect2.x + rect2.width/2.0;
                        double c2y = rect2.y + rect2.height/2.0;
                        double d1 = fabs(c2y - c1y);
                        double d2 = fabs(c1y - c2y);
                        //筛选接近平行的灯条
                        double duan1 = std::min(rect1.width, rect1.height);
                        double duan2 = std::min(rect2.width, rect2.height);

                        double jianju = sqrt(pow(c1x - c2x, 2) + pow(c1y - c2y, 2));
                        double bili1 = jianju / chang1;
                        double bili2 = jianju / chang2;

                        if(d1 < 3*duan1 && d2 < 3*duan2 && bili1 > 0.8 && bili1 < 3 && bili2 > 0.8 && bili2 < 3)
                        {   if(rect1.x<0.9*wimg&&rect1.x>0.1*wimg&&
                            rect2.x<0.9*wimg&&rect2.x>0.1*wimg&&
                            rect1.y<0.9*himg&&rect1.y>0.1*himg&&
                            rect2.y<0.9*himg&&rect2.y>0.1*himg)//边缘不检测
                            {
                            Arrmor onearrmor;
                            onearrmor.shezhiarrmor(rect1, rect2);
                            arrmors.push_back(onearrmor);
                        }
                        }

                    }
                }
        }
    }
    //雷霆大if嵌套找装甲板灯条

    




    for(int i=0;i<arrmors.size();i++)
    {
        cv::Point2f jiao[4] = {arrmors[i].jiao1, arrmors[i].jiao2, arrmors[i].jiao3, arrmors[i].jiao4};
        for(int k=0;k<4;k++)
            cv::line(image, jiao[k], jiao[(k+1)%4], cv::Scalar(255, 0, 255), 3);

        //中心点画个圆点
        cv::circle(image, arrmors[i].cpoint, 3, cv::Scalar(255,0,255), -1);

        //PnP位姿解算
        std::vector<cv::Point2f> imagePoints = {arrmors[i].jiao2, arrmors[i].jiao1, arrmors[i].jiao4, arrmors[i].jiao3};
        cv::Mat rvec, tvec;
        if(cv::solvePnP(SMALL_ARMOR_POINTS, imagePoints, neican, jibian, rvec, tvec, false, cv::SOLVEPNP_IPPE))
        {
            //画出装甲板坐标系（轴长100mm）：X红、Y绿、Z蓝，Z轴方向就是装甲板正面的朝向
            cv::drawFrameAxes(image, neican, jibian, rvec, tvec, 70.0f);

            //tvec是装甲板中心在相机坐标系下的位置(mm)，它的模就是相机到装甲板的直线距离
            double dist = cv::norm(tvec);
            cv::putText(image, cv::format("%.0fmm", dist), arrmors[i].cpoint, cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 255), 2);

        }
    }

    double time2 = static_cast<double>(cv::getTickCount());
    double timex = (time2 - time1)/cv::getTickFrequency();
    //计时结束


    cv::putText(image, cv::format("%fms", timex), cv::Point(100, 100), cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 255), 2);
    cv::resize(image, image, cv::Size(), 0.8, 0.8); 
    cv::resize(mask, mask, cv::Size(), 0.8, 0.8);




    cv::imshow("mask",mask);
    cv::imshow("image",image);
    timex=0;
    cv::waitKey(20);
    }
    return 0;
}