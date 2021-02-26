//
// Created by liushiqi on 21-1-16.
//

#ifndef ZHANG_CALIBRATION_ZZYCALIBRATION_HPP
#define ZHANG_CALIBRATION_ZZYCALIBRATION_HPP

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Eigenvalues>
#include <opencv2/core/eigen.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include "color.hpp"

using namespace std;
using namespace cv;

namespace CALIB {
    /**
     *相机标定类
     */
    class Calibration {
    public:
        Calibration(vector<string> &inputImages,int chessBoardDimension_rows,int chessBoardDimension_rows_cols,
                    int chessBoardDimension_x,int chessBoardDimension_y);

        Calibration() = delete;
        Calibration(const Calibration& obj) = delete;

        bool startCalibration();

        void getResult();

    public:
        bool calibrationResult;

    //private:
        //初始化3D点
        void initSrcPoint();
        //检测棋盘格2D点
        bool getDstPoint();
        // 求解单应矩阵
        void findHomographys();
        //构建Vij
        void getVectorFromHomography(Eigen::Matrix3d &Homography,Eigen::VectorXd &V,int i,int j);
        //构建矩阵(8)并且求解
        void constructAndSolve();
        //利用(8)的结果计算内参矩阵初值
        void solveMatrixA();
        //求解出姿态和位移
        void solveRAndT();
        //利用zui xiao er cheng qiu jie
        void refineResult();


    private:
        //输入相机的
        vector<string> _images;
        //chess board 3D points(Z axis is const zero)
        vector<Point2f> _p2f_srcPoint;
        //chessboard 2d points
        vector<vector<Point2f>> _p2f_dstPoint;
        //homegraphys
        vector<Eigen::Matrix3d> _vmatrix_homograpgys;
        //b向量
        Eigen::VectorXd B;
        //A矩阵
        Eigen::Matrix3d Matrix3dA;
        //每一帧的位置和位移
        vector<pair<Eigen::Matrix3d,Eigen::Vector3d>> vec_RAndT;
        //棋盘格规格
        cv::Size chessBoardSize;
        //棋盘格大小
        cv::Size chessBoardDimensions;
        //相机畸变
        double k1,k2;
        //相机内参
        double fx,fy,cx,cy;


    };
}
#endif //ZHANG_CALIBRATION_ZZYCALIBRATION_HPP
