//
// Created by liushiqi on 21-1-16.
//
#include "ZZYCalibration.hpp"

//using namespace std;
using namespace CALIB;

Calibration::Calibration(vector<string> &inputImages,int chessBoardDimension_rows,int chessBoardDimension_cols,
                         int chessBoardDimension_x,int chessBoardDimension_y){
    _images = inputImages;
    chessBoardSize = cv::Size(chessBoardDimension_cols,chessBoardDimension_rows);
    chessBoardDimensions = cv::Size(chessBoardDimension_x,chessBoardDimension_y);
    calibrationResult = false;
    B.resize(6);
}
void Calibration::initSrcPoint(){
    //left top corner is original
    //Attation:the unit is meters!!!But input is millimetre!!
    int x = 0;
    int y = 0;
    for(int i = 0;i<chessBoardSize.width;++i){
        x += chessBoardDimensions.width;
        y  = 0;
        for(int j =0; j<chessBoardSize.height;++j){
            y += chessBoardDimensions.height;
            //input is centimeter!!
            //That is the point coordinate in real world
            _p2f_srcPoint.emplace_back(x*0.001,y*0.001);
            //cout<<"X:"<<x*0.001<<",Y:"<<y*0.001<<endl;
        }
    }
    cout<<GREEN<<"init complete!"<<RESET<<endl;
}

bool Calibration::getDstPoint(){
    cv::Mat chessBoardImage;
    for(auto i:_images) {
        chessBoardImage = cv::imread(i,IMREAD_GRAYSCALE);
        vector<cv::Point2f> corners;
        // 如果搜索的角点和预测输出的角点数量不一致，那么就报错；
        if( (cv::findChessboardCorners(chessBoardImage,chessBoardSize,corners) == 0 ) ||
        (cv::find4QuadCornerSubpix(chessBoardImage,corners,Size(5,5)) == false)){
            cout<<RED<<"Calibration::getDstPoint():Can't Find Corners, Please check your input image!"<<endl;
            cout<<RED<<"ERROR input: "<<YELLOW<<i<<RESET<<endl;
            return false;
        }
        _p2f_dstPoint.push_back(corners);
#ifdef DEBUG_FLAG
        int result = cv::findChessboardCorners(chessBoardImage,chessBoardSize,corners);

        //cout<<corners.size()<<endl;
        //for(auto &i:corners) cout<<i.x<<", "<<i.y<<endl;

        //cout<<GREEN<<"Find: "<<result<<" Corners."<<RESET<<endl;
        //cv::drawChessboardCorners(chessBoardImage,chessBoardSize,corners,result);
        //cv::imshow("",chessBoardImage);
        //cv::waitKey(0);
#endif
    }
    cout<<GREEN<<"init 3D Points complete!"<<RESET<<endl;
    return true;
}

void Calibration::findHomographys() {

    for(auto &i:_p2f_dstPoint){
        cv::Mat  result = cv::findHomography(_p2f_srcPoint,i,RANSAC);
        Eigen::Matrix3d HomographyMatrix;
        cv::cv2eigen(result,HomographyMatrix);
        _vmatrix_homograpgys.push_back(HomographyMatrix);

#ifdef DEBUG_FLAG
        // output the homographyMatrix result
        //cout<<"--------------"<<endl;
        //cout<<HomographyMatrix<<endl;
        //cout<<"--------------"<<endl;
#endif
    }
    cout<<GREEN<<"find Homographys complete!"<<RESET<<endl;
}

void Calibration::getVectorFromHomography(Eigen::Matrix3d &h, Eigen::VectorXd &V,int i, int j) {

    V.resize(6);
    //注意Eigen中行列号从0开始
    i--;
    j--;
    V[0] = h(0,i)*h(0,j);
    V[1] = h(0,i)*h(1,j) + h(1,i)*h(0,j);

    V[2] = h(1,i)*h(1,j);
    V[3] = h(2,i)*h(0,j) + h(0,i)*h(2,j);

    V[4] = h(2,i)*h(1,j) + h(1,i)*h(2,j);
    V[5] = h(2,i)*h(2,j);
}

void Calibration::constructAndSolve() {

    //公式(8)矩阵的大小
    Eigen::MatrixXd V(2*_vmatrix_homograpgys.size(),6);
    int i = 0;
    for(auto &h:_vmatrix_homograpgys) {
        Eigen::VectorXd V12(6), V11(6), V22(6);
        getVectorFromHomography(h,V12,1,2);
        getVectorFromHomography(h,V11,1,1);
        getVectorFromHomography(h,V22,2,2);
        V.block(i,0, 1, 6) = V12.transpose();
        V.block(i+1,0, 1, 6) = (V11-V22).transpose();
        i+=2;
    }

    Eigen::EigenSolver<Eigen::MatrixXd> VTV(V.transpose()*V);
    //这里定义的MatrixXcd必须有c，表示获得的是complex复数矩阵
    Eigen::MatrixXcd evecs = VTV.eigenvectors();
    //get smallest eigenvalue;
    Eigen::MatrixXcd evals = VTV.eigenvalues();//获取矩阵特征值 6*1
    Eigen::MatrixXd evalsReal;//注意这里定义的MatrixXd里没有c
    evalsReal=evals.real();//获取特征值实数部分
    Eigen::MatrixXf::Index evalsMin;
    evalsReal.rowwise().sum().minCoeff(&evalsMin);//得到最xiao特征值的位置

    //得到对应特征向量
    B << evecs.real()(0, evalsMin),
    evecs.real()(1, evalsMin),
    evecs.real()(2, evalsMin),
    evecs.real()(3, evalsMin),
    evecs.real()(4, evalsMin),
    evecs.real()(5, evalsMin);

#ifdef DEBUG_FLAG
    cout<<"Eigen values:"<<evals<<endl;
    cout << "Eigen vectors = \n" <<evecs << endl;
    //cout << "smallest Eigen vector = \n" <<B << endl;
#endif

    cout<<GREEN<<"init calibration complete!"<<RESET<<endl;
}

void Calibration::solveMatrixA(){
    cy = (B[1]*B[3] - B[0]*B[4])/(B[0]*B[2] - B[1]*B[1]);
    double lamda = B[5] - (B[3]*B[3]+cy*(B[1]*B[3]-B[0]*B[4]))/B[0];
    cout<<"lamda:"<<lamda<<endl;
    cout<<"lamda/B[0] :"<<lamda/B[0]<<endl;
    fx = sqrt(lamda/B[0]);
    fy = sqrt(lamda*B[0]/(B[0]*B[2]-B[1]*B[1]));
    double gamma = -1.0*B[1]*fx*fx*fy/lamda;
    cout<<"gamma:"<<gamma<<endl;
    cx = gamma*cy/fy-B[3]*fx*fx/lamda;

#ifdef DEBUG_FLAG
    cout<<"fx:"<<fx<<endl;
    cout<<"fy:"<<fy<<endl;
    cout<<"cx:"<<cx<<endl;
    cout<<"cy:"<<cy<<endl;
#endif

}

bool Calibration::startCalibration() {
    // 初始化空间点，其中Z都设置为0(设定标定板为Z=0的平面)
    initSrcPoint();
    //检测所有图片中的角点
    getDstPoint();
    //求解homography matrix
    Mat HMatrix = findHomography(_p2f_srcPoint,_p2f_dstPoint,1);
    Eigen::Matrix3d Homography;
    //转换为eigen的格式
    cv2eigen(HMatrix,Homography);



}