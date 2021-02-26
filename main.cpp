#include <iostream>
#include "ZZYCalibration.hpp"


int main() {

    vector<string> vecStr_CalibrationImages;
    /*
    int count = 38;
    for(int i=1;i<=count;++i){
        string imagePath = "/home/liushiqi/catkin_ws/src/calibration_images/vio_images_pinhole/color_";
        string
        vecStr_CalibrationImages.push_back();
    }
*/
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_1.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_2.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_3.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_4.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_5.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_6.jpg");

    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_7.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_8.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_9.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_10.jpg");

    //vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_11.jpg");
    //vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_12.jpg");
    /*
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_13.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_14.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_15.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_16.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_17.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_18.jpg");
    */

    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_19.jpg");

    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_20.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_21.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_22.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_23.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_24.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_25.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_26.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_27.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_28.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_29.jpg");

    /*
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_30.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_31.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_32.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_33.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_34.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_35.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_36.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_37.jpg");
    vecStr_CalibrationImages.push_back("/home/liushiqi/catkin_ws/src/calibration_images/zed_image/color_38.jpg");
    */




    CALIB::Calibration calibr(vecStr_CalibrationImages,6,9,26,26);

    calibr.initSrcPoint();
    calibr.getDstPoint();
    calibr.findHomographys();
    calibr.constructAndSolve();
    calibr.solveMatrixA();

    return 0;
}