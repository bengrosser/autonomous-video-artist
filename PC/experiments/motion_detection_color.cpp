#include "opencv2/opencv.hpp"

using namespace cv;

int main()
{
    VideoCapture cap("camera0output0.avi");
    if(!cap.isOpened())
        printf("not opened\n");

    Mat prev;
    cap >> prev;
    if(prev.empty())
        return 0;

    Mat frame;
    cap >> frame;

    
    VideoWriter video("motion.avi", CV_FOURCC('M','J','P','G'), 1, Size(640,480), true);
    {
        if(!video.isOpened())
            printf("video is not opened\n");
    }

    while(!frame.empty())
    {
        Mat diff = frame-prev;
        diff.convertTo(diff, CV_32F);
        Mat mask = Mat::zeros(640, 480, CV_32FC1);
        pow(diff, 2, mask);
        transform(mask, mask, cv::Matx13f(1,1,1));
        sqrt(mask/3, mask);
        mask = mask > 10;
        printf("debug\n");
        Mat output;
        frame.copyTo(output, mask);
        imshow("difference", output);
        waitKey(1);
        video.write(output);
        cap >> frame;
    }

    cap.release();
    return 0;
}
