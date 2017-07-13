/* -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
* File Name : preliminary_analysis.cpp
* Creation Date : 13-07-2017
* Created By : Rui An  
_._._._._._._._._._._._._._._._._._._._._.*/


#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

unsigned int count_bits(int n){
	int count = 0;
	while(n){
		n &= n-1;
		count ++;
	}
	return count ;
}


bool check_ones(cv::Mat& img){ 
	cv::Mat gray_image;
	cv::cvtColor(img, gray_image, COLOR_BGR2GRAY);
	int sum = 0;
	for(int i=0; i<gray_image.rows; i++){
		for(int j=0; j<gray_image.cols; j++){
			int gray_intensity = gray_image.at<uint8_t>(i,j);
			sum += count_bits(gray_intensity);
		}
	}
	if(sum % 42 == 0){
		return true;
	}
	else{
		return false;
	}
}


int main(int argc, char** argv){
	char* imageName = argv[1];
	Mat image;
	image = imread( imageName, 1 );
  if( argc != 2 || !image.data )
  {
	  printf( " No image data \n " );
	  return -1;
  }
	else{
		if(check_ones(image)){
			printf("it is good\n");
		}
		else{
			printf("it is bad\n");
		}
	}
}

