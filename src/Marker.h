#ifndef Example_MarkerBasedAR_Marker_hpp
#define Example_MarkerBasedAR_Marker_hpp
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include "Struct.h"

class Marker
{  
private:
	int markertype;
	
public:
	Marker();
	Marker(int makertype_);
	~Marker();
	friend bool operator<(const Marker &M1,const Marker&M2);
	friend std::ostream & operator<<(std::ostream &str,const Marker &M);
	//opencvÐý×ªº¯Êý
	static cv::Mat rotate(cv::Mat  in);
	static int mat2id(const cv::Mat &bits);
	int hammDistMarker(cv::Mat bits);
	int getMarkerId(cv::Mat &in, int &nRotations);
	//
	int id;
	
	std::vector<cv::Point2f> points;
};
#endif