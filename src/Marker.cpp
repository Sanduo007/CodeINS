#include "Marker.h"

/************************************************构造函数****************************************************/
Marker::Marker(): id(-1),markertype(0)
{
}
Marker::Marker(int markertype_)
{
	markertype = markertype_;
}
/************************************************析构函数****************************************************/
Marker::~Marker() 
{
}

/*******************************************读取二维码内含信息**********************************************/
int Marker::getMarkerId(cv::Mat &markerImage, int &nRotations)
{
	assert(markerImage.rows == markerImage.cols);
	assert(markerImage.type() == CV_8UC1);
	int segmentnum = 0;
	if (markertype == OLDMARKER)
	{
		segmentnum = 4;
	}
	else if (markertype == NEWMARKER)
	{
		segmentnum = 3;
	}
	//如果assert判断的条件返回错误，则程序终止
	std::vector<int> pixelblocks;
	cv::Mat grey = markerImage;
	//threshold image
	//cv::threshold(grey, grey, 125, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	//外侧已进行二值化操作
	//Markers  are divided in 7x7 regions, of which the inner 5x5 belongs to marker info
	//the external border should be entirely black
	//去掉周围的一圈黑色，提取出5x5的网格
	int cellSize = markerImage.rows / 7;
	
	for (int y = 0; y<7; ++y)
	{
		int inc = 6;
		//for first and last row行, check the whole border
		if (y == 0 || y == 6) inc = 1;          //提取周围一圈检查！！！！
		for (int x = 0; x<7; x += inc)
		{
			int cellX = x * cellSize;
			int cellY = y * cellSize;
			cv::Mat cell = grey(cv::Rect(cellX, cellY, cellSize, cellSize));
			
			int nZ = cv::countNonZero(cell);
			//计算非零的像素个数？0 for blackn
			

			if (nZ >(cellSize*cellSize) / 2)
			{
				return -1;
			}
		}
	}

	//将图像标记信息存放在一个 5x5 的 Mat 中
	cv::Mat bitMatrix = cv::Mat::zeros(5, 5, CV_8UC1);

	//get information(for each inner square, determine if it is  black or white)  
	for (int y = 0; y<5; ++y)
	{
		for (int x = 0; x<5; ++x)
		{
			int cellX = (x + 1)*cellSize;
			int cellY = (y + 1)*cellSize;
			cv::Mat cell = grey(cv::Rect(cellX, cellY, cellSize, cellSize));
			int nZ = cv::countNonZero(cell);
			pixelblocks.push_back(nZ);

		}
	}
	
	std::sort(pixelblocks.begin(), pixelblocks.end());
	
	for (int y = 0; y<5; ++y)
	{
		for (int x = 0; x<5; ++x)
		{
			int cellX = (x + 1)*cellSize;
			int cellY = (y + 1)*cellSize;
			cv::Mat cell = grey(cv::Rect(cellX, cellY, cellSize, cellSize));
			
			int nZ = cv::countNonZero(cell);
			
			if (nZ<pixelblocks.at(segmentnum)) //这行代码有毒，新marker是3，旧的是4
				bitMatrix.at<uchar>(y, x) = 0;
			else 
				bitMatrix.at<uchar>(y, x) = 1;	
		}
	}
	
	//check all possible rotations
	//因为会有4种放置方向
	cv::Mat rotations[4];  //这个类类型的数组
	//海明距离,汉明距离是一个概念，它表示两个（相同长度）字对应位不同的数量
	int distances[4];

	rotations[0] = bitMatrix;
	distances[0] = hammDistMarker(rotations[0]);   //与正直marker的汉明距离

	std::pair<int, int> minDist(distances[0], 0);
	//最小汉明距离，pair

	for (int i = 1; i<4; ++i)
	{
		if (minDist.first == 0)
			break;
		//get the hamming distance to the nearest possible word
		rotations[i] = rotate(rotations[i - 1]);//顺时针转90度
		distances[i] = hammDistMarker(rotations[i]);

		if (distances[i] < minDist.first)
		{
			minDist.first = distances[i];
			minDist.second = i;
		}
	}
	//cout<<"minDist"<<" "<<minDist.first<<" "<<minDist.second<<endl;
	//cout<<"mat2id(rotations[minDist.second]):"<<" "<<mat2id(rotations[minDist.second])<<endl;

	nRotations = minDist.second;//记录在原始的图片上转了多少下才使得和标准marker距离最小（重合）
	
	if (minDist.first == 0)
	{
		return mat2id(rotations[minDist.second]);  //计算二维码信息
	}
	return -1;
}

/*****************************************Maker初始的坐标数据**********************************************/
int Marker::hammDistMarker(cv::Mat bits)
{
	int idsold[5][5] = //maker  1
	{
		{ 1, 1, 1, 1, 1 },
		{ 1, 1, 0, 1, 1 },
		{ 1, 0, 1, 0, 1 },
		{ 1, 1, 1, 1, 1 },
		{ 1, 1, 0, 1, 1 }
	};
	int idsnew[5][5] = //maker 2
	{
		{ 1, 1, 1, 1, 1 },
		{ 1, 1, 1, 1, 1 },
		{ 1, 1, 0, 1, 1 },
		{ 1, 1, 1, 1, 1 },
		{ 1, 0, 1, 0, 1 }
	};
	int ids[5][5];
	if (markertype == OLDMARKER)
	{
		memcpy(ids, idsold, sizeof(ids));
	}
	else if (markertype == NEWMARKER)
	{
		memcpy(ids, idsnew, sizeof(ids));
	}
	else
		return 1;
	/*
	int ids[5][5] = {
		{ 1, 1, 1, 1, 1 },
		{ 1, 1, 0, 1, 1 },
		{ 1, 0, 1, 1, 1 },
		{ 1, 1, 1, 0, 1 },
		{ 1, 1, 1, 1, 1 }
	};*/
	int dist = 0;
	int p = 0;

	for (int y = 0; y<5; ++y)
	{
		float minSum = 1e5; //hamming distance to each possible word
		for (; p<5; ++p)
		{
			if (p != y&&p > y)
				break;       
			float sum = 0;
			//now, count，逐行
			for (int x = 0; x<5; ++x)
			{
				sum += bits.at<uchar>(y, x) == ids[p][x] ? 0 : 1;
				//与ids（正图像标记）的汉明距离
			}
			if (minSum>sum)
				minSum = sum;
		}
		//do the and
		dist += minSum;
	}

	return dist;
}

/*********************************************计算二维码信息*************************************************/
int Marker::mat2id(const cv::Mat &bits)
{
	int val = 0;
	for (int y = 0; y<5; ++y)
	{
		val <<= 1;  //左移一位
		if (bits.at<uchar>(y, 1)) val |= 1;   //与1按位进行或运算，注意不是异或
		val <<= 1;
		if (bits.at<uchar>(y, 3)) val |= 1;
	}
	return val;
}

/********************************************CV图像旋转(顺90)函数*************************************************/
cv::Mat Marker::rotate(cv::Mat in)
{
	cv::Mat out;
	in.copyTo(out);
	for (int i = 0; i<in.rows; ++i)
	{
		for (int j = 0; j<in.cols; ++j)
		{
			out.at<uchar>(i, j) = in.at<uchar>(in.cols - j - 1, i);//顺时针转90度
		}
	}
	return out;
}

/*****************************************Maker初始的坐标数据**********************************************/
bool operator<(const Marker &M1, const Marker&M2)
{
	return M1.id<M2.id;
}
