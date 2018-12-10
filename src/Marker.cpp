#include "Marker.h"

/************************************************���캯��****************************************************/
Marker::Marker(): id(-1),markertype(0)
{
}
Marker::Marker(int markertype_)
{
	markertype = markertype_;
}
/************************************************��������****************************************************/
Marker::~Marker() 
{
}

/*******************************************��ȡ��ά���ں���Ϣ**********************************************/
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
	//���assert�жϵ��������ش����������ֹ
	std::vector<int> pixelblocks;
	cv::Mat grey = markerImage;
	//threshold image
	//cv::threshold(grey, grey, 125, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	//����ѽ��ж�ֵ������
	//Markers  are divided in 7x7 regions, of which the inner 5x5 belongs to marker info
	//the external border should be entirely black
	//ȥ����Χ��һȦ��ɫ����ȡ��5x5������
	int cellSize = markerImage.rows / 7;
	
	for (int y = 0; y<7; ++y)
	{
		int inc = 6;
		//for first and last row��, check the whole border
		if (y == 0 || y == 6) inc = 1;          //��ȡ��ΧһȦ��飡������
		for (int x = 0; x<7; x += inc)
		{
			int cellX = x * cellSize;
			int cellY = y * cellSize;
			cv::Mat cell = grey(cv::Rect(cellX, cellY, cellSize, cellSize));
			
			int nZ = cv::countNonZero(cell);
			//�����������ظ�����0 for blackn
			

			if (nZ >(cellSize*cellSize) / 2)
			{
				return -1;
			}
		}
	}

	//��ͼ������Ϣ�����һ�� 5x5 �� Mat ��
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
			
			if (nZ<pixelblocks.at(segmentnum)) //���д����ж�����marker��3���ɵ���4
				bitMatrix.at<uchar>(y, x) = 0;
			else 
				bitMatrix.at<uchar>(y, x) = 1;	
		}
	}
	
	//check all possible rotations
	//��Ϊ����4�ַ��÷���
	cv::Mat rotations[4];  //��������͵�����
	//��������,����������һ���������ʾ��������ͬ���ȣ��ֶ�Ӧλ��ͬ������
	int distances[4];

	rotations[0] = bitMatrix;
	distances[0] = hammDistMarker(rotations[0]);   //����ֱmarker�ĺ�������

	std::pair<int, int> minDist(distances[0], 0);
	//��С�������룬pair

	for (int i = 1; i<4; ++i)
	{
		if (minDist.first == 0)
			break;
		//get the hamming distance to the nearest possible word
		rotations[i] = rotate(rotations[i - 1]);//˳ʱ��ת90��
		distances[i] = hammDistMarker(rotations[i]);

		if (distances[i] < minDist.first)
		{
			minDist.first = distances[i];
			minDist.second = i;
		}
	}
	//cout<<"minDist"<<" "<<minDist.first<<" "<<minDist.second<<endl;
	//cout<<"mat2id(rotations[minDist.second]):"<<" "<<mat2id(rotations[minDist.second])<<endl;

	nRotations = minDist.second;//��¼��ԭʼ��ͼƬ��ת�˶����²�ʹ�úͱ�׼marker������С���غϣ�
	
	if (minDist.first == 0)
	{
		return mat2id(rotations[minDist.second]);  //�����ά����Ϣ
	}
	return -1;
}

/*****************************************Maker��ʼ����������**********************************************/
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
			//now, count������
			for (int x = 0; x<5; ++x)
			{
				sum += bits.at<uchar>(y, x) == ids[p][x] ? 0 : 1;
				//��ids����ͼ���ǣ��ĺ�������
			}
			if (minSum>sum)
				minSum = sum;
		}
		//do the and
		dist += minSum;
	}

	return dist;
}

/*********************************************�����ά����Ϣ*************************************************/
int Marker::mat2id(const cv::Mat &bits)
{
	int val = 0;
	for (int y = 0; y<5; ++y)
	{
		val <<= 1;  //����һλ
		if (bits.at<uchar>(y, 1)) val |= 1;   //��1��λ���л����㣬ע�ⲻ�����
		val <<= 1;
		if (bits.at<uchar>(y, 3)) val |= 1;
	}
	return val;
}

/********************************************CVͼ����ת(˳90)����*************************************************/
cv::Mat Marker::rotate(cv::Mat in)
{
	cv::Mat out;
	in.copyTo(out);
	for (int i = 0; i<in.rows; ++i)
	{
		for (int j = 0; j<in.cols; ++j)
		{
			out.at<uchar>(i, j) = in.at<uchar>(in.cols - j - 1, i);//˳ʱ��ת90��
		}
	}
	return out;
}

/*****************************************Maker��ʼ����������**********************************************/
bool operator<(const Marker &M1, const Marker&M2)
{
	return M1.id<M2.id;
}
