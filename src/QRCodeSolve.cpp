#include <time.h>
#include <ceres/ceres.h>

#include <opencv2/core/eigen.hpp>

#include "Marker.h"
#include "Const.h"
#include "Struct.h"
#include "QRCodeSolve.h"
#include "CostStruct.h"

using namespace std;
using namespace cv;

QRCodeSolve::QRCodeSolve(char imgdir_[1000], char outputdir_[1000], int cameratype, int markertype, Size2f markerSize_, Size2f markerSizeinWorld_)
{
	char stime[32] = {'\0'};
	char filename[100] = {'\0'};
	char filepath[1000] = {'\0'};
	char errorimgname[100] = {'\0'};
	char errorimgpath[1000] = {'\0'};
	double blockside2d = 0.0;
	double blockside3d = 0.0;
	Point2f temppoint(0.0, 0.0);
	time_t tt = time(0);
	int shep = strftime(stime, sizeof(stime), "%Y%m%d%H%M%S", localtime(&tt)); //get current UTC time
	sprintf(filename, "%s%s%s", "POSEResults", stime, ".txt");
	sprintf(filepath, "%s%s", outputdir_, filename);
	sprintf(errorimgname, "%s%s%s", "errorimg", stime, ".txt");
	sprintf(errorimgpath, "%s%s", outputdir_, errorimgname);
	markerSize = markerSize_;
	markerSizeinWorld = markerSizeinWorld_;
	m_markerCorners3d.push_back(Point3f(-markerSizeinWorld.width / 2.0, -markerSizeinWorld.height / 2.0, 0));
	m_markerCorners3d.push_back(Point3f(markerSizeinWorld.width / 2.0, -markerSizeinWorld.height / 2.0, 0)); //�������
	m_markerCorners3d.push_back(Point3f(markerSizeinWorld.width / 2.0, markerSizeinWorld.height / 2.0, 0));
	m_markerCorners3d.push_back(Point3f(-markerSizeinWorld.width / 2.0, markerSizeinWorld.height / 2.0, 0));

	m_markerCorners2d.push_back(Point2f(0, 0));
	m_markerCorners2d.push_back(Point2f(markerSize.width - 1, 0));
	m_markerCorners2d.push_back(Point2f(markerSize.width - 1, markerSize.height - 1)); //˳ʱ��
	m_markerCorners2d.push_back(Point2f(0, markerSize.height - 1));

	blockside2d = (markerSize.width - 1) / 7.0;
	blockside3d = markerSizeinWorld.width / 7.0;

	if (markertype == OLDMARKER)
	{
		m_markerinterCorners2d.push_back(Point2f(blockside2d, blockside2d));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 6, blockside2d));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 6, blockside2d * 6));
		m_markerinterCorners2d.push_back(Point2f(blockside2d, blockside2d * 6));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 3, blockside2d * 2));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 4, blockside2d * 2));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 4, blockside2d * 3));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 5, blockside2d * 3));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 5, blockside2d * 4));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 4, blockside2d * 4));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 3, blockside2d * 4));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 2, blockside2d * 4));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 2, blockside2d * 3));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 3, blockside2d * 3));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 3, blockside2d * 5));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 4, blockside2d * 5));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 4, blockside2d * 6));
		m_markerinterCorners2d.push_back(Point2f(blockside2d * 3, blockside2d * 6));

		for (int i = 0; i < m_markerinterCorners2d.size(); i++)
		{
			temppoint.x = (m_markerinterCorners2d[i].x - 3.5 * blockside2d) * blockside3d / blockside2d;
			temppoint.y = (m_markerinterCorners2d[i].y - 3.5 * blockside2d) * blockside3d / blockside2d;
			m_markerinterCorners3d.push_back(Point3f(temppoint.x, temppoint.y, 0.0));
		}
	}
	else if (markertype == NEWMARKER)
	{
		printf("Nothing now!\n");
	}
	memcpy(imgdir, imgdir_, sizeof(char) * 100);
	results = fopen(filepath, "w+");
	errorimg = fopen(errorimgpath, "w+");
	if (cameratype == GUIDANCE)
	{
		//old_version
		// camMatrix = (Mat_<double>(3, 3) << 240.2721, 1.5735, 163.4041, 0.000, 238.7327, 118.7457, 0.000, 0.000, 1.000); //����ڲ�
		// distCoeff = (Mat_<double>(5, 1) << -0.0123, 0.2247, -0.0090, 0.0018, -0.4472);
		//1206
		camMatrix = (Mat_<double>(3, 3) << 233.6475, 0.000, 164.4622, 0.000, 235.9954, 120.2638, 0.000, 0.000, 1.000); //����ڲ�
		distCoeff = (Mat_<double>(5, 1) << 0.0342, -0.0193, -0.00133, 4.5599e-04, 0.007);
		img_height = 240;
		img_width = 320;
	}
	else if (cameratype == X3)
	{
		camMatrix = (Mat_<double>(3, 3) << 752.6946, 0.7260, 640.3954, 0, 738.9115, 356.1570, 0.000, 0.000, 1.000);
		distCoeff = (Mat_<double>(5, 1) << -0.1362, 0.1201, -0.0036, -0.0018, -0.0282);

		img_height = 360;
		img_width = 640;
	}
}

QRCodeSolve::~QRCodeSolve()
{
}

float QRCodeSolve::perimeter(vector<Point2f> a)
{
	float sum = 0, dx, dy;

	for (size_t i = 0; i < a.size(); ++i)
	{
		size_t i2 = (i + 1) % a.size(); //ȡģ��ǿ���պϵĻ�·

		dx = a[i].x - a[i2].x;
		dy = a[i].y - a[i2].y;

		sum += sqrt(dx * dx + dy * dy);
	}

	return sum;
}
void QRCodeSolve::findMarkerCandidates(const vector<vector<Point>> &contours, vector<Marker> &detectedMarkers)
{
	float m_minContourLengthAllowed = 30.0;

	vector<Point2i> approxCurve;
	vector<Marker> possibleMarkers;
	Mat contour_display = Mat(img_height, img_width, CV_8UC1);

	Point vec;
	float squaredDistance = 0.0;

	for (size_t i = 0; i < contours.size(); ++i)
	{
		approxPolyDP(contours[i], approxCurve, double(contours[i].size()) * 0.15, true); //����.approxCurve����Ϻ�Ķ�����������㼯

		if (approxCurve.size() != 4)
			continue;

		if (!isContourConvex(approxCurve)) // �ж��ǲ���͹�����
			continue;

		float minDist = 1e10; //ÿ����ϵ�������Ӧ�����ֵ�Ƚ�!(���)
		for (int i = 0; i < 4; ++i)
		{
			vec = approxCurve[i] - approxCurve[(i + 1) % 4];
			squaredDistance = vec.dot(vec);
			minDist = min(minDist, squaredDistance); //ȡ������Сֵ
		}

		if (minDist < m_minContourLengthAllowed)
			continue;

		Marker m;
		for (int i = 0; i < 4; ++i)
		{
			m.points.push_back(Point2f(approxCurve[i].x, approxCurve[i].y));
		}

		Point2f v1 = m.points[1] - m.points[0];
		Point2f v2 = m.points[2] - m.points[0];
		double o = (v1.x * v2.y) - (v1.y * v2.x); //���Σ�������ж�,������ұ�Ӧ�ý����ڶ����͵��ĸ����ﵽЧ��
		if (o < 0.0)
		{
			swap(m.points[1], m.points[3]); //swap ����
		}

		possibleMarkers.push_back(m); //ֻ���¿��ܵ������������Ķ�������ϳ����ģ�������
	}
	detectedMarkers.clear();
	for (size_t i = 0; i < possibleMarkers.size(); ++i)
	{
		detectedMarkers.push_back(possibleMarkers[i]);
	}
}

void QRCodeSolve::detectMarkers(const Mat &grayscale, vector<Marker> &detectedMarkers)
{
	Mat canonicalMarker;
	vector<Marker> goodMarkers;

	for (size_t i = 0; i < detectedMarkers.size(); ++i)
	{
		Marker marker = detectedMarkers[i];
		Mat M = getPerspectiveTransform(marker.points, m_markerCorners2d); //�任��ϵ����
		warpPerspective(grayscale, canonicalMarker, M, markerSize);		   //�����ı任 ��canonicalmarker��ͼ���任֮��canonical�䷶��size��markerSizeָ����
		//imshow(to_string(i), canonicalMarker);
		//waitKey(1);
		int nRotations = 5;

		int id = 0;
		id = marker.getMarkerId(canonicalMarker, nRotations); //!!!�������������ȥ����Χ7*7�Ĳ������ʴ������������־��������idΪ-1

		if (id == -1) //�ж��Ƿ����Ԥ����ά����Ϣ
			continue;
		while (nRotations != 0) //������ʹ�ü�⵽��marker������ʵ��λ�ö�Ӧ������˳������PNP����
		{
			rotate(marker.points.begin(), marker.points.begin() + 1, marker.points.end());
			M = getPerspectiveTransform(marker.points, m_markerCorners2d);
			warpPerspective(grayscale, canonicalMarker, M, markerSize);
			id = marker.getMarkerId(canonicalMarker, nRotations);
		}
		marker.id = id;
		goodMarkers.push_back(marker);
		destroyWindow(to_string(i));
		break;
	}

	if (goodMarkers.size() > 0)
	{
		vector<Point2f> preciseCorners(4 * goodMarkers.size());
		for (size_t i = 0; i < goodMarkers.size(); ++i)
		{
			Marker marker = goodMarkers[i];
			for (int c = 0; c < 4; ++c)
			{
				preciseCorners[i * 4 + c] = marker.points[c];
			}
		}

		cornerSubPix(grayscale, preciseCorners, cvSize(5, 5), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_ITER, 30, 0.1));

		for (size_t i = 0; i < goodMarkers.size(); ++i) //��ϸ��λ�ø��Ƹ���ǽǵ�
		{
			for (int c = 0; c < 4; ++c)
			{
				goodMarkers[i].points[c] = preciseCorners[i * 4 + c];
			}
		}
	}
	detectedMarkers = goodMarkers;
}

bool QRCodeSolve::markerDetection(Mat img, GPSTIME gpst)
{
	Mat imgByMory;
	Mat imgByAdptThr;
	vector<Marker> detectedMarkers;
	vector<vector<Point>> contours;

	img.copyTo(originimg);

	for (int k = 0; k <= 6; k++) //�ݶ�ȫ����ֵ
	{
		cv::threshold(img, imgByAdptThr, imgthr[k], 255, THRESH_BINARY);
		morphologyEx(imgByAdptThr, imgByAdptThr, MORPH_OPEN, Mat());
		morphologyEx(imgByAdptThr, imgByAdptThr, MORPH_CLOSE, Mat());
		imshow("morphologyEx", imgByAdptThr);
		waitKey(1);

		imgByMory = imgByAdptThr.clone();

		vector<vector<Point2i>> allContours;
		vector<Vec4i> hierarchy;

		findContours(imgByAdptThr, allContours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE); //���������ֻ��Ϊ����

		contours.clear();
		for (size_t i = 0; i < allContours.size(); ++i)
		{
			if (allContours[i].size() >= 4)
			{
				contours.push_back(allContours[i]); //�˳�С������,���������С
			}
		}

		if (contours.size())
		{
			findMarkerCandidates(contours, detectedMarkers);
		}
		else
			continue;

		if (detectedMarkers.size())
		{

			detectMarkers(imgByMory, detectedMarkers); //��ֵ������̬ѧ����֮������Ϣ
			if (detectedMarkers.size())
			{
				estiMatePosition(detectedMarkers, gpst);
				if (k != 0)
					rotate(imgthr, imgthr + k, imgthr + k + 1);
				return true;
			}
		}
		else
			continue;
	}
	//fprintf(errorimg, "%d\n", imgindex);
	fprintf(results, "%f %f %f %f %f %f %f %f %f \n",
			gpst.SecOfWeek, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	return false;
}

template <typename T>
double QRCodeSolve::getdist(T pointa, T pointb)
{
	return (double)sqrt((pointa.x - pointb.x) * (pointa.x - pointb.x) + (pointa.y - pointb.y) * (pointa.y - pointb.y));
}
/*
����͸��ͶӰ�任����M��������Ķ�ά���ڲ��ǵ�ͶӰ��ͼ����
*/
void QRCodeSolve::warpPerspectivePoints(std::vector<cv::Point2f> src, std::vector<cv::Point2f> &dst, cv::Mat M)
{
	double M11, M12, M13, M21, M22, M23, M31, M32, M33;
	Point2f temp;
	M11 = M.at<double>(0, 0);
	M12 = M.at<double>(0, 1);
	M13 = M.at<double>(0, 2);
	M21 = M.at<double>(1, 0);
	M22 = M.at<double>(1, 1);
	M23 = M.at<double>(1, 2);
	M31 = M.at<double>(2, 0);
	M32 = M.at<double>(2, 1);
	M33 = M.at<double>(2, 2);

	for (int i = 0; i < src.size(); i++)
	{
		temp.x = (M11 * src[i].x + M12 * src[i].y + M13) / (M31 * src[i].x + M32 * src[i].y + M33);
		temp.y = (M21 * src[i].x + M22 * src[i].y + M23) / (M31 * src[i].x + M32 * src[i].y + M33);
		dst.push_back(temp);
	}
}

/*
	��ȡͼ���ж�ά���ڲ��Ľǵ㣬���������趨marker�ǵ����Ĵ�С�����������ͬʱ�޸���Χ�ĵ�
*/
void QRCodeSolve::getintercorners(Mat img, Marker &m, vector<Point2f> &intercorners)
{

	vector<Point2f> corners, intercorners_seq, intercorners_seqtmp, m_markerinterCorners3d4caltmp;
	vector<Point2f> perspintercorners;
	vector<Point2f> outercorners;
	vector<int> distflag;
	Point2f temppoint;
	Mat imgdebug;
	Mat imgaftHist, imageEnhance;

	cvtColor(img, imgdebug, CV_GRAY2BGR);

	Mat kernel = (Mat_<double>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
	filter2D(img, imageEnhance, -1, kernel);
	imshow("laplace", imageEnhance);
	waitKey(1);

	double ppdist = 0.0;		 //�ǵ㵽��������
	double cornermatch_th = 5.0; //���ǵ���ģ��ͶӰ�ǵ�ƥ����������ֵ
	double tempdist = 0.0;
	int outnum = 0;

	//m_markerinterCorners3d4cal = m_markerinterCorners3d;
	Mat M = getPerspectiveTransform(m_markerCorners2d, m.points);
	warpPerspectivePoints(m_markerinterCorners2d, perspintercorners, M);
	m_Mask = cv::Mat(img_height, img_width, CV_8UC1, cv::Scalar(0));
	for (int i = 0; i < m_Mask.rows; i++)
	{
		for (int j = 0; j < m_Mask.cols; j++)
		{
			if (pointPolygonTest(m.points, Point2f(j, i), false) == 1)
			{
				m_Mask.at<uchar>(i, j) = 255;
			}
		}
	}
	// imshow("mask", m_Mask);
	// waitKey(1);
	goodFeaturesToTrack(imageEnhance, corners, 30, 0.001, 6, m_Mask); //��ȡ����ͼ���е�SHi-Tomasi�ǵ�,Ӧ�����˽⡣����Harris

	for (int j = 0; j < corners.size(); j++)
	{
		ppdist = pointPolygonTest(m.points, corners[j], true);

		if (abs(ppdist) <= 5) //���ܻ�������㵱���ڵ�
		{
			circle(imgdebug, corners[j], 3, Scalar(0, 0, 255), -1, 0);
			outercorners.push_back(corners[j]);
			/*circle(imgdebug, corners[j], 3, Scalar(0, 0, 0), -1, 0);
			imshow("allcornersimg", imgdebug);
			waitKey(1);*/
		}
		else if (ppdist > 0) //��������ڵĽǵ�
		{
			circle(imgdebug, corners[j], 3, Scalar(0, 255, 0), -1, 0);
			intercorners.push_back(corners[j]);
		}
	}

	for (int k = 0; k < perspintercorners.size(); k++) //���ݶ�ά��ͶӰ������ƽ���ϵ��ڵ�
	{
		circle(imgdebug, perspintercorners[k], 1.5, Scalar(255, 0, 0), -1, 0);
		double mindist = 20;
		for (int n = 0; n < intercorners.size(); n++)
		{
			tempdist = getdist(perspintercorners[k], intercorners[n]);
			if (tempdist < mindist)
			{
				mindist = tempdist;
				temppoint = intercorners[n];
			}
		}
		//m_markerinterCorners3d4caltmp.push_back(m_markerinterCorners3d[k]);
		//if (mindist < cornermatch_th)
		distflag.push_back(1);
		// else
		// 	distflag.push_back(0);

		intercorners_seqtmp.push_back(temppoint);
		// if (mindist < cornermatch_th) //��ͶӰ�ǵ������ͼ��ǵ�Ӧ����ֵ��
		// {
		//m_markerinterCorners3d4cal.erase(m_markerinterCorners3d4cal.begin() + k - outnum);//��⵽���ڲ�����٣����ڼ���Ķ�Ӧ����ϵ�µ�3D��ҲӦ����
		//m_markerinterCorners3d4cal.push_back(m_markerinterCorners3d[k]); //ֻ�С��ж�Ӧͼ��ǵ㡱��3D��Ų������
		//�Լ������ڲ��ǵ�����
		//outnum++;
		//perspintercorners.erase(perspintercorners.begin() + k);
		// }
	}
	int index = 0;
	for (int i = 0; i < intercorners_seqtmp.size(); i++)
	{
		double mindist = 20;
		for (int j = 0; j < perspintercorners.size(); j++)
		{
			tempdist = getdist(perspintercorners[j], intercorners_seqtmp[i]);
			if (tempdist < mindist)
			{
				mindist = tempdist;
				index = j;
			}
		}
		if (i != index)
			distflag[i] = 0;
	}
	for (int i = 0; i < distflag.size(); i++)
	{
		if (distflag[i])
		{
			m_markerinterCorners3d4cal.push_back(m_markerinterCorners3d[i]);
			intercorners_seq.push_back(intercorners_seqtmp[i]);
		}
	}
	imshow("allcornersimg", imgdebug);
	waitKey(1);
	if (m.points.size() <= outercorners.size())
	{
		for (int q = 0; q < m.points.size(); q++)
		{
			double mindist = 20;
			for (int n = 0; n < outercorners.size(); n++)
			{
				tempdist = getdist(m.points[q], outercorners[n]);
				if (tempdist < mindist)
				{
					mindist = tempdist;
					temppoint = outercorners[n];
				}
			}
			if (mindist < 5)
			{
				vector<Point2f> vectortmp;
				vectortmp.push_back(temppoint);
				cornerSubPix(img, vectortmp, Size(5, 5), Size(-1, -1), cvTermCriteria(CV_TERMCRIT_ITER, 30, 0.1)); //��С���˷�ϸ���ڲ��ǵ㣬�����ؼ�

				m.points[q].x = (vectortmp[0].x + m.points[q].x) / 2.0; //ϸ�������ⲿ�ĸ��ǵ�
				m.points[q].y = (vectortmp[0].y + m.points[q].y) / 2.0;
			}
		}
	}

	cornerSubPix(img, intercorners_seq, Size(5, 5), Size(-1, -1), cvTermCriteria(CV_TERMCRIT_ITER, 30, 0.1)); //��С���˷�ϸ���ڲ��ǵ㣬�����ؼ�

	/*for (int m = 0; m < intercorners_seq.size(); m++)
	{
		circle(imgdebug, intercorners_seq[m], 4, Scalar(255, 0, 0), -1, 0);
		imshow("allcornersimg", imgdebug);
		waitKey(1);
	}*/
	intercorners = intercorners_seq;
}

void QRCodeSolve::optimize(vector<Point2f> allcorners, vector<Point3f> allworldcorners3d)
{
	ceres::Problem poseproblem;
	ceres::LocalParameterization *local_parameterization = new ceres::QuaternionParameterization();

	ceres::LossFunction *loss_function;
	loss_function = new ceres::CauchyLoss(1.0); //��ʧ���������ٴ�Ĳв��Ӱ�죬�������Ӿ��۲�

	poseproblem.AddParameterBlock(Q_wc, 4, local_parameterization);
	poseproblem.AddParameterBlock(T_wc, 3);

	for (int i = 0; i < allworldcorners3d.size(); i++)
	{
		Eigen::Vector3d pw(allworldcorners3d[i].x, allworldcorners3d[i].y, allworldcorners3d[i].z);
		double ux = (double)allcorners[i].x;
		double vy = (double)allcorners[i].y;
		ceres::CostFunction *cost_function = REPROJECT_COST::Create(ux, vy, pw);
		poseproblem.AddResidualBlock(cost_function, loss_function, Q_wc, T_wc);
	}
	ceres::Solver::Options poseoptions;
	poseoptions.linear_solver_type = ceres::DENSE_SCHUR;
	poseoptions.minimizer_progress_to_stdout = false; // �����cout
	poseoptions.trust_region_strategy_type = ceres::DOGLEG;

	ceres::Solver::Summary posesummary;
	ceres::Solve(poseoptions, &poseproblem, &posesummary);

	cout << posesummary.BriefReport() << endl;
	LOG(INFO) << posesummary.BriefReport();
}
//rotation vector to quaternion,ref: Niu
bool Rvec2Q(double Rvec[], double Q[])
{
	double rvecnorm = 0.0;
	double n = 0.0;
	double nRvec[3] = {0.0};
	rvecnorm = sqrt(Rvec[0] * Rvec[0] + Rvec[1] * Rvec[1] + Rvec[2] * Rvec[2]);
	n = sin(0.5 * rvecnorm) / rvecnorm;

	Q[0] = cos(0.5 * rvecnorm);
	Q[1] = n * Rvec[0];
	Q[2] = n * Rvec[1];
	Q[3] = n * Rvec[2];
	return true;
}
// quaternion to rotation vector,ref: Shin 14
bool Q2Rvec(double Q[], double Rvec[])
{
	double f = 0;
	double halfPhiNorm = sqrt(Q[1] * Q[1] + Q[2] * Q[2] + Q[3] * Q[3]) / Q[0];
	f = sin(halfPhiNorm) / (2 * halfPhiNorm);
	Rvec[0] = Q[1] / f;
	Rvec[1] = Q[2] / f;
	Rvec[2] = Q[3] / f;
}
bool QRCodeSolve::cv2double(const Mat Rvec, const Mat Tvec)
{
	assert(Rvec.rows == 3 && Rvec.cols == 1 && Tvec.rows == 3 && Tvec.cols == 1);
	double R[3] = {0.0};
	cout << "Rvec " << Rvec << endl;
	cout << "Tvec " << Tvec << endl;

	Eigen::Vector3f RvecEg, TvecEg;
	cv2eigen(Rvec, RvecEg);
	cv2eigen(Tvec, TvecEg);

	R[0] = RvecEg(0);
	R[1] = RvecEg(1);
	R[2] = RvecEg(2);

	T_wc[0] = TvecEg(0);
	T_wc[1] = TvecEg(1);
	T_wc[2] = TvecEg(2);
	Rvec2Q(R, Q_wc);
	double Rcheck[3] = {0.0};
	Q2Rvec(Q_wc, Rcheck);
	return true;
}
void QRCodeSolve::estiMatePosition(std::vector<Marker> &detectedMarkers, GPSTIME gpst)
{
	vector<Point2f> intercorners, allcorners, allcorners_undis, allcorners_norm, allcorners_repro, optm_allcorners_repro;
	vector<Point3f> allworldcorners3d;
	Marker m = detectedMarkers[0];
	Mat Rvec, Tvec, raux, taux;
	//Mat optmraux, optmraux;
	Mat jacobian, rjacobian, tjacobian;
	Mat rjacobianT, tjacobianT;
	Mat rsquare, tsquare;
	double reproject_err = 0.0;
	double optm_reproject_err = 0.0;
	Eigen::Vector3f Euler;

	Eigen::Matrix3f R_wcEg, R_cw, R_bw; //rotate matrix from world to camera !ŷ���ǵĶ���

	Eigen::Vector3f P_w, T_wcEg; //the Marker's center in UAVbody coor ,which is parelled with camera coor.transformation from world to camera,Eigen
	Mat_<float> rotMat(3, 3);

	Mat rgbimg;

	double fx = camMatrix.at<double>(0, 0); //double or uchar depends on the type of Matrix!
	double fy = camMatrix.at<double>(1, 1);
	double cx = camMatrix.at<double>(0, 2);
	double cy = camMatrix.at<double>(1, 2);

	getintercorners(originimg, m, intercorners);

	allcorners.insert(allcorners.end(), m.points.begin(), m.points.end());
	allcorners.insert(allcorners.end(), intercorners.begin(), intercorners.end());
	allworldcorners3d.insert(allworldcorners3d.end(), m_markerCorners3d.begin(), m_markerCorners3d.end());
	allworldcorners3d.insert(allworldcorners3d.end(), m_markerinterCorners3d4cal.begin(), m_markerinterCorners3d4cal.end());

	undistortPoints(allcorners, allcorners_undis, camMatrix, distCoeff, cv::noArray(), camMatrix); //���÷�����ע��.����֮ǰ��ȥ����

	cvtColor(originimg, rgbimg, cv::COLOR_GRAY2BGR);
	for (int i = 0; i < allcorners.size(); i++)
	{
		allcorners_norm.push_back(cv::Point2f((allcorners_undis[i].x - cx) / fx, (allcorners_undis[i].y - cy) / fy)); //��һ���ǵ�
		//LOG(INFO) << allcorners[i].x << " " << allcorners[i].y << "  " << allcorners_undis[i].x << " " << allcorners_undis[i].y << "  " << allcorners_norm[i].x << " " << allcorners_norm[i].y << endl;
		circle(rgbimg, allcorners[i], 2, Scalar(0, 0, 255), -1, 0);
	}
	imshow("QRALLCorner", rgbimg);
	waitKey(1);

	vector<int> inliers;
	//solvePnP(allworldcorners3d, allcorners_undis, camMatrix, cv::noArray(), raux, taux, false, SOLVEPNP_EPNP);
	solvePnPRansac(allworldcorners3d, allcorners_undis, camMatrix, cv::noArray(), raux, taux, false, 100, 1.0, 0.99, inliers, SOLVEPNP_ITERATIVE);
	LOG_IF(INFO, inliers.size() < 10) << "inliers " << inliers.size();
	for (int i = 0; i < inliers.size(); i++)
	{
		LOG_IF(INFO, inliers.size() < 10) << inliers[i];
	}
	//solvePnP(allworldcorners3d, allcorners_undis, camMatrix, cv::noArray(), raux, taux, true, SOLVEPNP_ITERATIVE);
	m_markerinterCorners3d4cal.clear();
	//�Ż�֮ǰ����ͶӰ���
	projectPoints(allworldcorners3d, raux, taux, camMatrix, cv::noArray(), allcorners_repro, jacobian); //df/dr,df/dt,......
	for (int j = 0; j < allcorners_repro.size(); j++)													//�����л���ĵ�
	{
		reproject_err += (sqrt((allcorners_undis[j].x - allcorners_repro[j].x) * (allcorners_undis[j].x - allcorners_repro[j].x) + (allcorners_undis[j].y - allcorners_repro[j].y) * (allcorners_undis[j].y - allcorners_repro[j].y)) / allcorners_repro.size());
	}

	raux.convertTo(Rvec, CV_32F); //��ת����
	taux.convertTo(Tvec, CV_32F); //ƽ������
	cv2double(Rvec, Tvec);
	cout << "Q_wc: " << Q_wc[0] << " " << Q_wc[1] << " " << Q_wc[2] << " " << Q_wc[3] << endl;
	cout << "T_wc: " << T_wc[0] << " " << T_wc[1] << " " << T_wc[2] << endl;
	LOG(INFO) << "Q_wc: " << Q_wc[0] << " " << Q_wc[1] << " " << Q_wc[2] << " " << Q_wc[3];
	LOG(INFO) << "T_wc: " << T_wc[0] << " " << T_wc[1] << " " << T_wc[2];
	optimize(allcorners_norm, allworldcorners3d); //nolinear optimization,Q_wc & T_wc���г�ֵ

	//�Ż�֮�����ͶӰ���
	double optmrvec[3] = {0.0};
	Q2Rvec(Q_wc, optmrvec);
	Mat optmraux = (Mat_<double>(3, 1) << optmrvec[0], optmrvec[1], optmrvec[2]); //ceres�Ż�֮�����ת������ƽ��������cv��ʽ��
	Mat optmtaux = (Mat_<double>(3, 1) << T_wc[0], T_wc[1], T_wc[2]);
	projectPoints(allworldcorners3d, optmraux, optmtaux, camMatrix, cv::noArray(), optm_allcorners_repro, jacobian); //df/dr,df/dt,......
	for (int j = 0; j < optm_allcorners_repro.size(); j++)															 //�����л���ĵ�
	{
		optm_reproject_err += (sqrt((allcorners_undis[j].x - optm_allcorners_repro[j].x) * (allcorners_undis[j].x - optm_allcorners_repro[j].x) + (allcorners_undis[j].y - optm_allcorners_repro[j].y) * (allcorners_undis[j].y - optm_allcorners_repro[j].y)) / optm_allcorners_repro.size());
	}
	LOG_IF(INFO, optm_reproject_err > 1) << "img: " << imgindex << " optm_reproject_err: " << optm_reproject_err << " inliers " << inliers.size();
	// rjacobian = jacobian.colRange(0, 3);
	// tjacobian = jacobian.colRange(3, 6);

	// transpose(rjacobian, rjacobianT);
	// transpose(tjacobian, tjacobianT);
	// //cout << rjacobianT*rjacobian << endl << tjacobianT*tjacobian << endl;
	// invert(rjacobianT * rjacobian, rsquare, CV_SVD);
	// invert(tjacobianT * tjacobian, tsquare, CV_SVD);
	//cout << rsquare << endl << tsquare << endl;1
	optmraux.convertTo(Rvec, CV_32F); //��ת��������������������ceres�Ż�ǰ��Ľ����
	optmtaux.convertTo(Tvec, CV_32F); //ƽ������
	Rodrigues(Rvec, rotMat);
	cv2eigen(rotMat, R_wcEg);
	cv2eigen(Tvec, T_wcEg);

	R_cw = R_wcEg.transpose();

	P_w = -R_cw * T_wcEg;
	Euler[0] = atan2(R_wcEg(2, 1), R_wcEg(2, 2)) * RAD2DEG;														//roll,ref:Niu's PPT
	Euler[1] = atan2(-R_wcEg(2, 0), sqrt(R_wcEg(2, 1) * R_wcEg(2, 1) + R_wcEg(2, 2) * R_wcEg(2, 2))) * RAD2DEG; //pitch
	Euler[2] = atan2(R_wcEg(1, 0), R_wcEg(0, 0)) * RAD2DEG;														//yaw
	if (abs(P_w[2] / 1000.0) > 5)
	{
		printf("%dOutlier\n", imgindex);
	}
	fprintf(results, "%f %f %f %f %f %f %f %f %f\n", gpst.SecOfWeek, P_w[0] / 1000.0, P_w[1] / 1000.0, P_w[2] / 1000.0, Euler[0], Euler[1], Euler[2], reproject_err, optm_reproject_err);
}
