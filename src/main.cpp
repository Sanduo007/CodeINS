#include "Marker.h"
#include "Struct.h"
#include "QRCodeSolve.h"
using namespace cv;

#define MYNT

int main(int argc, char **argv)
{

	google::InitGoogleLogging(argv[0]);
	google::SetLogDestination(google::GLOG_INFO, "../output/wyb_log_");

	Mat img;
	FILE *fimgbin;

	MyTime mytime;
	char imgname[50] = {'\0'};
	char imgpath[1000] = {'\0'};
	char imgindexpath[1000] = {'\0'};
	char indexbuff[1000] = {'\0'};
	std::vector<Marker> detectedM;

#ifdef MYNT
	std::vector<std::vector<double>> allimu;
	std::vector<double> allimg_timestmp;
	double imgstamp;
	double epoch[7] = {0.0};

	fimgbin = fopen("/media/wyb/Study/MyPaper/实验/0118/first/mynteye/left/imggpst.bin", "rb");
	while (!feof(fimgbin))
	{
		fread((void *)&imgstamp, sizeof(double), 1, fimgbin);
		allimg_timestmp.push_back(imgstamp);
	}
	fclose(fimgbin);
	QRCodeSolve qrcodesolver = QRCodeSolve("/home/wyb/wyb/CodeINS/config.yaml", "/home/wyb/wyb/CodeINS/output/");

	int i = 0;

	while (allimg_timestmp.size())
	{
		qrcodesolver.imgindex = i++;
		std::stringstream ss; //studied from mynteye SDK
		ss << qrcodesolver.imgdir << std::dec << std::setw(6) << std::setfill('0') << i << ".png";

		img = imread(ss.str(), CV_LOAD_IMAGE_GRAYSCALE);

		if (img.empty())
		{
			std::cout << "empty" << std::endl;
			break;
		}
		std::cout << "-------------img" << qrcodesolver.imgindex << "--------------" << std::endl;
		qrcodesolver.markerDetection(img, allimg_timestmp[0]);
		allimg_timestmp.erase(allimg_timestmp.begin());
	}
	//std::cout << allimg_timestmp.size() << std::endl;
#else

	QRCodeSolve qrcodesolver = QRCodeSolve("/media/wyb/Study/MyPaper/prepaper/ExperienceData/0822/OutputFiles20180822190151/",
										   "/home/wyb/wyb/CodeINS/output/", GUIDANCE, OLDMARKER);

	sprintf(imgindexpath, "%s%s", qrcodesolver.imgdir, "realtime.txt");

	f_imgindex = fopen(imgindexpath, "r");
	int k = sizeof(indexbuff);

	while (fgets(indexbuff, sizeof(indexbuff), f_imgindex))
	{
		sscanf(indexbuff, "%d %d %d %d %d %lf\n", &qrcodesolver.imgindex, &mytime.min, &mytime.sec, &mytime.msec, &gpst.Week, &gpst.SecOfWeek);
		sprintf(imgname, "image%d.jpg", qrcodesolver.imgindex);
		sprintf(imgpath, "%s%s", qrcodesolver.imgdir, imgname);
		if (qrcodesolver.imgindex == 1484)
		{
			int hhh = 0;
		}
		img = imread(imgpath, CV_LOAD_IMAGE_GRAYSCALE);
		//imshow("image", img);
		//waitKey(1);
		std::cout << "-------------img" << qrcodesolver.imgindex << "--------------" << std::endl;
		qrcodesolver.markerDetection(img, gpst);
	}
#endif

	cvDestroyAllWindows();
	//google::ShutdownGoogleLogging();
	return 1;
}