#include "Marker.h"
#include "Struct.h"
#include "QRCodeSolve.h"
using namespace cv;

int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);
	google::SetLogDestination(google::GLOG_INFO, "../output/wyb_log_");

	Mat img, imghist;
	FILE *f_imgindex;
	MyTime mytime;
	GPSTIME gpst;
	char imgname[50] = {'\0'};
	char imgpath[1000] = {'\0'};
	char imgindexpath[1000] = {'\0'};
	char indexbuff[1000] = {'\0'};
	std::vector<Marker> detectedM;
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
		if (qrcodesolver.imgindex == 938)
		{
			int hhh = 0;
		}
		img = imread(imgpath, CV_LOAD_IMAGE_GRAYSCALE);
		imshow("image", img);
		waitKey(1);
		qrcodesolver.markerDetection(img, gpst);
		//destroyWindow("image" + to_string(qrcodesolver.imgindex));
	}
	cvDestroyAllWindows();
	while (fclose(f_imgindex))
		sleep(50);
	//google::ShutdownGoogleLogging();
	return 1;
}