
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#include <string>
#include <cv.h>
#include <math.h>
#include <unistd.h>
//#include <fstream.h>

#include <wiringPi.h>

using namespace cv;
using namespace std;

#define LED1     24
#define LED2     25

CvCapture* g_capture = NULL;
IplImage* PreProcessImg = NULL;
CvPoint point1, point2, point3, pointend, pointtemp;
CvPoint pointmeet;//相交点

vector<CvPoint> points;
vector<CvPoint> pointsleft;
vector<CvPoint> pointsright;

              

//结构体1，左侧的线，右侧的线
struct line
{
	CvPoint pointbottom;
	CvPoint pointtop;
}lineleft,lineright,linemid;



struct greycount
{
	uchar grey;
	int count;
};

 void distortion(Mat& camera_matrix,Mat& distortion_coefficients)
 {      
                // Mat camera_matrix = Mat(3, 3, CV_32FC1);
		//Mat distortion_coefficients;
 
		//导入相机内参和畸变系数矩阵
		FileStorage file_storage("out_YourCame_Camera_data.yml", FileStorage::READ);
		file_storage["Camera_Matrix"] >> camera_matrix;
		file_storage["Distortion_Coefficients"] >> distortion_coefficients;
		file_storage.release();
 
		//矫正
		//undistort(image, image1, camera_matrix, distortion_coefficients);

 }
//大津阈值,车道从背景中分离出来
void ImproveOTSU(IplImage* image, IplImage* image2)
{
	
	//对单通道数组应用固定阈值操作。该函数的典型应用是对灰度图像进行阈值操作得到二值图像
	//cvThreshold(image, image2, 0, 255, CV_THRESH_OTSU);//改变阈值
        //cvThreshold(image, image2, 180,255, CV_THRESH_BINARY);
  cvAdaptiveThreshold(image,image2,255,CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY_INV,45,15);

}

//hough变换检测车道
void Hough(IplImage* image, float* tabSin, float *tabCos, int *accum, int numrho, int &predistance1, int &preangle1, int &predistance2, int &preangle2)
{
	int r, angle1 = 1, angle2 = 1, distance1 = 0, distance2 = 0, n = 0;
	long j;
	int angelstart1 = 5;
	int angleend1 = 90;
	int anglestart2 = 95;
	int angleend2 = 180;

	if (preangle1 != 0)//chu shi hua
	{
		angelstart1 = preangle1 - 5;
		angleend1 = preangle1 + 5;
	}
	if (preangle2 != 0)
	{
		anglestart2 = preangle2 - 5;
		angleend2 = preangle2 + 5;
	}

	int step = image->widthStep / sizeof(uchar);//存储一行像素所需要的字节数
	uchar *data0 = (uchar *)image->imageData;//先找到存放img图片所有像素点的像素值信息的地址

	//for (int i = 480-image->height/6; i > image->height/2; i--)//截取图像一半
	for (int i = 0; i < image->height; i++)
	//for(int i = image->height/2; i <image->height; i++)
	{
		for (j = 0; j < image->width; j++)
		{
			if (data0[i*step + j] == 255)//这个像素点是白色,将每个非零点，转换为霍夫空间的离散正弦曲线，并统计。
			{
				for (n = angelstart1; n < angleend1; n++)//40--70
				{
					r = cvRound(j * tabCos[n] + i * tabSin[n]);
					//printf("r1: %d\n ",r);
					r += (numrho - 1) / 2;//距离偏移一半,r有负值，防止生成的索引产生覆盖  

					accum[(n + 1) * (numrho + 2) + r + 1]++;//累加器相应单元+1，
				}
				for (n = anglestart2; n < angleend2; n++)//110--160
				{

					r = cvRound(j * tabCos[n] + i * tabSin[n]);
					//printf("r2: %d\n ",r);
					r += (numrho - 1) / 2;
					accum[(n + 1) * (numrho + 2) + r + 1]++;
				}
			}
		}
	}
	//printf("preangle1,preangle2:%d %d\n ",preangle1,preangle2);
	//找出数值最大的小格
	int numbermax1 = 30, numbermax2 = 30;
	
	

	for (r = 0; r < numrho; r++)//angle1\angle2
	{
		for (n = 40; n < 85;/*70*/ n++)
		{
			int base = (n + 1) * (numrho + 2) + r + 1;//累加器空间的索引
			if (accum[base] > numbermax1)
			{
				numbermax1 = accum[base];
				angle1 = n;
				distance1 = cvRound(r - (numrho - 1)*0.5f);//找到白色像素点对应的点数
				
			}
		}
		for (n = 95/*110*/; n<160; n++)
		{
			int base = (n + 1) * (numrho + 2) + r + 1;
			if (accum[base] > numbermax2)
			{
				numbermax2 = accum[base];
				angle2 = n;
				distance2 = cvRound(r - (numrho - 1)*0.5f);
			
			}
		}
	}
	//printf("angle1,angle2, distance1,distance2:%d %d %d %d \n",angle1, angle2,distance1,distance2);

	//if( distance1 || distance2 )
	//{
	if (angle1 == 0)
	{
		lineleft.pointbottom.x = 0;
		lineleft.pointbottom.y = 0;
		lineleft.pointtop.x = 0;
		lineleft.pointtop.y = 0;

	}
	if (angle2 == 0)
	{

		lineright.pointbottom.x = 0;
		lineright.pointbottom.y = 0;
		lineright.pointtop.x = 0;
		lineright.pointtop.y = 0;
	}
	
	if (angle1 < 90)//左车道线
	{
		if ((lineleft.pointbottom.y = (int)(distance1 / tabSin[angle1])) > image->height)//此处分母tabSin[angle1]为零
		{
			lineleft.pointbottom.x = (int)((distance1 - image->height  * tabSin[angle1]) / tabCos[angle1]);
			lineleft.pointbottom.y = image->height;
		}
		else
			lineleft.pointbottom.x = 0;

		if ((lineleft.pointtop.x = (int)(distance1 / tabCos[angle1])) > image->width)
		{
			lineleft.pointtop.x = image->width;
			lineleft.pointtop.y = (int)((distance1 - image->width*tabCos[angle1]) / tabSin[angle1]);
		}
		else
			lineleft.pointtop.y = 0;
			//printf("lbx,lby, ltx,lty:%d %d %d %d \n",lineleft.pointbottom.x,lineleft.pointbottom.y,lineleft.pointtop.x ,lineleft.pointtop.y );//????????


		/*if ((lineleft.pointtop.y = int(distance1 / tabSin[angle1])) < 0)
		{
			lineleft.pointtop.y = 0;
			lineleft.pointtop.x = int(distance1 / tabCos[angle1]);
		}
		else
			lineleft.pointtop.x = 0;
		if (((lineleft.pointbottom.x = (int)((distance1 - image->height *tabSin[angle1]) / tabCos[angle1])) > image->width))
		{
			lineleft.pointbottom.x = image->width;
			lineleft.pointbottom.y = int((distance1 - image->width*tabCos[angle1]) / tabSin[angle2]);
		}
		else
			lineleft.pointbottom.y = image->height;
			printf("lbx,lby, ltx,lty:%d %d %d %d \n",lineleft.pointbottom.x,lineleft.pointbottom.y,lineleft.pointtop.x ,lineleft.pointtop.y );
           */	
	}


	if (angle2 > 90 /* && angle2 < 180*/)//右车道线
	{
		if ((lineright.pointtop.y = int(distance2 / tabSin[angle2])) < 0)
		{
			lineright.pointtop.y = 0;
			lineright.pointtop.x = int(distance2 / tabCos[angle2]);
		}
		else
			lineright.pointtop.x = 0;
		if (((lineright.pointbottom.x = (int)((distance2 - image->height *tabSin[angle2]) / tabCos[angle2])) > image->width))
		{
			lineright.pointbottom.x = image->width;
			lineright.pointbottom.y = int((distance2 - image->width*tabCos[angle2]) / tabSin[angle2]);
		}
		else
			lineright.pointbottom.y = image->height;
		//printf("rbx,rby, rtx,rty:%d %d %d %d \n",lineright.pointbottom.x,lineright.pointbottom.y,lineright.pointtop.x ,lineright.pointtop.y );//????????
	}


	predistance1 = distance1;
	preangle1 = angle1;
	predistance2 = distance2;
	preangle2 = angle2;

	//return angle1;
	//}

}



//找出直线的相交点
CvPoint findmeetpoint(IplImage* image, float* tabSin, float *tabCos, int angleleft, int angleright)
{

	float tanA = -tabCos[angleleft] / tabSin[angleleft];
	float tanB = -tabCos[angleright] / tabSin[angleright];
	CvPoint pointmeet;
	pointmeet.x = (lineright.pointbottom.y - lineleft.pointbottom.y + lineleft.pointbottom.x*tanA - lineright.pointbottom.x*tanB) / (tanA - tanB);
	pointmeet.y = (pointmeet.x - lineleft.pointbottom.x)*tanA + lineleft.pointbottom.y;
	return pointmeet;  
}

//判断出弯曲的方向,0代表没有弯曲，1代表左弯，2代表右弯
int finddirection(IplImage* image, float* tabSin, float *tabCos, int angleleft, int angleright)
{
	int i = 0, j = 0;
	int i1 = 0, j1 = 0;
	int step = image->widthStep / sizeof(uchar); //imagestep为排列的图像行大小
	uchar *data0 = (uchar *)image->imageData; //指向排列的图像数据
	uchar *data1 = (uchar *)image->imageData;
	int numwhite = 0, numblack = 0;
	int num = 0;

	float tanA = -tabCos[angleleft] / tabSin[angleleft];
	float tanB = -tabCos[angleright] / tabSin[angleright];
	pointmeet.x = 0;
	pointmeet.y = 0;
	pointmeet = findmeetpoint(image, tabSin, tabCos, angleleft, angleright);
	
	//左车道
	//偏离点的确定。
	for (i = lineleft.pointbottom.y; i > pointmeet.y; i--)
	{
		j = float((i - lineleft.pointbottom.y)) / tanA + lineleft.pointbottom.x;//？？？？
		if (data0[i*step + j] == 255)//????
		{
			
			pointtemp.x = j;
			pointtemp.y = i;
			numwhite++;
		}
		if (data0[i*step + j] != 255 && numwhite > 20)
		{
			pointsleft.push_back(pointtemp);
			numblack++;
			break;
		}
		if (numblack>3)
			pointsleft.push_back(pointtemp);
	}
	numwhite = 0;


	for (i = lineright.pointbottom.y; i > pointmeet.y; i--)
	{
		j = float((i - lineright.pointbottom.y)) / tanB + lineright.pointbottom.x;
		
		if (data1[i*step + j] ==255)     
		{
			
			pointtemp.x = j;
			pointtemp.y = i;
			numwhite++;
		}
		if (data1[i*step + j] != 255 && numwhite>20)
		{
			pointsright.push_back(pointtemp);
			numblack++;
			break;
		}
		if (numblack>3)
			pointsright.push_back(pointtemp);
	}
	///////////////////////////////////////////////



	if (pointsleft.size() == 0 || pointsright.size() == 0)
	{
		pointtemp.y = 0;
		pointtemp.x = 0;
		pointsright.clear();
		pointsleft.clear();
		pointsright.push_back(pointtemp);
		pointsleft.push_back(pointtemp);
		return 0;
	}



	////////////////////////////////////////////////////
	//结合左右车道进行直线与曲线切点的校正
	int tempy = (pointsleft.back().y<pointsright.back().y) ? pointsleft.back().y : pointsright.back().y;
	for (i = tempy; i<pointsleft.back().y; i++)
	{
		j = float((i - lineleft.pointbottom.y)) / tanA + lineleft.pointbottom.x;
		if (data0[i*step + j] == 255)
		{
			pointsleft.pop_back();
			pointtemp.x = j;
			pointtemp.y = i;
			pointsleft.push_back(pointtemp);
			break;
		}
	}
	for (i = tempy; i<pointsright.back().y; i++)
	{
		j = float((i - lineright.pointbottom.y)) / tanB + lineright.pointbottom.x;
		if (data0[i*step + j] == 255)
		{
			pointsright.pop_back();
			pointtemp.x = j;
			pointtemp.y = i;
			pointsright.push_back(pointtemp);
			break;
		}
	}
	//判断车道弯曲方向
	int numll = 0, numlr = 0, numrl = 0, numrr = 0;
	for (i = pointsleft.back().y; i>pointmeet.y; i--)
	{
		j = float((i - lineleft.pointbottom.y)) / tanA + lineleft.pointbottom.x;
		for (int m = 1; m<5; m++)
		{
			if (data0[i*step + j - m] == 255)
			{
				numll++;
				break;
			}
		}
		for (int m = 1; m<5; m++)
		{
			if (data0[i*step + j + m] == 255)
			{
				numlr++;
				break;
			}
		}
	}
	for (i = pointsright.back().y; i>pointmeet.y; i--)
	{
		j = float((i - lineright.pointbottom.y)) / tanB + lineright.pointbottom.x;
		for (int m = 1; m<5; m++)
		{
			if (data0[i*step + j - m] == 255)
			{
				numrl++;
				break;
			}
		}
		for (int m = 1; m<5; m++)
		{
			if (data0[i*step + j + m] == 255)
			{
				numrr++;
				break;
			}
		}
	}
	if (numrr>numll&&numll<10)
		return 2;
	else if (numll>numrr&&numrr<10)
		return 1;
	else
		return 0;

}

//找出弯道中的点
void findcurvepoint(IplImage* image, float* tabSin, float *tabCos, int angleleft, int angleright, int direction)
{
	int i, j, m, n = 0;
	int N = 10;
	int step = image->widthStep / sizeof(uchar);
	uchar *data0 = (uchar *)image->imageData;
	int numwhite = 0, numblack = 0;
	int num = 0;

	float tanA = -tabCos[angleleft] / tabSin[angleleft];
	float tanB = -tabCos[angleright] / tabSin[angleright];
	//pointmeet=findmeetpoint(image,tabSin,tabCos,angleleft,angleright);
	if (pointmeet.y<2 * image->height / 5)
		pointmeet.y = 2 * image->height / 5;

	if (direction == 0)    //走直道的情况
	{
		pointsleft.push_back(pointmeet);//将pointmeet放到末尾
		pointsright.push_back(pointmeet);
		return;
	}
	else if (direction == 1)   //左转弯时
	{
		for (i = pointsleft.back().y; i>pointmeet.y; i--)
		{
			j = float((i - lineleft.pointbottom.y)) / tanA + lineleft.pointbottom.x;
			for (m = n; m<N; m++)
			{
				if (data0[i*step + j - m] == 255)
				{
					CvPoint pointtemp;
					pointtemp.x = j - m;
					pointtemp.y = i;
					pointsleft.push_back(pointtemp);
					n = m;
					if (m == N - 1)
						N++;
					//	N++;
				}
			}
		}
		N = 10;
		n = 1;
		for (i = pointsright.back().y; i>pointmeet.y; i--)
		{
			j = float((i - lineright.pointbottom.y)) / tanB + lineright.pointbottom.x;
			for (m = n; m<N; m++)
			{
				if (data0[i*step + j - m] == 255)
				{
					CvPoint pointtemp;
					pointtemp.x = j - m;
					pointtemp.y = i;
					pointsright.push_back(pointtemp);
					n = m;
					if (m == N - 1)
						N++;
				}
			}
		}
	}
	else if (direction == 2)   //右转弯时
	{
		for (i = pointsleft.back().y; i>pointmeet.y; i--)
		{
			j = float((i - lineleft.pointbottom.y)) / tanA + lineleft.pointbottom.x;
			for (m = n; m<N; m++)
			{
				if (data0[i*step + j + m] == 255)
				{
					CvPoint pointtemp;
					pointtemp.x = j + m;
					pointtemp.y = i;
					pointsleft.push_back(pointtemp);
					n = m;
					if (m == N - 1)
						N++;
				}
			}
		}
		N = 10;
		n = 1;
		for (i = pointsright.back().y; i>pointmeet.y; i--)
		{
			j = float((i - lineright.pointbottom.y)) / tanB + lineright.pointbottom.x;
			for (m = n; m<N; m++)
			{
				if (data0[i*step + j + m] == 255)
				{
					CvPoint pointtemp;
					pointtemp.x = j + m;
					pointtemp.y = i;
					pointsright.push_back(pointtemp);
					n = m;
					if (m == N - 1)
						N++;
				}
			}
		}
	}

}

 void FindLineIntersection(struct line linemid, CvPoint &p3,CvPoint &graveity,int &straight,int &turnleft,int &turnright)
  {
				   
	int delta_y=linemid.pointbottom.y-p3.y;
    int fun_1=delta_y*0.3;
	int a0=0,b0=1,c0=-(linemid.pointbottom.y-fun_1);
	int a1=linemid.pointbottom.y-p3.y;
	int b1=p3.x-linemid.pointbottom.x;
	int c1=linemid.pointbottom.x*p3.y-p3.x*linemid.pointbottom.y;
	int D=a0*b1-a1*b0;
				   
    graveity.x=(b0*c1-b1*c0)/D;
	graveity.y=(a1*c0-a0*c1)/D;
	
	if(graveity.x>270&&graveity.x<370)
	     straight++;
	else
	     straight=0;
	if(graveity.x<270)
	     turnleft++;
	else
	     turnleft=0;
	if(graveity.x>370)
	     turnright++;
	else
	      turnright=0;

 }


//主函数
int main(int argc, char* argv[])
{
	

        
	//cout<<"aaaaaaa"<<endl;
        //VideoCapture cap("/home/pi/1CarLine_Project/0411/00.mp4");
        VideoCapture cap(0);//打开自己摄像头
       ofstream fout("output.txt");


   // 
              
	//ROI
	Rect box;
	box.width = 640;
	box.height = 330;
	box.x = 0;
	box.y = 50;

	int turnleft=0;
	int turnright=0;
	int straight=0;
	IplImage* tempimage = NULL;
	IplImage* greyimage = NULL;
	
    int frame_count = 0;            //对视频帧计数的变量
	int Frmnum = 0;
	int numrho, n;
	float ang;
	int *accum;

	float theta = (float)(CV_PI / 180);//极坐标空间theta轴细分程度
	float *tabSin = 0;
	float *tabCos = 0;
	int numangle = 180;


	CvPoint p1;//一个含有x,y的点
	CvPoint p2;
	CvPoint p3;
    

	IplImage* pFrame = NULL;

	 
      wiringPiSetup () ;
       pinMode (LED1, OUTPUT) ;
       pinMode (LED2, OUTPUT) ;

       
             Mat camera_matrix = Mat(3, 3, CV_32FC1);
		Mat distortion_coefficients;
distortion(camera_matrix,distortion_coefficients);

	
	while (1)
	{
		
		
		
        usleep(800);
		Mat frame; 
       cap >> frame;   
		
	   Mat img(frame,box);
	   
       //滤波
	   int kernel_size=3;
	   Mat kernel=Mat::ones(kernel_size,kernel_size,CV_32F)/(float)(kernel_size*kernel_size);
	   filter2D(img,img,-1,kernel);

	   IplImage temp=(IplImage)img;
       pFrame=&temp;
       tempimage = cvCreateImage(cvGetSize(pFrame), 8, 3);//创建3通道（bgr)图像首地址，并分配存储空间。
		
		Mat img1;
               //矫正
		undistort(img, img1, camera_matrix, distortion_coefficients);
	   // distortion(img,img1);
				
		*pFrame = IplImage(img1);
		cvCopy(pFrame, tempimage);
		Frmnum++;
		//cvFlip(tempimage, tempimage, -1);  //读图像是倒着的，用这句就正过来了		

		if (Frmnum == 1)//当为第一帧时
		{
			PreProcessImg = cvCreateImage(cvSize((tempimage)->width, (tempimage)->height), IPL_DEPTH_8U, 1);//将tempimage改为pFrame
			greyimage = cvCreateImage(cvSize((tempimage)->width, (tempimage)->height), IPL_DEPTH_8U, 1);		
		
		}
		else
		{
			double t = (double)getTickCount();
			if (Frmnum % 1== 0)//每隔5帧一检测，设置帧率
			{
				
				cvCvtColor(tempimage, greyimage, CV_BGR2GRAY);
				ImproveOTSU(greyimage, PreProcessImg);//车道从背景中分离
				cvShowImage("double image", PreProcessImg);
				//cvShowImage("greyimage", greyimage);
				cvAnd(PreProcessImg, greyimage, PreProcessImg);//将前两个按位与运算
                              //cvShowImage("and after", PreProcessImg);

                              //极坐标空间rho轴细分程度，分配空间
				numrho = cvRound((PreProcessImg->width + PreProcessImg->height) * 2 + 1);//对double类型的数进行四舍五入，输出整型数
				accum = new int[(numangle + 2) * (numrho + 2)];//numrho是周长，numangle是180
				tabSin = new float[sizeof(tabSin[0]) * numangle];
				tabCos = new float[sizeof(tabCos[0]) * numangle];
				
				for (ang = 0, n = 0; n < numangle; ang += theta, n++)
				{
					tabSin[n] = (float)sin(ang);
					tabCos[n] = (float)cos(ang);
				}
		
				int predistanceleft = 0, preangleleft = 0, predistanceright = 0, preangleright = 0;

				vector<CvPoint>::iterator iter;
				///////////
				vector<CvPoint>::iterator iter1;
				vector<CvPoint>::iterator iter2;
                                /////////////
				memset(accum, 0, (numangle + 2) * (numrho + 2)*sizeof(int));//定义了一个numangle*numrho数组

				//cvSobel(PreProcessImg, PreProcessImg, 0, 1, CV_SCHARR);//scharr滤波器进行边缘提取
				
                 
				//blur(img,diss,Size(3,3));
				cvCanny(PreProcessImg,PreProcessImg,1,3,3);
				cvShowImage("edge", PreProcessImg);

				//cvShowImage("cvSobel", PreProcessImg);

				Hough(PreProcessImg, tabSin, tabCos, accum, numrho, predistanceleft, preangleleft, predistanceright, preangleright);
			
			
				//判断弯道的方向
				int direction = 0;

				if (predistanceleft && predistanceright)
				{
					direction = finddirection(PreProcessImg, tabSin, tabCos, preangleleft, preangleright);
				}


				//找出弯道中的点
				findcurvepoint(PreProcessImg, tabSin, tabCos, preangleleft, preangleright, direction);

				//画左车道线
				iter = pointsleft.begin();
				//printf("(*iter).x,(*iter).y:%d %d  \n",(*iter).x,(*iter).y);
				//printf("lineleft.pointbottom.x,lineleft.pointbottom.y,p1.x, p1.y:%d %d %d %d \n",lineleft.pointbottom.x,lineleft.pointbottom.y, p1.x,p1.y);
				if ((*iter).x != 0 && (*iter).y != 0)
				{

					p1.x = (lineleft.pointbottom.x + (*iter).x) / 2;
					p1.y = (lineleft.pointbottom.y + (*iter).y) / 2;
					//cvLine(tempimage, lineleft.pointbottom, *iter, CV_RGB(0, 255, 0), 3, 8);
					cvLine(tempimage, lineleft.pointbottom, p1, CV_RGB(0, 255, 0), 3, 8);
					
					
					
					                  
				}
				
				pointsleft.clear();


				//画右车道线
				iter = pointsright.begin();
				//printf("lineright.pointbottom.x,lineright.pointbottom.y,p2.x, p2.y:%d %d %d %d \n",lineright.pointbottom.x,lineright.pointbottom.y,p2.x, p2.y);
				if ((*iter).x != 0 && (*iter).y != 0)
				{
					p2.x = (lineright.pointbottom.x + (*iter).x) / 2;
					p2.y = (lineright.pointbottom.y + (*iter).y) / 2;
					//cvLine(tempimage, lineright.pointbottom, *iter, CV_RGB(0, 255, 0), 3, 8);
					cvLine(tempimage, lineright.pointbottom, p2, CV_RGB(0, 255, 0), 3, 8);
						
				}
				pointsright.clear();


				//画中线
				iter1 = pointsleft.begin();
				iter2 = pointsright.begin();
			    linemid.pointbottom.x=0,linemid.pointbottom.y=0;
				linemid.pointbottom.x=int((lineleft.pointbottom.x +lineright.pointbottom.x)/2);
				linemid.pointbottom.y=int((lineleft.pointbottom.y +lineright.pointbottom.y)/2);
				
              
				//if ((*iter1).x != 0 && (*iter1).y != 0&& (*iter2).x != 0&& (*iter2).y != 0)
				if (1)
				{
                   //float graveity[2];
				  // float graveityEnd[2];
				  CvPoint graveity;
				  CvPoint graveityEnd;
				  CvPoint imgMidLeft;
				  CvPoint imgMidEndLeft;
				  CvPoint imgMidRight;
				  CvPoint imgMidEndRight;
					p3.x = ((lineleft.pointbottom.x +lineright.pointbottom.x)/2+((*iter1).x+(*iter2).x)/2 ) / 2;
					p3.y = ((lineleft.pointbottom.y +lineright.pointbottom.y)/2+((*iter1).y+(*iter2).y)/2 ) / 2;
					//cvLine(tempimage, lineleft.pointbottom, *iter, CV_RGB(0, 255, 0), 3, 8);
					//cvLine(tempimage, linemid.pointbottom, p3, CV_RGB(0, 0, 255), 3, 8);
                    printf("linemid.pointbottom.x,linemid.pointbottom.y,p3.x,p3.y:%d %d %d %d \n",linemid.pointbottom.x,linemid.pointbottom.y,p3.x,p3.y);
                   
				 FindLineIntersection(linemid,p3,graveity,straight,turnleft,turnright);
                  graveityEnd.x=graveity.x;
				  graveityEnd.y=graveity.y-120.0;
				  cvLine(tempimage,graveity, graveityEnd, CV_RGB(0, 0, 255), 3, 8); 

				  imgMidLeft.x=box.width/2-75;
				  imgMidLeft.y=0;
				  imgMidEndLeft.x=box.width/2-75;
				  imgMidEndLeft.y=box.height;
				  cvLine(tempimage,imgMidLeft, imgMidEndLeft, CV_RGB(255, 0, 0), 3, 8); 

				  imgMidRight.x=box.width/2+70;
				  imgMidRight.y=0;
				  imgMidEndRight.x=box.width/2+70;
				  imgMidEndRight.y=box.height;
				  cvLine(tempimage,imgMidRight, imgMidEndRight, CV_RGB(255, 0, 0), 3, 8); 
				 //printf("graveity0,graveity1:%f %f \n",graveity[0],graveity[1]);
				   
                  printf("straight,turnleft,turnright:%d %d %d \n",straight,turnleft,turnright);
				  //fout<<graveity[0]<<" "<<linemid.pointbottom.x<<" "<<straight<<" "<<turnleft<<" "<<turnright<<endl;
				   
				   
				     /*   
					 //中 
					 
					               if(turnleft>10)
					{
						    digitalWrite (LED1, 1) ;     // On
  						  digitalWrite (LED2, 0) ;     // On
					}
					else if(turnright>10)
					{
						digitalWrite (LED1, 0) ;     // On
  						  digitalWrite (LED2, 1) ;     // On
					}
					else if(straight>10)
					{
						digitalWrite (LED1, 1) ;     // On
  						  digitalWrite (LED2, 1);     // On

					}*/
					
					//用中点控制
                                         if(graveity.x<245)
					{
						    digitalWrite (LED1, 1) ;     // On
  						  digitalWrite (LED2, 0) ;     // On
					}
					else if(graveity.x>390)
					{
						digitalWrite (LED1, 0) ;     // On
  						  digitalWrite (LED2, 1) ;     // On
					}
					else
					{
						digitalWrite (LED1, 1) ;     // On
  						  digitalWrite (LED2, 1);     // On
					}
				}
				pointsleft.clear();
				pointsright.clear();


				cvShowImage("aaaa", tempimage);
				delete[]accum;  //释放内存。非常重要
				delete[]tabSin;
				delete[]tabCos;
				cvWaitKey(3);
				
			}
			t = (double)getTickCount() - t;
			//printf("识别时间= %gms\n\n", t*1000. / cv::getTickFrequency());
		}
		
		frame_count++;
                //usleep(1000);

	}

	
	
	cvReleaseImage(&greyimage);
	cvReleaseImage(&PreProcessImg);
	cvReleaseCapture(&g_capture);

	cvDestroyWindow("aaaa");

	return 0;
}



