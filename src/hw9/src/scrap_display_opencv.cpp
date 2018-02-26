#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>

#include <iostream>
#include <string>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
     string imageName("/home/jeff_michelle_ws/src/hw9/src/testImage.png");
     if(argc > 1)
     {
          imageName = argv[1];
     }

     Mat image;
     image = cv::imread(imageName.c_str(), IMREAD_COLOR);

     if(image.empty())
     {
          cout << "Could not open or find the image" << std::endl;
          return -1;
     }

     cv::namedWindow("Display Window", WINDOW_AUTOSIZE);
     cv::imshow("Display window", image);

     cv::waitKey(0);
     return 0;
}
