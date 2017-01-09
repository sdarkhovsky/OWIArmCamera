//#include "cv.h"
//#include "highgui.h"
#include <iostream>
#include <string>
#include <unistd.h>

//Maybe in OpenCV2.2 the correct include statement would be:
//#include "opencv2/opencv.hpp"

//#include "opencv2/calib3d/calib3d.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/imgcodecs.hpp"
//#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/core/utility.hpp"



// command line: CameraCapture <camera_dev_number> <image_path>
int main(int argc, char** argv)
{
    int result = 0;
	if (argc != 3)
	{
		printf("usage: %s <camera_dev_number> <image_path>", argv[0]);
		exit(1);
	}

	// Settings
	int camera_dev = atoi(argv[1]);
    std::string img_path(argv[2]);
    img_path += ".png";
    
	char* window_name = argv[0];

    cv::VideoCapture cap(camera_dev); // open the camera

    if(!cap.isOpened())  // check if we succeeded
    {
        std::cerr << "ERROR: Could not open cameras." << std::endl;
		exit(1);
    }


    cv::namedWindow(window_name,1);

    bool isValid = true;

    cv::Mat frame;

    try
    {
      cap >> frame; // get a new frame from the camera
    }
    catch( cv::Exception& e )
    {
      std::cout << "An exception occurred. Ignoring frame. " << e.err << std::endl;
      isValid = false;
      result = 1;
    }

    if (isValid)
    {
      try
      {
        cv::imshow(window_name, frame);

        if (!cv::imwrite(img_path.c_str(), frame)) {
        	printf("error\n");
	      	result = 1;
        }
      }
      catch( cv::Exception& e )
      {
        /************************************************************
        *    Sometimes an "Unrecognized or unsuported array type"   *
        *    exception is received so we handle it to avoid dying   *
        ************************************************************/
        std::cout << "An exception occurred. " << e.err << std::endl;
        result = 1;

      }
    }

    // the camera will be deinitialized automatically in VideoCapture destructor
    return result;
}

