#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

#include <algorithm>
#include <vector>
#include <dirent.h>
#include <sys/types.h>

#include "opencv2/highgui/highgui.hpp"
//#include <opencv2/core/utility.hpp>
//#include "opencv2/core/cuda.hpp"
//#include "opencv2/highgui.hpp"
//#include "opencv2/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <unistd.h>

std::string joint_name[] = 
{
//	"base_joint1", 
	"shoulder 1", 
	"elbow 1", 
//	"wrist1", 
//	"gripper_joint1",
//	"base_joint1", 
	"shoulder 2", 
	"elbow 2", 
//	"wrist1", 
//	"gripper_joint1"

};

// found at http://www.cplusplus.com/forum/unices/3548/
// read_directory()
//   Return an ASCII-sorted vector of filename entries in a given directory.
//   If no path is specified, the current working directory is used.
//
//   Always check the value of the global 'errno' variable after using this
//   function to see if anything went wrong. (It will be zero if all is well.)
//
std::vector <std::string> read_directory( const std::string& path = std::string() )
{
  std::vector <std::string> result;
  dirent* de;
  DIR* dp;
  errno = 0;
  dp = opendir( path.empty() ? "." : path.c_str() );
  if (dp)
    {
    while (true)
      {
      errno = 0;
      de = readdir( dp );
      if (de == NULL) break;
	  std::string s(de->d_name);
	  if (s == "." || s == "..")
		continue;
      result.push_back(s);
      }
    closedir( dp );
    std::sort( result.begin(), result.end() );
    }
  return result;
}


/*
read a training image
apply a feature selecting yellow regions
build predictor: yellow_region(t), joint_command(t) => yellow_region(t+1)
test the predictor on the training samples
command line: OWIFeatCalcPrediction [<training_samples_directory>]   (without terminating slash)
*/
int main(int argc, char** argv)
{
    int result = 0;
	std::string training_samples_directory;
	bool make_gray=false;
    bool make_hsv=true;
    bool resize_src = false;
	float width_scale = 0.2f;
    float height_scale = 0.2f;

	if (argc == 1)
	{
		char buf[500]; 
		if (readlink("/proc/self/exe", buf, sizeof(buf)) > 0)
		{
			std::string sbuf = buf;
			std::size_t found = sbuf.find_last_of("/\\");
			training_samples_directory = sbuf.substr(0,found+1) + "../TrainingSamples";
		}
	}	
	else
	if (argc == 2)
		training_samples_directory = argv[1];
	else
	{
		printf("usage: %s [<training_samples_directory>]   (without terminating slash)", argv[0]);
		exit(1);
	}

	std::vector <std::string> img_files = read_directory(training_samples_directory);

 	for (std::vector<std::string>::iterator it = img_files.begin() ; it != img_files.end(); ++it)
	{
		cv::Mat orig_img, img, img_aux, img_to_show;

		std::string img_path =  training_samples_directory + "/" + *it;
		orig_img = cv::imread(img_path);
        if (orig_img.empty())
		{
			std::cout << "can't open image file: "  << *it << std::endl;
			continue;
        }


        // Change format of the image
        if (make_gray) cv::cvtColor(orig_img, img_aux, cv::COLOR_BGR2GRAY);
//        else if (use_gpu) cv::cvtColor(orig_img, img_aux, cv::COLOR_BGR2BGRA);
		else if (make_hsv) cv::cvtColor(orig_img, img_aux, cv::COLOR_BGR2HSV);
        else orig_img.copyTo(img_aux);


        // Resize image
        if (resize_src) cv::resize(img_aux, img, cv::Size(img.cols*width_scale, img.rows*height_scale));
        else img = img_aux;

		std::string winname = "OWIFeatCalcPrediction";

		// select yellow pixels
		cv::Mat feat_img;

		// debugging
		#ifdef DEBUG_DATA 
		{
			int row = 180;
			int col = 289;
            cv::Mat dbg_img = orig_img;
			cv::Vec3b bgrPixel = dbg_img.at<cv::Vec3b>(row, col);

			uint8_t* pixelPtr = (uint8_t*)dbg_img.data;
			int cn = dbg_img.channels();

			cv::Scalar_<uint8_t> bgrPixel1;
			bgrPixel1.val[0] = pixelPtr[row*dbg_img.cols*cn + col*cn + 0]; // B
			bgrPixel1.val[1] = pixelPtr[row*dbg_img.cols*cn + col*cn + 1]; // G
			bgrPixel1.val[2] = pixelPtr[row*dbg_img.cols*cn + col*cn + 2]; // R
			int dummy=0;
			dummy++;
		}
		#endif

		// OpenCV channels's order is BGR, not RGB
        // select yellow pixels
		cv::Scalar lower_bound = cv::Scalar(43,81,100);
        cv::Scalar upper_bound = cv::Scalar(102,167,205);
        if (make_hsv)
		{
			// In HSV color space the hue range is [0,360] degrees, but for 8-bit images H stores H/2 to fit 0 to 255
			lower_bound = cv::Scalar(35/2,0.3*255,0.3*255);
	        upper_bound = cv::Scalar(45/2,255,255);
		}
		cv::inRange(img, lower_bound, upper_bound, feat_img);
		img_to_show = feat_img;
		cv::putText(img_to_show, *it, cv::Point(5, 65), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 100, 0), 2);
		cv::imshow(winname, img_to_show);
		cv::waitKey(0);
/*
		// create clusters from the points with the distance from each other not more than a certain value
		cv::Mat kernel;
        int kernel_size = 5;
		kernel = cv::Mat::ones( kernel_size, kernel_size, CV_32F )/ (float)(kernel_size*kernel_size);
		cv::Mat smooth_feat_img;
		cv::filter2D(feat_img, smooth_feat_img,-1, kernel);
		cv::Mat thresh_feat_img;
		cv::threshold(smooth_feat_img, thresh_feat_img, 100, 255, cv::THRESH_BINARY);
 
		img_to_show = thresh_feat_img;
		cv::putText(img_to_show, "Image: " + *it, cv::Point(5, 65), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 100, 0), 2);
		cv::imshow(winname, img_to_show);
		cv::waitKey(0);
*/
	}

	return result;
}
