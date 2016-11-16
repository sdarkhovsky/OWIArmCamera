#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

#include <algorithm>
#include <functional>
#include <vector>
#include <dirent.h>
#include <sys/types.h>

#include "opencv2/highgui/highgui.hpp"
//#include <opencv2/core/utility.hpp>
//#include "opencv2/core/cuda.hpp"
//#include "opencv2/highgui.hpp"
//#include "opencv2/objdetect.hpp"
//#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <unistd.h>

#include "ais.h"
#include "../OWIArmControl/owi_arm_control.h"

#include "logger.hpp"

std::vector<std::string> get_joint_commands();

namespace ais {
	extern c_ais g_ais;
}

// found at http://www.cplusplus.com/forum/unices/3548/
// read_directory()
//   Return an ASCII-sorted vector of filename entries in a given directory.
//   If no path is specified, the current working directory is used.
//
//   Always check the value of the global 'errno' variable after using this
//   function to see if anything went wrong. (It will be zero if all is well.)
//
int get_file_number(std::string a)
{
	if (a.size()>3) a = a.substr(3);

	std::size_t found = a.find("_");
	if (found >= 0) a = a.substr(0,found);

	return std::stoi(a);
}

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

	struct {
	    bool operator()(std::string a, std::string b)
	    {   
			int a1 = get_file_number(a);
			int b1 = get_file_number(b);
	        return a1 < b1;
	    }   
	} sort_by_names;

    std::sort( result.begin(), result.end(), sort_by_names );
    }
  return result;
}

void get_connected_components(cv::Mat src, cv::Mat& labelImage, std::vector<cv::Vec2i>& component_areas)
{
		labelImage = cv::Mat(src.size(), CV_32S);
        cv::Mat stats, centroids;
		int nLabels = cv::connectedComponentsWithStats(src, labelImage, stats, centroids, 8);

		component_areas.resize(nLabels);
		std::vector<cv::Vec3b> colors(nLabels);

		for(int label = 0; label < nLabels; ++label){
			if (label == 0)
				colors[label] = cv::Vec3b(0, 0, 0);//background
			else
				colors[label] = cv::Vec3b( (rand()&255), (rand()&255), (rand()&255) );
			component_areas[label] = cv::Vec2i(label, stats.at<int>(label, cv::CC_STAT_AREA));
		}
		// sort component_areas by area
		struct {
		    bool operator()(cv::Vec2i a, cv::Vec2i b)
		    {   
		        return a.val[1] > b.val[1];
		    }   
		} sort_by_area;
	    std::sort(component_areas.begin(), component_areas.end(), sort_by_area);

        cv::Mat connected_components = cv::Mat(src.size(), CV_8UC3);
		for(int r = 0; r < connected_components.rows; ++r){
			for(int c = 0; c < connected_components.cols; ++c){
				int label = labelImage.at<int>(r, c);
				cv::Vec3b &pixel = connected_components.at<cv::Vec3b>(r, c);
				pixel = colors[label];
			 }
		}
/*
		cv::imshow( "Connected Components", connected_components );
		cv::waitKey(0);
*/
}

void calc_convexHull(cv::Mat src, cv::Mat& dst)
{
	dst = cv::Mat(src.size(), CV_8U, cv::Scalar(0));

	for(int r = 0; r < dst.rows; ++r){
		int start=-1,end=-1;
		for(int c = 0; c < dst.cols; ++c){
			int pixel_start = src.at<int>(r, c);
			int pixel_end = src.at<int>(r, dst.cols-c-1);
			if (start == -1 && pixel_start!=0)
			{
				start=c;
			}
			if (end == -1 && pixel_end!=0)
			{
				end = dst.cols-c-1;
			}
			if (start != -1 && end != -1) break;
	 	}
		if (start == -1 && end == -1) continue;

		for(int c = start; c <= end; ++c){

/*
			If matrix is of type CV_8U then use Mat.at<uchar>(y,x).
			If matrix is of type CV_8S then use Mat.at<schar>(y,x).
			If matrix is of type CV_16U then use Mat.at<ushort>(y,x).
			If matrix is of type CV_16S then use Mat.at<short>(y,x).
			If matrix is of type CV_32S then use Mat.at<int>(y,x).
			If matrix is of type CV_32F then use Mat.at<float>(y,x).
			If matrix is of type CV_64F then use Mat.at<double>(y,x).
*/
			dst.at<uchar>(r,c) = 255;
		}
	}
/*
	cv::imshow( "Convex Hull", dst );
	cv::waitKey(0);
*/
}


void drawAxis(cv::Mat& img, cv::Point p, cv::Point q, cv::Scalar colour, const float scale = 0.2)
{
    double angle;
    double hypotenuse;
    angle = atan2( (double) p.y - q.y, (double) p.x - q.x ); // angle in radians
    hypotenuse = sqrt( (double) (p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
//    double degrees = angle * 180 / CV_PI; // convert radians to degrees (0-180 range)
//    cout << "Degrees: " << abs(degrees - 180) << endl; // angle in 0-360 degrees range
    // Here we lengthen the arrow by a factor of scale
    q.x = (int) (p.x - scale * hypotenuse * cos(angle));
    q.y = (int) (p.y - scale * hypotenuse * sin(angle));
    cv::line(img, p, q, colour, 1, CV_AA);
    // create the arrow hooks
    p.x = (int) (q.x + 9 * cos(angle + CV_PI / 4));
    p.y = (int) (q.y + 9 * sin(angle + CV_PI / 4));
    cv::line(img, p, q, colour, 1, CV_AA);
    p.x = (int) (q.x + 9 * cos(angle - CV_PI / 4));
    p.y = (int) (q.y + 9 * sin(angle - CV_PI / 4));
    cv::line(img, p, q, colour, 1, CV_AA);
}

void calc_event_mc_and_orientation(cv::Mat event,  double time)
{
    cv::Point cntr(0,0);
	int event_area = 0;
	double angle = 0.0;

	//could use calcCovarMatrix instead to calculate the mass center and the scatter (covariance) matrix
	// using cv::PCA directly crashes system sometimes probably because the number of points is too large

	for(int r = 0; r < event.rows; ++r){
		for(int c = 0; c < event.cols; ++c){
			if (event.at<uchar>(r, c) != 0)
			{
				cntr = cntr + cv::Point(c,r);
				event_area++;
			}	
		 }
	}
	if (event_area == 0)
	{
    	return;
	}

	cntr = cntr/event_area;
    
	cv::Mat scatter_matrix(2,2,CV_64FC1, cv::Scalar(0));
	double& s00 = scatter_matrix.at<double>(0, 0);
	double& s01 = scatter_matrix.at<double>(0, 1);
	double& s10 = scatter_matrix.at<double>(1, 0);
	double& s11 = scatter_matrix.at<double>(1, 1);

	for(int r = 0; r < event.rows; ++r){
		for(int c = 0; c < event.cols; ++c){
			if (event.at<uchar>(r, c) != 0)
			{
				s00 += (c-cntr.x)*(c-cntr.x);
				s01 += (c-cntr.x)*(r-cntr.y);
				s10 += (r-cntr.y)*(c-cntr.x);
				s11 += (r-cntr.y)*(r-cntr.y);
			}	
		 }
	}
	scatter_matrix /= event_area;  // Opencv seems to divide be (event_area-1), but it doesn't matter for the eigen values
	
	cv::Mat eigenvalues, eigenvectors;
	cv::eigen(scatter_matrix, eigenvalues, eigenvectors);

    //Store the eigenvalues and eigenvectors
    std::vector<cv::Point2d> eigen_vecs(2);
    std::vector<double> eigen_val(2);
    for (int i = 0; i < 2; ++i)
    {
        eigen_vecs[i] = cv::Point2d(eigenvectors.at<double>(i, 0),
                                eigenvectors.at<double>(i, 1));
        eigen_val[i] = eigenvalues.at<double>(0, i);
    }
    // Draw the principal components
    cv::circle(event, cntr, 3, cv::Scalar(80), 2);
    cv::Point p1 = cntr + 0.02 * cv::Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]), static_cast<int>(eigen_vecs[0].y * eigen_val[0]));
    cv::Point p2 = cntr - 0.02 * cv::Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]), static_cast<int>(eigen_vecs[1].y * eigen_val[1]));
    drawAxis(event, cntr, p1, cv::Scalar(80), 1);
    drawAxis(event, cntr, p2, cv::Scalar(80), 5);
    angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians

	cv::imshow("event", event);
	cv::waitKey(1);

	std::vector<double> param_value;
	param_value.push_back(cntr.x);
	param_value.push_back(cntr.y);
	ais::g_ais.history.add_event(ais::c_event(time, ais::CENTER_POSITION_EVENT, param_value));
	param_value.clear();
	param_value.push_back(angle);
	ais::g_ais.history.add_event(ais::c_event(time, ais::ORIENTATION_EVENT, param_value));
}

/*
read a training image
apply a feature selecting yellow regions
build predictor: yellow_region(t), joint_command(t) => yellow_region(t+1)
test the predictor on the training samples
command line: OWIFeatCalcPrediction [<training_samples_directory>]   # Without terminating slash. 
May need: export LD_LIBRARY_PATH=/usr/local/lib before running the program
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
	double cur_time = 0.0;

	std::vector<std::string> joint_commands = get_joint_commands();
	
	for (auto& s : joint_commands)
	{
		std::replace( s.begin(), s.end(), ' ', '_'); 
	}

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
	cv::Mat prev_feat_img;

 	for (std::vector<std::string>::iterator it = img_files.begin() ; it != img_files.end(); ++it)
	{
		cv::Mat orig_img, img, img_aux;

		// the image is taken after the command
		cur_time = (double)(OWI_COMMAND_DURATION_MILLISECONDS*get_file_number(*it));

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

		// select yellow pixels
		cv::Mat feat_img;

		// OpenCV channels's order is BGR, not RGB
        // select yellow pixels
		cv::Scalar lower_bound = cv::Scalar(43,81,100);
        cv::Scalar upper_bound = cv::Scalar(102,167,205);
        if (make_hsv)
		{
			// In HSV color space the hue range is [0,360] degrees, but for 8-bit images H stores H/2 to fit 0 to 255
			lower_bound = cv::Scalar(0/2,0.0*255,0.10*255);
	        upper_bound = cv::Scalar(360/2,255,0.35*255);
		}
		cv::inRange(img, lower_bound, upper_bound, feat_img);
/*		 
		cv::imshow("feat_img", feat_img);
		cv::waitKey(0);
*/
		// opencv uses the saturation arithmetics: min(max(round(r),0),255)
		if (!prev_feat_img.empty())
		{
			// add history record for the g_command
			std::size_t found1 = (*it).find("_");
			std::size_t found2 = (*it).find(".");
			std::string cmd_name;

			if (found1 >= 0 && found2 >= 0 && found2 > found1) cmd_name=(*it).substr(found1+1,found2-found1-1);
			for (size_t icmd=0; icmd < joint_commands.size(); icmd++)
			{
				if (joint_commands[icmd]==cmd_name)
				{
					std::vector<double> param_value;
					param_value.push_back(icmd);
					// the image is taken after the command
					double cmd_time=cur_time - (double)OWI_COMMAND_DURATION_MILLISECONDS;
					ais::g_ais.history.add_event(ais::c_event(cmd_time, ais::ACTUATOR_COMMAND_EVENT, param_value));

					break;
				}
			}

			cv::Mat diff_feat_img;
			cv::absdiff(feat_img,prev_feat_img, diff_feat_img);
/*
			cv::imshow("diff_feat_img", diff_feat_img);
			cv::waitKey(0);
*/

			// find convex hull for major connected components
			std::vector<cv::Vec2i> component_areas;
			cv::Mat labelImage;
			get_connected_components(diff_feat_img, labelImage, component_areas);
            int max_num_components=1;
			for (int area_label=1; area_label < std::min(max_num_components+1, (int)component_areas.size()); area_label++)
			{
				cv::Mat component_img = labelImage;
				for(int r = 0; r < labelImage.rows; ++r){
					for(int c = 0; c < labelImage.cols; ++c){
						int label = labelImage.at<int>(r, c);
						if (label != area_label)
							component_img.at<int>(r, c) = 0;
					 }
				}

				cv::Mat component_hull;
				calc_convexHull(component_img, component_hull);
				cv::Mat feat_event;
				cv::bitwise_and(feat_img, component_hull, feat_event);
/*
				cv::imshow("feat_event", feat_event);
				cv::waitKey(0);
*/
				calc_event_mc_and_orientation(feat_event, cur_time);
			}
		}

		prev_feat_img = feat_img;

		// interpret (correctly classify) the observed events by comparing them with the predicted events 
		ais::interpret_observed_events_and_update_prediction_map(cur_time);
	}

#ifdef LOGGING
		ais::g_ais.history.print();
#endif

	return result;
}

