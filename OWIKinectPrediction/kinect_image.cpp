#include "kinect_image.h"

#include <iostream>
#include <fstream>
#include <string>

#define DEBUGGING

#ifdef DEBUGGING
bool write_png_file(const char* file_name, ais::c_point_cloud& point_cloud);
#endif

using namespace std;

namespace ais {

bool c_kinect_image::read_file(std::string file_path, c_point_cloud& point_cloud)
{
    string line;
    ifstream infile;
    size_t width, height;
    size_t i, u, v;
    infile.open(file_path, std::ifstream::in);
    infile >> height;
    infile >> width;
    point_cloud.points.clear();
    point_cloud.points.resize(height);
    for (i = 0; i < height; i++) {
        point_cloud.points[i].resize(width);
    }

    while (!infile.eof()) 
    {
        infile >> u;
        infile >> v;
        if (u < height && v < width) {
            infile >> point_cloud.points[u][v].X(0);
            infile >> point_cloud.points[u][v].X(1);
            infile >> point_cloud.points[u][v].X(2);
            infile >> point_cloud.points[u][v].Clr(0);
            infile >> point_cloud.points[u][v].Clr(1);
            infile >> point_cloud.points[u][v].Clr(2);
        }
    }
    infile.close();

#if 1
    for (u = 0; u < height; u++) {
        for (v = 0; v < width; v++) {
            point_cloud.points[u][v].X = Vector3f(u, v, 1.0);
            if (u < height/2)
                point_cloud.points[u][v].Clr = Vector3f(0, 0, 255);
            else
                point_cloud.points[u][v].Clr = Vector3f(0, 255, 0);
        }
    }

    std::string png_file_path = file_path + ".png";
    write_png_file(png_file_path.c_str(), point_cloud);
#endif

	return true;
}

bool c_kinect_image::write_file(std::string file_path, c_point_cloud& point_cloud, c_image_format image_format)
{
    string line;
    ofstream outfile;
    size_t height = point_cloud.points.size();
    size_t width = point_cloud.points[0].size();
    size_t u, v;

    outfile.open(file_path, std::ifstream::out);

    if (image_format == c_image_format::kinect) {
        outfile << height << std::endl;
        outfile << width << std::endl;
    }
    
    for (u = 0; u < height; u++) {
        assert(point_cloud.points[u].size() == width);
        for (v = 0; v < width; v++) {
            if (point_cloud.points[u][v].X == Vector3f::Zero()) // the input .kin files have no points for some u,v
                continue;
            if (image_format == c_image_format::kinect) {
                outfile << u << " ";
                outfile << v << " ";
            }
            outfile << point_cloud.points[u][v].X(0) << " ";
            outfile << point_cloud.points[u][v].X(1) << " ";
            outfile << point_cloud.points[u][v].X(2) << " ";
            outfile << point_cloud.points[u][v].Clr(0) << " ";
            outfile << point_cloud.points[u][v].Clr(1) << " ";
            if (image_format == c_image_format::xyze && point_cloud.points[u][v].Vector != Vector3f::Zero()) {
                outfile << point_cloud.points[u][v].Clr(2) << " ";
                outfile << point_cloud.points[u][v].Vector(0) << " ";
                outfile << point_cloud.points[u][v].Vector(1) << " ";
                outfile << point_cloud.points[u][v].Vector(2) << std::endl;
            }
            else {
                outfile << point_cloud.points[u][v].Clr(2) << std::endl;
            }
        }
    }

    outfile.close();

#if 1
    std::string png_file_path = file_path + ".png";
    write_png_file(png_file_path.c_str(), point_cloud);
#endif

    return true;
}


}
