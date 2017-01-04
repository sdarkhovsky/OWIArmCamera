#include "kinect_image.h"

#include <iostream>
#include <fstream>
#include <string>

using namespace std;

namespace ais {

bool c_kinect_image::read_file(std::string file_path, c_point_cloud& point_cloud)
{
    string line;
    ifstream infile;
    int width, height;
    size_t i, u, v;
    infile.open(file_path, std::ifstream::in);
    infile >> width;
    infile >> height;
    point_cloud.points.clear();
    point_cloud.points.resize(height);
    for (size_t i = 0; i < height; i++) {
        point_cloud.points[i].resize(width);
    }

    while (!infile.eof()) 
    {
        infile >> u;
        infile >> v;
        if (u < height && v < width) {
            infile >> point_cloud.points[u][v].X;
            infile >> point_cloud.points[u][v].Y;
            infile >> point_cloud.points[u][v].Z;
            infile >> point_cloud.points[u][v].Blue;
            infile >> point_cloud.points[u][v].Green;
            infile >> point_cloud.points[u][v].Red;
        }
    }
    infile.close();
	return true;
}

}
