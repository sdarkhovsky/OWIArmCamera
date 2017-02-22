#include "kinect_image.h"

#include <iostream>
#include <fstream>
#include <string>

#include "png_visualize.h"

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

        Vector3f X, Clr;
        while (!infile.eof())
        {
            infile >> u;
            infile >> v;
            infile >> X(0);
            infile >> X(1);
            infile >> X(2);
            infile >> Clr(0);
            infile >> Clr(1);
            infile >> Clr(2);

            if (u < height && v < width) {
                point_cloud.points[u][v].X = X;
                point_cloud.points[u][v].Clr = Clr;
                point_cloud.points[u][v].uv = Vector2i(u, v);
            }
        }
        infile.close();

        point_cloud.fix();

#if 1
        std::string png_file_path = file_path + ".png";
        png_visualize_point_cloud(png_file_path.c_str(), point_cloud);
#endif

        return true;
    }


    void c_kinect_image::get_cloud_point_color(c_point_cloud_point& cloud_point, Vector3f& Clr) {
        if (cloud_point.Clr_edge != 0) {
            if (cloud_point.edge_corner_angle_cos > corner_angle_cosine_thresh)
                Clr = Vector3f(0, 255, 0);
            else
                Clr = Vector3f(255, 0, 0);
        }
        else {
            Clr = cloud_point.Clr;
        }
    }

    bool c_kinect_image::write_file(std::string file_path, c_point_cloud& point_cloud, c_image_format image_format)
    {
        string line;
        ofstream outfile;
        size_t height = point_cloud.points.size();
        size_t width = point_cloud.points[0].size();
        size_t u, v;
        Vector3f Clr;

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
                get_cloud_point_color(point_cloud.points[u][v], Clr);

                if (image_format == c_image_format::kinect) {
                    outfile << u << " ";
                    outfile << v << " ";
                }
                outfile << point_cloud.points[u][v].X(0) << " ";
                outfile << point_cloud.points[u][v].X(1) << " ";
                outfile << point_cloud.points[u][v].X(2) << " ";
                outfile << Clr(0) << " ";
                outfile << Clr(1) << " ";
                if (image_format == c_image_format::xyze && point_cloud.points[u][v].Vector != Vector3f::Zero()) {
                    outfile << Clr(2) << " ";
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
        png_visualize_point_cloud(png_file_path.c_str(), point_cloud);
    #endif

        return true;
    }

}
