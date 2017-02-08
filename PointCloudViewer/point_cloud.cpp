#include "point_cloud.h"

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

using namespace std;
using namespace Eigen;
using namespace pcv;

namespace pcv {

    bool c_point_cloud::read_point_cloud_file(std::string file_path) {
        string line;
        std::ifstream infile;
        size_t width, height;
        size_t u, v;

        for (int i = 0; i < 3; i++) {
            min_coord(i) = FLT_MAX;
            max_coord(i) = FLT_MIN;
        }

        size_t file_path_size = file_path.size();
        bool b_kinect_file = false;
        if (file_path_size > 4) {
            std::string file_path_ext = file_path.substr(file_path_size - 4, 4);
            transform(file_path_ext.begin(), file_path_ext.end(), file_path_ext.begin(), ::tolower);
            if (file_path_ext == ".kin") {
                b_kinect_file = true;
            }
        }

        infile.open(file_path, std::ifstream::in);

        if (b_kinect_file) {
            std::stringstream linestream;

            std::getline(infile, line);
            linestream << line;
            linestream >> height;

            std::getline(infile, line);
            linestream.clear();
            linestream << line;
            linestream >> width;
        }

        // read points
        while (std::getline(infile, line)) {
            c_point_cloud_point point;
            std::stringstream linestream(line);

            if (b_kinect_file) {
                linestream >> u;
                linestream >> v;
            }

            for (int i = 0; i < 3; i++) {
                linestream >> point.X(i);
            }

            for (int i = 0; i < 3; i++) {
                linestream >> point.Clr(i);
            }

            if (linestream.good()) {
                for (int i = 0; i < 3; i++) {
                    linestream >> point.Vector(i);
                }
            }
            else {
                point.Vector = Vector3f::Zero();
            }

            for (int i = 0; i < 3; i++) {
                if (point.X(i) < min_coord(i)) min_coord(i) = point.X(i);
                if (point.X(i) > max_coord(i)) max_coord(i) = point.X(i);
            }
            point.visible = 1;
            points.push_back(point);
        }
        infile.close();

        return true;
    }

    bool c_point_cloud::write_point_cloud_file(std::string file_path) {
        // read points from xyz file
        string line;
        std::ofstream outfile;
        outfile.open(file_path, std::ifstream::out);

        for (auto it = points.begin(); it != points.end(); ++it) {
            if (!it->visible)
                continue;

            outfile << it->X(0) << " ";
            outfile << it->X(1) << " ";
            outfile << it->X(2) << " ";
            outfile << it->Clr(0) << " ";
            outfile << it->Clr(1) << " ";

            if (it->Vector != Vector3f::Zero()) {
                outfile << it->Clr(2) << " ";
                outfile << it->Vector(0) << " ";
                outfile << it->Vector(1) << " ";
                outfile << it->Vector(2) << std::endl;
            }
            else {
                outfile << it->Clr(2) << std::endl;
            }
        }

        outfile.close();
        return true;
    }

    void c_point_cloud::reset_visibility() {
        for (auto it = points.begin(); it != points.end(); ++it) {
            it->visible = true;
        }
    }
}