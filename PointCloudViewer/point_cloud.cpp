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
        for (int i = 0; i < 3; i++) {
            min_coord(i) = FLT_MAX;
            max_coord(i) = FLT_MIN;
        }

        // read points from xyz file
        string line;
        std::ifstream infile;
        infile.open(file_path, std::ifstream::in);
        while (std::getline(infile, line)) {
            c_point_cloud_point point;

            std::stringstream linestream(line);
            for (int i = 0; i < 3; i++) {
                linestream >> point.X(i);
            }

            for (int i = 0; i < 3; i++) {
                linestream >> point.Clr(i);
            }

            for (int i = 0; i < 3; i++) {
                linestream >> point.Edge(i);
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

    void c_point_cloud::reset_visibility() {
        for (auto it = points.begin(); it != points.end(); ++it) {
            it->visible = true;
        }
    }
}