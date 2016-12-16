#ifndef KINECT_IMAGE_H
#define KINECT_IMAGE_H

#include <vector>
#include <string>

namespace ais {

class c_kinect_image
{
public:
	c_kinect_image() {
	}
    
    bool read_file( std::string file_path );
};


}

#endif

