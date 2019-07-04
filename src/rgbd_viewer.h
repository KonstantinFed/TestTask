////////////////////////////////////////////////////////////////////////////////
//
// rgbd_viewer.h
//
// Structures and methods for conversion RGB-D data to axonometry.
//
// author: Konstantin Fedorov

#ifndef RGBD_VIEWER_H_
#define RGBD_VIEWER_H_

#include <stdint.h>
#include <vector>

#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>


// 3D point metric coordinates
struct point3d
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};


// Load depth map from file
bool load_depth_map(std::vector<std::vector<uint16_t>> &depth_map,
    const char* filename);

// Fills 2-dimensional array of coordinates according to depth map
void calculate_cartesian(std::vector<std::vector<point3d>> &coords, 
    const std::vector<std::vector<uint16_t>> &depth_map, 
    const int cols,
    const int rows);

// Draw axonometry on image according to cartesian coordinates
void create_axonometry(cv::Mat &out, 
    const std::vector<std::vector<point3d>> &coords, 
    const cv::Mat &src_colors);


#endif // RGBD_VIEWER_H_