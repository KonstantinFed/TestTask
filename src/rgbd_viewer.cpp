////////////////////////////////////////////////////////////////////////////////
//
// rgbd_viewer.cpp
//
// Implementation of rgbd_viewer.h.
//
// author: Konstantin Fedorov

#include "rgbd_viewer.h"

#define _USE_MATH_DEFINES

#include <algorithm>
#include <math.h>
#include <stdio.h>

#include <opencv2/highgui/highgui.hpp>


// Camera presets
const float FOV_hor = 58.f * M_PI / 180.f;
const float FOV_hor_2 = FOV_hor / 2.f;
const float FOV_vert = 45.f * M_PI / 180.f;
const float FOV_vert_2 = FOV_vert / 2.f;
// Output image settings
const int ImageWidth = 1024;
const int ImageHeight = 768;
#define AXONOMETRIC_SCALE (0.125f)


bool load_depth_map(std::vector<std::vector<uint16_t>> &depth_map,
    const char* filename)
{
    FILE *file = fopen(filename, "rb");
    if (!file)
        return false;

    int32_t width = 0;
    int32_t height = 0;
    if (fread(&height, sizeof(height), 1, file) &&
        fread(&width, sizeof(width), 1, file))
    {
        if (height && width)
        {
            fseek(file, 0, SEEK_END);
            if (ftell(file) >= sizeof(height) + sizeof(width) + 
                sizeof(uint16_t) * width * height)
            {
                fseek(file, sizeof(height) + sizeof(width), SEEK_SET);
                depth_map.resize(height, std::vector<uint16_t>(width));

                for (int h = 0; h < height; h++)
                    fread(depth_map[h].data(), sizeof(uint16_t), width, file);
                
                fclose(file);
                return true;
            }
        }
    }
    fclose(file);
    return false;
}

inline point3d spherical_to_cartesian(const float radius,
    const float teta,
    const float fi)
{
    point3d point;
    point.x = radius * std::sinf(teta) * std::cosf(fi);
    point.y = radius * std::sinf(teta) * std::sinf(fi);
    point.z = radius * std::cosf(teta);
    return point;
}

void calculate_cartesian(std::vector<std::vector<point3d>> &coords,
    const std::vector<std::vector<uint16_t>> &depth_map,
    const int cols,
    const int rows)
{
    coords.resize(rows, std::vector<point3d>(cols));

    int center_width = (cols + 1) / 2;
    int center_height = (rows + 1) / 2;
    float add_w = cols % 2 ? 0.f : 0.5f;
    float add_h = rows % 2 ? 0.f : 0.5f;
    
    float dist_to_camera = (float)center_width / std::tanf(FOV_hor_2);
    
    std::vector<float> teta(rows);
    for (int i = 0; i < center_height; i++)
    {
        float angle = std::atan2f(center_height - i - add_h, dist_to_camera);
        teta[i] = M_PI_2 - angle;
        teta[rows - 1 - i] = M_PI_2 + angle;
    }
    std::vector<float> fi(cols);
    for (int i = 0; i < center_width; i++)
    {
        float angle = std::atan2f(center_width - i - add_w, dist_to_camera);
        fi[i] = M_PI_4 - angle;
        fi[cols - 1 - i] = M_PI_4 + angle;
    }

    for (int r = 0; r < rows; r++)
        for (int c = 0; c < cols; c++)
        {
            float radius = std::sqrtf(std::powf(r - rows / 2.f, 2.f) +
                std::powf(c - cols / 2.f, 2.f));
            float real_length = (float)depth_map[r][c] /
                std::cosf(std::atan2f(radius, dist_to_camera));
            coords[r][c] = spherical_to_cartesian(real_length, teta[r], fi[c]);
        }
}

void create_axonometry(cv::Mat &out,
    const std::vector<std::vector<point3d>> &coords,
    const cv::Mat &src_colors)
{
    out = cv::Mat(ImageHeight, ImageWidth, CV_8UC3, cv::Scalar(0, 0, 0));

    int rows = coords.size();
    int cols = coords[0].size();

    for (int r = rows - 1; r >= 0; r--)
        for (int c = 0; c < cols; c++)
        {
            cv::Vec3b color = src_colors.at<cv::Vec3b>(cv::Point(c, r));

            const point3d &p = coords[r][c];            
            float x = (p.y - p.x) * std::cosf(M_PI / 6);
            float y = -p.z + -(p.x + p.y) * std::sinf(M_PI / 6); 
            x *= AXONOMETRIC_SCALE;
            y *= AXONOMETRIC_SCALE;
            x += ImageWidth / 2;
            y += ImageHeight;

            if (x >= 0 && x < ImageWidth && y >= 0 && y < ImageHeight)
                out.at<cv::Vec3b>(cv::Point(x, y)) = color;
        }
}