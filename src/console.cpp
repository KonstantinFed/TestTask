////////////////////////////////////////////////////////////////////////////////
//
// console.cpp
//
// Console application for work ind with RGB-D images.
//
// author: Konstantin Fedorov

#define _CRT_SECURE_USE_NO_WARNINGS

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "rgbd_viewer.h"


// Settings from command line
struct user_settings
{
    std::string in_img;       // Input image filename
    std::string in_depth_map; // Input depth map filename
    std::string out_img;      // Output image filename

    // Check that all settings are set
    bool is_filled()
    {
        return in_img.size() && in_depth_map.size() && out_img.size();
    }
};

// Fill user_settings with values from command line args
bool parse_args(user_settings &settings, const int argc, const char* argv[])
{
    if (argc == 1)
    {
        printf("RGB-D Viewer\n");
        printf("-in_rgb [filename] - Input image with RGB info\n");
        printf("-in_dm [filename]  - Input depth map\n");
        printf("-out [filename]    - Output image with axonometry\n");
        return false;
    }

    int cur_arg = 1;
    while (cur_arg < argc)
    {
        if (!strcmp("-in_rgb", argv[cur_arg]))
        {
            if ((cur_arg + 1) >= argc)
                return false;
            settings.in_img = argv[cur_arg + 1];
            cur_arg += 2;
            continue;
        }
        if (!strcmp("-in_dm", argv[cur_arg]))
        {
            if ((cur_arg + 1) >= argc)
                return false;
            settings.in_depth_map = argv[cur_arg + 1];
            cur_arg += 2;
            continue;
        }
        if (!strcmp("-out", argv[cur_arg]))
        {
            if ((cur_arg + 1) >= argc)
                return false;
            settings.out_img = argv[cur_arg + 1];
            cur_arg += 2;
            continue;
        }

        printf("Unknown option - %s\n", argv[cur_arg]);
        return false;
    }

    return settings.is_filled();
}

void on_errors_occured(char *message)
{
    printf(message);
    exit(EXIT_FAILURE);
}

int main(int argc, const char* argv[])
{
    user_settings settings;
    bool success = parse_args(settings, argc, argv);
    if (!success)
        on_errors_occured("Incorrect command line arguments\n");

    cv::Mat rgb;
    rgb = cv::imread(settings.in_img, CV_LOAD_IMAGE_UNCHANGED);
    if (!rgb.data)
        on_errors_occured("Can't open image\n");

    std::vector<std::vector<uint16_t>> depth_map;
    success = load_depth_map(depth_map, settings.in_depth_map.c_str());
    if (!success)
        on_errors_occured("Can't open depth map\n");

    std::vector<std::vector<point3d>> coords;
    calculate_cartesian(coords, depth_map, depth_map[0].size(),
        depth_map.size());

    cv::Mat axonometry;
    create_axonometry(axonometry, coords, rgb);

    cv::imwrite(settings.out_img, axonometry);
}