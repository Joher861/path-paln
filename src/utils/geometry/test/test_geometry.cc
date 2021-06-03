#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

#include <iostream>

#include "area.h"
#include "geometry_func.h"

using namespace planning_utils;
using namespace std;

Pose gridToPose(const Grid &grid)
{
    return Pose{grid.x * 1.0f, grid.y * 1.0f};
}

Grid poseToGrid(const Pose &pose)
{
    return Grid{
        static_cast<int32_t>(roundf(pose.pt.x / 1.0f)),
        static_cast<int32_t>(roundf(pose.pt.y / 1.0f))
    };
}

int main()
{
    // // test 1
    // Contour contour{std::vector<Grid>{{0, 0}, {0, 25}, {25, 25}, {25, 0}}};
    // Contour inner_contour1{std::vector<Grid>{{6, 6}, {15, 6}, {15, 15}, {6, 15}}, Contour::INNER_CONTOUR};
    // Contour inner_contour2{std::vector<Grid>{{24, 13}, {20, 18}, {18, 11}}, Contour::INNER_CONTOUR};
    // AreaContour area_contour{std::move(contour)};
    // area_contour.addInnerContour(std::move(inner_contour1));
    // area_contour.addInnerContour(std::move(inner_contour2));

    // // inner_contour2.print();
    // cv::Mat && mat = area_contour.getMat();
    // GridRect rect = area_contour.outer_contour.getRect();
    // GridSet grids;
    // fillAreaContour(area_contour, grids);
    // for (auto & grid : grids)
    // {
    //     mat.at<uint8_t>(rect.max_grid.x - grid.x,
    //         rect.max_grid.y - grid.y) = 128;
    // }
    // cv::imwrite("/tmp/contour.bmp", mat);

    // // test 2
    // Contour contour{std::vector<Grid>{{0, 0}, {0, 25}, {5, 25}, {5, 10}, {15, 10}, {15, 25}, {25, 25}, {25, 0}}};
    // contour.print();
    // cv::Mat && mat = contour.getMat();

    // GridRect rect = contour.getRect();
    // GridSet grids;
    // fillContour(contour, grids);
    // for (auto & grid : grids)
    // {
    //     mat.at<uint8_t>(rect.max_grid.x - grid.x,
    //         rect.max_grid.y - grid.y) = 128;
    // }
    // cv::imwrite("/tmp/contour.bmp", mat);

    // // test 3
    // cv::Mat input = cv::imread("Pictures/test.bmp");
    // std::vector<AreaContour> area_contours;
    // generateContourFromMat(input, [](uint8_t val) { return val < 10; },
    //     GridRect{{0, 0}, {input.rows - 1, input.cols - 1}}, area_contours);

    // int cnt = 0;
    // for (auto & area_contour : area_contours)
    // {
    //     cv::Mat && mat = area_contour.getMat();
    //     // std::string name = "/tmp/contour_" + (std::to_string(cnt++)) + ".bmp";
    //     // cv::imwrite(name.c_str(), mat);
    //     GridRect rect = area_contour.outer_contour.getRect();
    //     GridSet grids;
    //     fillAreaContour(area_contour, grids);
    //     for (auto & grid : grids)
    //     {
    //         mat.at<uint8_t>(rect.max_grid.x - grid.x,
    //             rect.max_grid.y - grid.y) = 128;
    //     }
    //     std::string name = "/tmp/contour_" + (std::to_string(cnt++)) + ".bmp";
    //     cv::imwrite(name.c_str(), mat);
    // }

    // // test 4 area erode, dilate, getFilledMat

    // system("mkdir -p /tmp/test");

    // cv::Mat input = cv::imread("Pictures/test.bmp");
    // std::vector<AreaContour> area_contours;
    // generateContourFromMat(input, [](uint8_t val) { return val < 10; },
    //     GridRect{{0, 0}, {input.rows - 1, input.cols - 1}}, area_contours);

    // cout << area_contours.size() << endl;

    // int cnt = 0;
    // for (auto &area_contour : area_contours)
    // {
    //     string area_name = "area_" + to_string(cnt);
    //     Area area(area_contour);
    //     cout << area_name << " center = " << area.getCenter().toString() << endl;
    //     auto &&erode_areas = area.erode(5);
    //     int cnt2 = 0;
    //     for (auto &&erode_area : erode_areas)
    //     {
    //         string name = "erode_" + to_string(cnt) + "_" + to_string(cnt2++);
    //         auto && [mat, rect] = erode_area.getFilledMat();
    //         cout << name << " rect = " << rect.toString() << endl;
    //         string map_name = "/tmp/test/" + name + "_map.bmp";
    //         cv::imwrite(map_name.c_str(), mat);
    //     }

    //     auto &&dilate_area = area.dilate(5);
    //     cnt2 = 0;
    //     {
    //         string name = "dilate_" + to_string(cnt) + "_" + to_string(cnt2++);
    //         auto && [mat, rect] = dilate_area.getFilledMat();
    //         cout << name << " rect = " << rect.toString() << endl;
    //         string map_name = "/tmp/test/" + name + "_map.bmp";
    //         cv::imwrite(map_name.c_str(), mat);
    //     }

    //     cnt++;
    // }

    // // test 5 test transform contour, area contour, area

    // system("mkdir -p /tmp/test");

    // auto grid_to_pose = gridToPose;
    // auto pose_to_grid = poseToGrid;

    // cv::Mat input = cv::imread("Pictures/test.bmp");
    // std::vector<AreaContour> area_contours;
    // generateContourFromMat(input, [](uint8_t val) { return val < 10; },
    //     GridRect{{0, 0}, {input.rows - 1, input.cols - 1}}, area_contours);

    // cout << area_contours.size() << endl;

    // int cnt = 0;
    // for (auto &area_contour : area_contours)
    // {
    //     string area_name = "area_" + to_string(cnt);
    //     Area area(area_contour);

    //     float clean_dir = deg2rad(10);

    //     Pose center_pose = gridToPose(area.getCenter());
    //     center_pose.theta = clean_dir;
    //     Pose transform = reverseTransform(getTransform(Pose{}, center_pose));
    //     cout << "transform = " << transform.toString() << "theta = "
    //         << rad2deg(transform.theta) << endl;

    //     auto &&new_area = transformArea(transform, area,
    //         grid_to_pose, pose_to_grid);
    //     auto && [mat, rect] = new_area.getFilledMat();
    //     cout << "center = " << new_area.getCenter() << endl;
    //     cout << "rect = " << new_area.rect.toString() << endl;
    //     // auto &&mat = contour.getMat();
    //     cv::imwrite("/tmp/test/trans_contour_" + to_string(cnt++) + ".bmp", mat);
    // }

    Contour contour{std::vector<Grid>{{-20, -20}, {20, -20}, {20, 20}, {-20, 20}}};
    Grid start_grid{20, 1};
    Grid end_grid{20, 4};
    auto && cgs = contour.getPart(start_grid, end_grid, Contour::CCW);
    printf("cgs size = %lu", cgs.size());

    return 0;
}