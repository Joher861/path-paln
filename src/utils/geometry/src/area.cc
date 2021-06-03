#include "area.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include "geometry_func.h"

using namespace planning_utils;

Area::Area(const AreaContour &_contour)
    : contour(_contour)
{
    fillAreaContour(contour, grids);
    rect = contour.outer_contour.getRect();
}

Area::Area(std::vector<Grid> &outer_contour_grids)
    : contour(std::move(outer_contour_grids))
{
    fillAreaContour(contour, grids);
    rect = contour.outer_contour.getRect();
}

Grid Area::getCenter() 
{
    return (rect.max_grid + rect.min_grid) * 0.5f;
}

Grid Area::getCenter() const
{
    return (rect.max_grid + rect.min_grid) * 0.5f;
}

std::pair<cv::Mat, GridRect> Area::getFilledMat(int padding)
{
    cv::Mat &&mat = contour.getMat();

    for (auto &grid : grids)
    {
        mat.at<uint8_t>(rect.max_grid.x - grid.x,
            rect.max_grid.y - grid.y) = 255;
    }

    if (padding == 0)
    {
        return std::make_pair(mat, rect);
    }
    else
    {
        Grid offset{padding, padding};
        GridRect new_rect{rect.min_grid - offset, rect.max_grid + offset};
        Grid size = new_rect.getSize();
        cv::Mat new_mat(size.x, size.y, CV_8UC1, cv::Scalar(0));
        
        int rows = mat.rows;
        int cols = mat.cols;
        uint8_t *row;
        uint8_t *new_row;
        for (int i = 0; i < rows; i++)
        {
            row = mat.ptr<uint8_t>(i);
            new_row = new_mat.ptr<uint8_t>(i + padding);
            memcpy(new_row + padding, row, sizeof(uint8_t) * cols);
        }

        return std::make_pair(new_mat, new_rect);
    }
}

std::vector<Area> Area::erode(int dist)
{
    auto && [mat, rect] = getFilledMat(dist);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
        cv::Size(2 * dist - 1, 2 * dist - 1));
    cv::erode(mat, mat, kernel);

    std::vector<AreaContour> area_contours;
    generateContourFromMat(mat, [](uint8_t val) { return val == 255; }, rect,
        area_contours);
    
    std::vector<Area> new_areas;
    for (auto &area_contour : area_contours)
    {
        new_areas.emplace_back(Area(area_contour));
    }
    return new_areas;
}

Area Area::dilate(int dist)
{
    auto && [mat, rect] = getFilledMat(dist);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
        cv::Size(2 * dist - 1, 2 * dist - 1));
    cv::dilate(mat, mat, kernel);

    std::vector<AreaContour> area_contours;
    generateContourFromMat(mat, [](uint8_t val) { return val == 255; }, rect,
        area_contours);
    
    if (area_contours.size() != 1)
    {
        std::string err_str = "area should not split after dilation, cur size = "
            + std::to_string(area_contours.size());
        throw std::runtime_error(err_str.c_str());
    }
    
    return Area(area_contours.front());
}