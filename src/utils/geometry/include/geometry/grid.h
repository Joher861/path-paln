#ifndef UTILS_GRID_H
#define UTILS_GRID_H

#include <cinttypes>
#include <ostream>
#include <cmath>
#include <string>

#include "point.h"
// #include "pose.h"

namespace planning_utils
{
    struct Point;
    // struct Pose;
    struct Grid
    {
        int32_t x;
        int32_t y;

        Grid();
        Grid(int32_t x, int32_t y);
        Grid(const Point &pt);
        // Grid(const Pose &pose);

        std::string toString() const;
        const char* toCString() const;

        friend std::ostream &operator<<(std::ostream &out, const Grid &pt);
        friend bool operator==(const Grid &grid1, const Grid &grid2);
        friend bool operator!=(const Grid &grid1, const Grid &grid2);
        friend Grid operator-(const Grid &grid1, const Grid &grid2);
        friend Grid operator+(const Grid &grid1, const Grid &grid2);
        friend Grid operator*(const Grid &grid, float factor);
        friend Grid operator/(const Grid &grid, float factor);
        friend bool operator<(const Grid &g1, const Grid &g2);
    };

    inline std::string Grid::toString() const
    {
        static std::string str = "";
        str = "{ "
            + std::to_string(x)
            + ", "
            + std::to_string(y)
            + " }";
        return str;
    }

    inline const char* Grid::toCString() const
    {
        return toString().c_str();
    }

    class GridHash
    {
      public: 
        size_t operator()(const Grid& grid) const;
    };
}

#endif // UTILS_GRID_H