#ifndef LOCALMAP_2D_FOOTPRINT_H
#define LOCALMAP_2D_FOOTPRINT_H


#include "misc/planning_typedefs.h"


/**
 * @brief Calculate the extreme distances for the footprint
 *
 * @param footprint The footprint to examine
 * @param min_dist Output parameter of the minimum distance
 * @param max_dist Output parameter of the maximum distance
 */
void calculateMinAndMaxDistances(const std::vector<Point>& footprint,
                                 double& min_dist, double& max_dist);

/**
 * @brief  Given a pose and base footprint, build the oriented footprint of the robot (list of Points)
 * @param  x The x position of the robot
 * @param  y The y position of the robot
 * @param  theta The orientation of the robot
 * @param  footprint_spec Basic shape of the footprint
 * @param  oriented_footprint Will be filled with the points in the oriented footprint of the robot
*/
void transformFootprint(double x, double y, double theta, const std::vector<Point>& footprint_spec,
                        std::vector<Point>& oriented_footprint);

/**
 * @brief  Given a pose and base footprint, build the oriented footprint of the robot (PolygonStamped)
 * @param  x The x position of the robot
 * @param  y The y position of the robot
 * @param  theta The orientation of the robot
 * @param  footprint_spec Basic shape of the footprint
 * @param  oriented_footprint Will be filled with the points in the oriented footprint of the robot
*/
void transformFootprint(double x, double y, double theta, const std::vector<Point>& footprint_spec,
                        std::vector<Point> & oriented_footprint);

/**
 * @brief Adds the specified amount of padding to the footprint (in place)
 */
void padFootprint(std::vector<Point>& footprint, double padding);

/**
 * @brief Create a circular footprint from a given radius
 */
std::vector<Point> makeFootprintFromRadius(double radius);

/**
 * @brief Make the footprint from the given string.
 *
 * Format should be bracketed array of arrays of floats, like so: [[1.0, 2.2], [3.3, 4.2], ...]
 *
 */
bool makeFootprintFromString(const std::string& footprint_string, std::vector<Point>& footprint);




#endif  // COSTMAP_2D_FOOTPRINT_H
