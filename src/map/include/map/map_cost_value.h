#ifndef MAP_2D_COST_VALUES_H_
#define MAP_2D_COST_VALUES_H_
/** Provides a mapping for often used cost values */
namespace planning_map
{
    // static const unsigned char NO_INFORMATION = 255;
    // static const unsigned char LETHAL_OBSTACLE = 254;
    // static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
    // static const unsigned char FREE_SPACE = 0;

    static const unsigned char UNKNOWN = 255;
    static const unsigned char FREE = 0;
    static const unsigned char OCCUPIED = 255;
    static const unsigned char OBSTACLE = 255;
    static const unsigned char INFLATE = 254;

} // namespace planning_map
#endif // MAP_2D_COST_VALUES_H_