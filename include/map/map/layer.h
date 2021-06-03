#ifndef MAP_LAYER_H
#define MAP_LAYER_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include "map_base_info.h"

namespace planning_map
{
    class Layer;
    using LayerPtr = std::shared_ptr<Layer>;
    class Layer
    {
    public:
        Layer() {}
        virtual ~Layer() {}

        // virtual void reset() {}
        // virtual void reset(const RobotGridRect &range) {}

        // virtual bool inRange(const Point &pt) { return true; }
        // virtual bool inRange(const RobotGrid &grid) { return true; }
        // virtual RobotGrid poseToGrid(const RobotPose &pose) { return RobotGrid{}; }
        // virtual RobotGrid poseToGridWithoutBoundary(const RobotPose &pose)
        // {
        //     return RobotGrid{};
        // }
        // virtual RobotPose gridToPose(const RobotGrid &grid) { return RobotPose{}; }
        // virtual cv::Mat saveMapToMat(RobotGridRect region, MapSaveOption opt,
        //     uint8_t channel = CV_8UC1, uint32_t padding = 5) { return cv::Mat{}; }
        // virtual bool loadMapFromMat(const RobotGridRect &region, const cv::Mat &map,
        //     uint8_t channel = CV_8UC1) { return false; }

        virtual void reset() = 0;
        virtual void reset(const RobotGridRect &range) = 0;

        void setMapFromLayer(const RobotGridRect &dest_region,
                             LayerPtr layer, const RobotGridRect &src_region);
        virtual bool inRange(const Point &pt) = 0;
        virtual bool inRange(const RobotGrid &grid) = 0;
        virtual RobotGrid poseToGrid(const RobotPose &pose) = 0;
        virtual RobotGrid poseToGridWithoutBoundary(const RobotPose &pose) = 0;
        virtual RobotPose gridToPose(const RobotGrid &grid) = 0;
        virtual cv::Mat saveMapToMat(RobotGridRect region, MapSaveOption opt,
                                     uint8_t channel = CV_8UC1, uint32_t padding = 5) = 0;
        virtual bool loadMapFromMat(const RobotGridRect &region, const cv::Mat &map,
                                    uint8_t channel = CV_8UC1) = 0;
    };
} // namespace planning_map

#endif // MAP_LAYER_H