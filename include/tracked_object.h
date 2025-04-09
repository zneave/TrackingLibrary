#ifndef TRACKED_OBJECT_H
#define TRACKED_OBJECT_H

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include "detection.h"
#include "filter.h"
#include "FilterFactory.h"

class _TrackedObjectFactory;
class CoordinatesTransformation;

class TrackedObject {
public:
    TrackedObject(_TrackedObjectFactory* obj_factory,
                  const Detection& initial_detection,
                  int hit_counter_max,
                  int initialization_delay,
                  int pointwise_hit_counter_max,
                  double detection_threshold,
                  int period,
                  FilterFactory* filter_factory,
                  int past_detections_length,
                  int reid_hit_counter_max,
                  const CoordinatesTransformation* coord_transformations = nullptr);

    void tracker_step();
    bool is_active() const;
    void hit(const Detection& detection, int period = 1);
    void update_coordinate_transformation(const CoordinatesTransformation& coordinate_transformation);
    const Eigen::MatrixXd& get_estimate(bool absolute = false) const;

    int get_id() const;
    void assign_id(int new_id);
    std::vector<cv::Point> trail;
    static constexpr size_t max_trail_length = 20;

private:
    void _conditionally_add_to_past_detections(const Detection& detection);
    void _acquire_ids();

    _TrackedObjectFactory* obj_factory;
    Eigen::MatrixXd estimate;
    int id = -1;
    int global_id;
    int hit_counter;
    int hit_counter_max;
    int pointwise_hit_counter_max;
    int initialization_delay;
    double detection_threshold;
    int period;
    int reid_hit_counter_max;
    int age;
    bool is_initializing;
    Eigen::MatrixXd absolute_points;
    std::vector<int> point_hit_counter;
    std::shared_ptr<Filter> filter;
    std::vector<Detection> past_detections;
    int past_detections_length;
    CoordinatesTransformation* coord_transformations;
};

#endif // TRACKED_OBJECT_H
