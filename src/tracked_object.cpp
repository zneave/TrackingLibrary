#include "tracked_object.h"
#include "coordinates_transformation.h"
#include "filter.h"

static int global_id_counter = 0;

TrackedObject::TrackedObject(_TrackedObjectFactory* obj_factory,
                             const Detection& initial_detection,
                             int hit_counter_max,
                             int initialization_delay,
                             int pointwise_hit_counter_max,
                             double detection_threshold,
                             int period,
                             FilterFactory* filter_factory,
                             int past_detections_length,
                             int reid_hit_counter_max,
                             const CoordinatesTransformation* coord_transformations)
    : obj_factory(obj_factory),
      hit_counter_max(hit_counter_max),
      pointwise_hit_counter_max(pointwise_hit_counter_max),
      detection_threshold(detection_threshold),
      period(period),
      reid_hit_counter_max(reid_hit_counter_max),
      past_detections_length(past_detections_length),
      coord_transformations(const_cast<CoordinatesTransformation*>(coord_transformations)),
      is_initializing(true),
      hit_counter(hit_counter_max),
      age(0),
      id(-1), global_id(-1) {

    absolute_points = initial_detection.get_absolute_points();
    estimate = initial_detection.get_points();
}

void TrackedObject::tracker_step() {
    if (filter) {
        filter->predict();
    }
    hit_counter -= 1;

    for (auto& val : point_hit_counter) {
        val = std::max(0, val - 1);
    }
}

bool TrackedObject::is_active() const {
    return hit_counter >= 0;
}

void TrackedObject::hit(const Detection& detection, int period) {
    estimate = detection.get_points();
    hit_counter = hit_counter_max;
    _conditionally_add_to_past_detections(detection);
}

void TrackedObject::update_coordinate_transformation(const CoordinatesTransformation& coordinate_transformation) {
    if (coord_transformations != nullptr) {
        absolute_points = coordinate_transformation.rel_to_abs(absolute_points);
    }
}

void TrackedObject::_conditionally_add_to_past_detections(const Detection& detection) {
    past_detections.push_back(detection);
    if (past_detections.size() > static_cast<size_t>(past_detections_length)) {
        past_detections.erase(past_detections.begin());
    }
}

const Eigen::MatrixXd& TrackedObject::get_estimate(bool absolute) const {
    if (absolute && coord_transformations != nullptr) {
        static Eigen::MatrixXd transformed = coord_transformations->rel_to_abs(estimate);
        return transformed;
    }
    return estimate;
}

void TrackedObject::set_estimate(const Eigen::MatrixXd& new_estimate) {
    estimate = new_estimate;
}

int TrackedObject::get_id() const {
    return id;
}

void TrackedObject::assign_id() {
    id = global_id_counter++;
    global_id = id;
}
