// tracked_object.cpp

#include "tracked_object.h"
#include "coordinates_transformation.h"
#include "filter.h"

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
      is_initializing(true) {
    
    absolute_points = initial_detection.get_absolute_points();
    // Initialize filter here using filter_factory
}

void TrackedObject::tracker_step() {
    // Logic for updating the tracker's state
    filter->predict();
    hit_counter -= 1;
    point_hit_counter = std::max(0, point_hit_counter - 1);
}

bool TrackedObject::is_active() const {
    return hit_counter >= 0;
}

void TrackedObject::hit(const Detection& detection, int period) {
    // Logic to update tracked object with a new detection
}

void TrackedObject::update_coordinate_transformation(const CoordinatesTransformation& coordinate_transformation) {
    if (coord_transformations != nullptr) {
        absolute_points = coordinate_transformation.rel_to_abs(absolute_points);
    }
}

void TrackedObject::_conditionally_add_to_past_detections(const Detection& detection) {
    // Logic to add to past detections
}

void TrackedObject::_acquire_ids() {
    // Logic to acquire IDs from factory
}

const Eigen::MatrixXd& TrackedObject::get_estimate(bool absolute) const {
    if (absolute && coord_transformations != nullptr) {
        return coord_transformations->rel_to_abs(estimate);
    }
    return estimate;
}
