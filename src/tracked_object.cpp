#include "tracked_object.h"
#include "coordinates_transformation.h"
#include "filter.h"
#include <iostream>

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
      id(-1),
      age(0),
      hit_counter(hit_counter_max) {

    absolute_points = initial_detection.get_absolute_points();
    estimate = absolute_points;

    if (filter_factory) {
        filter = filter_factory->create(estimate);
    }

    if (estimate.rows() > 0 && estimate.cols() >= 2) {
        trail.emplace_back(estimate(0, 0), estimate(0, 1));
        if (trail.size() > max_trail_length) {
            trail.erase(trail.begin());
        }
    }
}

void TrackedObject::tracker_step() {
    if (filter) {
        filter->predict();
        estimate = filter->get_state().transpose();
    }

    hit_counter -= 1;
    age += 1;

    if (estimate.rows() > 0 && estimate.cols() >= 2) {
        trail.emplace_back(cv::Point(static_cast<int>(estimate(0, 0)), static_cast<int>(estimate(0, 1))));
    }
}

bool TrackedObject::is_active() const {
    return hit_counter >= 0;
}

void TrackedObject::hit(const Detection& detection, int period) {
    estimate = detection.get_absolute_points();

    if (filter) {
        Eigen::VectorXd z = Eigen::Map<const Eigen::VectorXd>(estimate.data(), estimate.size());
        filter->update(z);
    }

    hit_counter = hit_counter_max;

    if (estimate.rows() > 0 && estimate.cols() >= 2) {
        trail.emplace_back(cv::Point(static_cast<int>(estimate(0, 0)), static_cast<int>(estimate(0, 1))));
    }

    _conditionally_add_to_past_detections(detection);
}

void TrackedObject::update_coordinate_transformation(const CoordinatesTransformation& coordinate_transformation) {
    if (coord_transformations != nullptr) {
        absolute_points = coordinate_transformation.rel_to_abs(absolute_points);
    }
}

void TrackedObject::_conditionally_add_to_past_detections(const Detection& detection) {
    if (past_detections.size() >= static_cast<size_t>(past_detections_length)) {
        past_detections.erase(past_detections.begin());
    }
    past_detections.push_back(detection);
}

void TrackedObject::_acquire_ids() {
    static int next_id = 0;
    id = next_id++;
}

void TrackedObject::assign_id(int new_id) {
    id = new_id;
}

int TrackedObject::get_id() const {
    return id;
}

const Eigen::MatrixXd& TrackedObject::get_estimate(bool absolute) const {
    if (absolute && coord_transformations != nullptr) {
        static Eigen::MatrixXd transformed = coord_transformations->rel_to_abs(estimate);
        return transformed;
    }
    return estimate;
}
