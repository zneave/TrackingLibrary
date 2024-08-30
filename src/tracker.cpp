// tracker.cpp

#include "include/tracker.h"
#include "include/FilterFactory.h"
#include <algorithm>
#include <iostream>

Tracker::Tracker(std::function<double(const Detection&, const TrackedObject&)> distance_function,
                 double distance_threshold, int hit_counter_max, int initialization_delay,
                 int pointwise_hit_counter_max, double detection_threshold,
                 FilterFactory* filter_factory, int past_detections_length, 
                 std::function<double(const TrackedObject&, const TrackedObject&)> reid_distance_function,
                 double reid_distance_threshold, int reid_hit_counter_max)
    : distance_function(distance_function), distance_threshold(distance_threshold),
      hit_counter_max(hit_counter_max), pointwise_hit_counter_max(pointwise_hit_counter_max),
      detection_threshold(detection_threshold), filter_factory(filter_factory),
      past_detections_length(past_detections_length), reid_distance_function(reid_distance_function),
      reid_distance_threshold(reid_distance_threshold), reid_hit_counter_max(reid_hit_counter_max) {

    if (initialization_delay == -1) {
        this->initialization_delay = hit_counter_max / 2;
    } else {
        this->initialization_delay = initialization_delay;
    }
}

std::vector<std::shared_ptr<TrackedObject>> Tracker::update(
    const std::vector<std::shared_ptr<Detection>>& detections,
    int period, CoordinatesTransformation* coord_transformations) {

    if (coord_transformations != nullptr) {
        for (auto& det : detections) {
            det->update_coordinate_transformation(*coord_transformations);
        }
    }

    remove_stale_trackers();
    update_tracker_with_detections(detections, period);
    create_new_tracked_objects(detections, period);

    return get_active_objects();
}

void Tracker::remove_stale_trackers() {
    tracked_objects.erase(
        std::remove_if(tracked_objects.begin(), tracked_objects.end(),
                       [](const std::shared_ptr<TrackedObject>& obj) { return !obj->is_active(); }),
        tracked_objects.end());
}

void Tracker::update_tracker_with_detections(const std::vector<std::shared_ptr<Detection>>& detections, int period) {
    // Add your logic to update tracked objects with detections here
}

void Tracker::create_new_tracked_objects(const std::vector<std::shared_ptr<Detection>>& unmatched_detections, int period) {
    // Add logic to create new tracked objects from unmatched detections
}

int Tracker::current_object_count() const {
    return std::count_if(tracked_objects.begin(), tracked_objects.end(),
                         [](const std::shared_ptr<TrackedObject>& obj) { return obj->is_active(); });
}

int Tracker::total_object_count() const {
    return tracked_objects.size();
}

std::vector<std::shared_ptr<TrackedObject>> Tracker::get_active_objects() const {
    std::vector<std::shared_ptr<TrackedObject>> active_objects;
    for (const auto& obj : tracked_objects) {
        if (obj->is_active()) {
            active_objects.push_back(obj);
        }
    }
    return active_objects;
}
