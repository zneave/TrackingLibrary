#ifndef TRACKER_H
#define TRACKER_H

#include <vector>
#include <memory>
#include <functional>
#include "detection.h"
#include "tracked_object.h"
#include "FilterFactory.h"
#include "coordinates_transformation.h"

class Tracker {
public:
    Tracker(std::function<double(const Detection&, const TrackedObject&)> distance_function,
            double distance_threshold, int hit_counter_max = 15,
            int initialization_delay = -1, int pointwise_hit_counter_max = 4,
            double detection_threshold = 0, FilterFactory* filter_factory = nullptr,
            int past_detections_length = 4, 
            std::function<double(const TrackedObject&, const TrackedObject&)> reid_distance_function = nullptr,
            double reid_distance_threshold = 0, int reid_hit_counter_max = -1);

    std::vector<std::shared_ptr<TrackedObject>> update(
        const std::vector<std::shared_ptr<Detection>>& detections,
        int period = 1, 
        CoordinatesTransformation* coord_transformations = nullptr
    );

    int current_object_count() const;
    int total_object_count() const;
    std::vector<std::shared_ptr<TrackedObject>> get_active_objects() const;

private:
    std::vector<std::shared_ptr<TrackedObject>> tracked_objects;
    std::function<double(const Detection&, const TrackedObject&)> distance_function;
    double distance_threshold;
    int hit_counter_max;
    int pointwise_hit_counter_max;
    double detection_threshold;
    FilterFactory* filter_factory;
    int past_detections_length;
    std::function<double(const TrackedObject&, const TrackedObject&)> reid_distance_function;
    double reid_distance_threshold;
    int reid_hit_counter_max;
    int initialization_delay;

    void remove_stale_trackers();
    void update_tracker_with_detections(const std::vector<std::shared_ptr<Detection>>& detections, int period);

    // 👇 Updated signature:
    void create_new_tracked_objects(const std::vector<std::shared_ptr<Detection>>& unmatched_detections, int period, CoordinatesTransformation* coord_transformations);
};

#endif // TRACKER_H
