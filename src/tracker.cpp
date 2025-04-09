#include "tracker.h"
#include "FilterFactory.h"
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

    std::cout << "[DEBUG] Running update() with " << detections.size() << " detections\n";

    if (coord_transformations != nullptr) {
        for (auto& det : detections) {
            det->update_coordinate_transformation(*coord_transformations);
        }
    }

    remove_stale_trackers();

    std::cout << "[DEBUG] Removed " << (tracked_objects.size()) << " stale trackers\n";

    std::vector<bool> matched(detections.size(), false);

    for (auto& obj : tracked_objects) {
        double best_dist = distance_threshold;
        int best_idx = -1;

        for (size_t i = 0; i < detections.size(); ++i) {
            if (matched[i]) continue;

            try {
                double dist = distance_function(*detections[i], *obj);
                if (dist < best_dist) {
                    best_dist = dist;
                    best_idx = static_cast<int>(i);
                }
            } catch (const std::exception& e) {
                std::cerr << "[ERROR] Exception in distance_function: " << e.what() << "\n";
            }
        }

        if (best_idx != -1) {
            obj->hit(*detections[best_idx], period);
            matched[best_idx] = true;
            std::cout << "[DEBUG] Matched detection " << best_idx << " to tracked object\n";
        } else {
            obj->tracker_step();
            std::cout << "[DEBUG] Tracker stepped due to no match\n";
        }
    }

    std::vector<std::shared_ptr<Detection>> unmatched_detections;
    for (size_t i = 0; i < detections.size(); ++i) {
        if (!matched[i]) {
            unmatched_detections.push_back(detections[i]);
        }
    }

    create_new_tracked_objects(unmatched_detections, period, coord_transformations);

    std::cout << "[DEBUG] Total tracked objects after update: " << tracked_objects.size() << "\n";

    return get_active_objects();
}

void Tracker::remove_stale_trackers() {
    tracked_objects.erase(
        std::remove_if(tracked_objects.begin(), tracked_objects.end(),
                       [](const std::shared_ptr<TrackedObject>& obj) { return !obj->is_active(); }),
        tracked_objects.end());
}

void Tracker::create_new_tracked_objects(const std::vector<std::shared_ptr<Detection>>& unmatched_detections, int period, CoordinatesTransformation* coord_transformations) {
    for (const auto& det : unmatched_detections) {
        std::cout << "[DEBUG] Creating new tracked object\n";

        auto tracked_object = std::make_shared<TrackedObject>(
            nullptr, *det, hit_counter_max, initialization_delay, pointwise_hit_counter_max,
            detection_threshold, period, filter_factory, past_detections_length,
            reid_hit_counter_max, coord_transformations);

        tracked_object->assign_id();  // Assign unique ID
        tracked_objects.push_back(tracked_object);
    }
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
