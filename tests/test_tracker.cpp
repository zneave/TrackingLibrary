#include <iostream>
#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "tracker.h"
#include "detection.h"
#include "tracked_object.h"

int main() {
    Eigen::MatrixXd points(1, 2);
    points << 1.0, 2.0;

    std::shared_ptr<Detection> detection = std::make_shared<Detection>(points);
    std::vector<std::shared_ptr<Detection>> detections = { detection };

    Tracker tracker([](const Detection& det, const TrackedObject& obj) {
        return (det.get_points() - obj.get_estimate()).norm();
    }, 10.0);

    tracker.update(detections);

    const auto& objects = tracker.get_objects();
    for (const auto& obj : objects) {
        std::cout << "Track ID: " << obj->get_id() << "\n";
        std::cout << "Estimated Position:\n" << obj->get_estimate() << "\n";
    }

    std::cout << "Tracking complete!" << std::endl;
    return 0;
}
