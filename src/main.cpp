// main.cpp

#include <iostream>
#include "tracker.h"
#include "detection.h"
#include "tracked_object.h"

int main() {
    // Create example detections
    Eigen::MatrixXd points(1, 2);
    points << 1.0, 2.0;
    Detection detection(points);

    // Create tracker
    Tracker tracker([](const Detection& det, const TrackedObject& obj) {
        // Example distance function
        return (det.get_points() - obj.get_estimate()).norm();
    }, 10.0);

    // Update tracker
    std::vector<std::shared_ptr<Detection>> detections;
    detections.push_back(std::make_shared<Detection>(detection));
    tracker.update(detections);

    std::cout << "Tracking complete!" << std::endl;
    return 0;
}
