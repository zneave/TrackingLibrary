#include <iostream>
#include <memory>
#include <vector>
#include "tracker.h"
#include "detection.h"
#include "tracked_object.h"

int main() {
    auto distance_fn = [](const Detection& det, const TrackedObject& obj) {
        auto d = det.get_points();
        auto o = obj.get_estimate();
        if (d.rows() != o.rows() || d.cols() != o.cols()) {
            std::cerr << "[ERROR] Shape mismatch in distance_function: det " << d.rows() << "x" << d.cols()
                      << " vs obj " << o.rows() << "x" << o.cols() << "\n";
            return 1e9;
        }
        return (d - o).norm();
    };

    Tracker tracker(distance_fn, 20.0); // distance threshold = 20

    std::vector<std::pair<double, double>> input_points = {
        {100, 200}, {102, 202}, {104, 204}, {106, 206}, {108, 208}
    };

    for (size_t frame_idx = 0; frame_idx < input_points.size(); ++frame_idx) {
        std::cout << "\n[FRAME " << frame_idx << "] Feeding detection: "
                  << input_points[frame_idx].first << ", " << input_points[frame_idx].second << "\n";

        Eigen::MatrixXd pts(1, 2);
        pts << input_points[frame_idx].first, input_points[frame_idx].second;
        auto det = std::make_shared<Detection>(pts);
        std::vector<std::shared_ptr<Detection>> dets = {det};

        auto tracked = tracker.update(dets);

        std::cout << "[INFO] Tracked objects count: " << tracked.size() << "\n";
        for (const auto& obj : tracked) {
            const auto& est = obj->get_estimate();
            std::cout << " - ID: " << obj->get_id() << ", Estimate ("
                      << est.rows() << "x" << est.cols() << "):\n" << est << "\n\n";
        }
    }

    std::cout << "\n✅ Tracker test completed.\n";
    return 0;
}
