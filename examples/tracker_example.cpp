#include <iostream>
#include <opencv2/opencv.hpp>
#include "tracker.h"
#include "detection.h"
#include "FilterFactory.h"
#include "NoFilter.h"

constexpr int WIDTH = 800;
constexpr int HEIGHT = 600;
constexpr int NUM_TARGETS = 10;
constexpr int TRAIL_MAX_LENGTH = 20;

struct Target {
    cv::Point2f position;
    cv::Point2f velocity;
};

cv::Scalar generate_color(int id) {
    static cv::RNG rng(12345);
    rng.state = id;
    return cv::Scalar(rng.uniform(64, 255), rng.uniform(64, 255), rng.uniform(64, 255));
}

int main() {
    cv::namedWindow("Tracker", cv::WINDOW_AUTOSIZE);

    std::vector<Target> targets;
    for (int i = 0; i < NUM_TARGETS; ++i) {
        Target t;
        t.position = cv::Point2f(rand() % WIDTH, rand() % HEIGHT);
        t.velocity = cv::Point2f((rand() % 5 + 1) * ((rand() % 2) ? 1 : -1),
                                 (rand() % 5 + 1) * ((rand() % 2) ? 1 : -1));
        targets.push_back(t);
    }

    auto distance_func = [](const Detection& det, const TrackedObject& obj) {
        Eigen::MatrixXd det_pts = det.get_absolute_points();
        Eigen::MatrixXd est = obj.get_estimate();
        if (det_pts.size() != est.size()) {
            std::cerr << "[ERROR] Shape mismatch in distance_function: det "
                      << det_pts.rows() << "x" << det_pts.cols()
                      << " vs obj " << est.rows() << "x" << est.cols() << "\n";
            throw std::runtime_error("Shape mismatch");
        }
        return (det_pts - est).norm();
    };

    NoFilterFactory no_filter_factory;
    Tracker tracker(distance_func, 50.0, 5, 2, 2, 0.0, &no_filter_factory, 5);

    int frame_num = 0;
    while (true) {
        cv::Mat frame(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(20, 20, 20));

        std::vector<std::shared_ptr<Detection>> detections;
        for (auto& t : targets) {
            t.position += t.velocity;

            if (t.position.x <= 0 || t.position.x >= WIDTH)
                t.velocity.x *= -1;
            if (t.position.y <= 0 || t.position.y >= HEIGHT)
                t.velocity.y *= -1;

            Eigen::MatrixXd points(1, 2);
            points(0, 0) = t.position.x;
            points(0, 1) = t.position.y;
            detections.push_back(std::make_shared<Detection>(points));

            cv::circle(frame, t.position, 3, cv::Scalar(0, 255, 0), -1);
        }

        auto tracked_objects = tracker.update(detections);

        for (const auto& obj : tracked_objects) {
            int id = obj->get_id();
            auto color = generate_color(id);

            const auto& trail = obj->trail;
            for (size_t i = 1; i < trail.size(); ++i) {
                float alpha = static_cast<float>(i) / trail.size();
                cv::Scalar faded_color = color * alpha;
                cv::line(frame, trail[i - 1], trail[i], faded_color, 2, cv::LINE_AA);
            }

            const Eigen::MatrixXd& est = obj->get_estimate();
            if (est.rows() > 0 && est.cols() >= 2) {
                cv::Point pt(est(0, 0), est(0, 1));
                cv::circle(frame, pt, 5, color, 2);
                cv::putText(frame, std::to_string(id), pt + cv::Point(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
            }
        }

        cv::imshow("Tracker", frame);
        char key = static_cast<char>(cv::waitKey(30));
        if (key == 27) break;

        ++frame_num;
    }

    std::cout << "✅ Tracker visualization ended." << std::endl;
    return 0;
}
