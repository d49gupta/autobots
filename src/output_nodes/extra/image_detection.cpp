#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <iostream>
#include <fstream>

int main() {
    // Load YOLO model
    std::string modelConfiguration = "/home/dgupta/ros2_ws/src/Autonomous_car_project/models/yolov3.cfg";
    std::string modelWeights = "/home/dgupta/ros2_ws/src/Autonomous_car_project/models/yolov3.weights";
    std::string classesFile = "/home/dgupta/ros2_ws/src/Autonomous_car_project/models/coco.names";

    cv::dnn::Net net = cv::dnn::readNetFromDarknet(modelConfiguration, modelWeights);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    // Load the class names
    std::vector<std::string> classNames;
    std::ifstream classFile(classesFile);
    std::string line;
    while (std::getline(classFile, line)) {
        classNames.push_back(line);
    }

    // Open video capture (use 0 for the default camera or provide a video file path)
    cv::VideoCapture cap("/home/dgupta/ros2_ws/src/Autonomous_car_project/videos/174141-850771228_small.mp4");

    if (!cap.isOpened()) {
        std::cerr << "Error opening video stream!" << std::endl;
        return -1;
    }

    while (true) {
        cv::Mat frame;
        cap >> frame;  // Capture a new frame

        if (frame.empty()) {
            std::cerr << "No more frames or camera disconnected!" << std::endl;
            break;
        }

        // Prepare the frame for YOLO
        cv::Mat blob = cv::dnn::blobFromImage(frame, 1 / 255.0, cv::Size(416, 416), cv::Scalar(0, 0, 0), true, false);
        net.setInput(blob);

        // Run forward pass to get the output from the output layers
        std::vector<cv::Mat> outputs;
        net.forward(outputs, net.getUnconnectedOutLayersNames());

        // Post-processing: extract bounding boxes, class IDs, and confidences
        std::vector<int> classIds;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;

        for (const auto& output : outputs) {
            for (int i = 0; i < output.rows; ++i) {
                auto* data = output.ptr<float>(i);
                float confidence = data[4];
                if (confidence > 0.5) {  // Confidence threshold
                    int centerX = static_cast<int>(data[0] * frame.cols);
                    int centerY = static_cast<int>(data[1] * frame.rows);
                    int width = static_cast<int>(data[2] * frame.cols);
                    int height = static_cast<int>(data[3] * frame.rows);
                    int left = centerX - width / 2;
                    int top = centerY - height / 2;

                    classIds.push_back(static_cast<int>(data[5]));  // Class ID
                    confidences.push_back(confidence);
                    boxes.emplace_back(left, top, width, height);
                }
            }
        }

        // Apply non-maxima suppression to remove redundant overlapping boxes with lower confidences
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, 0.5, 0.4, indices);

        // Draw bounding boxes and labels on the frame
        for (size_t i = 0; i < indices.size(); ++i) {
            int idx = indices[i];
            cv::Rect box = boxes[idx];
            cv::rectangle(frame, box, cv::Scalar(0, 255, 0), 2);

            std::string label = classNames[classIds[idx]] + ": " + cv::format("%.2f", confidences[idx]);
            int baseLine;
            cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
            auto top = std::max(box.y, labelSize.height);
            cv::rectangle(frame, cv::Point(box.x, top - labelSize.height),
                          cv::Point(box.x + labelSize.width, top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
            cv::putText(frame, label, cv::Point(box.x, top), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        }

        // Display the result
        cv::imshow("YOLO Object Detection", frame);

        // Press 'q' to exit the loop
        if (cv::waitKey(30) == 'q') {
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();

    return 0;
}
