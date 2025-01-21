#include <memory>
#include <iostream>
#include <utility>
#include "subscriber.hpp"

class AlphaBetaFilter:public rclcpp::Node {
public:
    AlphaBetaFilter(const std::shared_ptr<counterSubscriber> counterSensor, 
                 const std::shared_ptr<ImuSubscriber> imuSensor, float alpha, float beta, float dt);

    void update();
    
private:
    const std::shared_ptr<counterSubscriber> counterSensor; 
    const std::shared_ptr<ImuSubscriber> imuSensor;
    rclcpp::TimerBase::SharedPtr timer_;

    float alpha;
    float beta;
    float dt;
    std::vector<float> position = {0, 0, 0};
    std::vector<float> velocity = {0, 0, 0};
    std::vector<std::pair<std::vector<float>, std::vector<float>>> plottedData;

};