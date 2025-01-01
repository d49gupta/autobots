#include "alphaBetaFilter.hpp"

void AlphaBetaFilter::update() {
  int sensor1 = counterSensor->counterCache.newestValue();
  ImuSubscriber::IMUdata sensor2 = imuSensor->imuCache.newestValue();

  std::vector<float> predicted_velocity = {velocity[0] + sensor2.linear_acceleration.x * dt,
                                      velocity[1] + sensor2.linear_acceleration.y * dt,
                                      velocity[2] + sensor2.linear_acceleration.z * dt
                                      };

  std::vector<float> predicted_position = {position[0] + velocity[0] * dt + 0.5 * sensor2.linear_acceleration.x * dt * dt,
                                      position[1] + velocity[1] * dt + 0.5 * sensor2.linear_acceleration.y * dt* dt,
                                      position[2] + velocity[2] * dt + 0.5 * sensor2.linear_acceleration.z * dt * dt
                                      };
  position[0] = alpha*sensor2.linear_acceleration.x*dt*dt + (1.0 - alpha)*predicted_position[0];
  velocity[0] = beta*(position[0] - predicted_position[0]) / dt + (1.0 - beta)*predicted_velocity[0];
  
  position[1] = alpha*sensor2.linear_acceleration.y*dt*dt + (1.0 - alpha)*predicted_position[1];
  velocity[1] = beta*(position[1] - predicted_position[1]) / dt + (1.0 - beta)*predicted_velocity[1];

  position[2] = alpha*sensor2.linear_acceleration.z*dt*dt + (1.0 - alpha)*predicted_position[2];
  velocity[2] = beta*(position[2] - predicted_position[2]) / dt + (1.0 - beta)*predicted_velocity[2];

  plottedData.push_back(make_pair(position, velocity));
}

AlphaBetaFilter::AlphaBetaFilter(const std::shared_ptr<counterSubscriber> counterSensor, const std::shared_ptr<ImuSubscriber> imuSensor,
                                float alpha, float beta, float dt)  : Node("AlphaBetaFilter"), counterSensor(counterSensor), imuSensor(imuSensor) {
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&AlphaBetaFilter::update, this));
  this->alpha =  alpha;
  this->beta = beta;
  this->dt = dt;
  this->position = {0.0, 0.0, 0.0};
  this->velocity = {0.0, 0.0, 0.0};
}

