/*
 * Copyright (C) 2022 Gennaro Raiola
 * Author: Gennaro Raiola
 * email:  gennaro.raiola@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#ifndef HARDWARE_INTERFACE_GROUND_TRUTH_INTERFACE_H
#define HARDWARE_INTERFACE_GROUND_TRUTH_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>

namespace hardware_interface
{

class GroundTruthHandle
{
public:
  struct Data
  {
    Data()
      : name(),
        frame_id(),
        orientation(0),
        orientation_covariance(0),
        angular_velocity(0),
        angular_velocity_covariance(0),
        angular_acceleration(0),
        angular_acceleration_covariance(0),
        linear_position(0),
        linear_position_covariance(0),
        linear_velocity(0),
        linear_velocity_covariance(0),
        linear_acceleration(0),
        linear_acceleration_covariance(0) {}

    std::string name;                       
    std::string frame_id;                   
    double* orientation;                   
    double* orientation_covariance;        
    double* angular_velocity;               
    double* angular_velocity_covariance;    
    double* angular_acceleration;              
    double* angular_acceleration_covariance;    
    double* linear_position;
    double* linear_position_covariance;
    double* linear_velocity;
    double* linear_velocity_covariance;
    double* linear_acceleration;
    double* linear_acceleration_covariance;
  };

  GroundTruthHandle(const Data& data = Data())
    : name_(data.name),
      frame_id_(data.frame_id),
      orientation_(data.orientation),
      orientation_covariance_(data.orientation_covariance),
      angular_velocity_(data.angular_velocity),
      angular_velocity_covariance_(data.angular_velocity_covariance),
      angular_acceleration_(data.angular_acceleration),
      angular_acceleration_covariance_(data.angular_acceleration_covariance),
      linear_position_(data.linear_position),
      linear_position_covariance_(data.linear_position_covariance),
      linear_velocity_(data.linear_velocity),
      linear_velocity_covariance_(data.linear_velocity_covariance),
      linear_acceleration_(data.linear_acceleration),
      linear_acceleration_covariance_(data.linear_acceleration_covariance)
  {}

  std::string getName()                           const {return name_;}
  std::string getFrameId()                        const {return frame_id_;}
  const double* getOrientation()                  const {return orientation_;}
  const double* getOrientationCovariance()        const {return orientation_covariance_;}
  const double* getAngularVelocity()              const {return angular_velocity_;}
  const double* getAngularVelocityCovariance()    const {return angular_velocity_covariance_;}
  const double* getAngularAcceleration()          const {return angular_acceleration_;}
  const double* getAngularAccelerationCovariance()const {return angular_acceleration_covariance_;}
  const double* getLinearPosition()               const {return linear_position_;}
  const double* getLinearPositionCovariance()     const {return linear_position_covariance_;}
  const double* getLinearVelocity()               const {return linear_velocity_;}
  const double* getLinearVelocityCovariance()     const {return linear_velocity_covariance_;}
  const double* getLinearAcceleration()           const {return linear_acceleration_;}
  const double* getLinearAccelerationCovariance() const {return linear_acceleration_covariance_;}

private:
  std::string name_;
  std::string frame_id_;

    double* orientation_;                   
    double* orientation_covariance_;        
    double* angular_velocity_;               
    double* angular_velocity_covariance_;    
    double* angular_acceleration_;              
    double* angular_acceleration_covariance_;    
    double* linear_position_;
    double* linear_position_covariance_;
    double* linear_velocity_;
    double* linear_velocity_covariance_;
    double* linear_acceleration_;
    double* linear_acceleration_covariance_;
};

/** \brief Hardware interface to support reading the ground truth state. */
class GroundTruthInterface : public HardwareResourceManager<GroundTruthHandle> {};

}

#endif // HARDWARE_INTERFACE_GROUND_TRUTH_INTERFACE_H
