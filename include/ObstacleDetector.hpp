/**
 * Copyright 2018, Niket Shah
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @file ObstacleDetector.hpp
 * @author Niket Shah
 * @copyright 2018 BSD
 * @brief Implementation of header of obstacle detector function.
 */

#ifndef INCLUDE_OBSTACLEDETECTOR_HPP_
#define INCLUDE_OBSTACLEDETECTOR_HPP_

#include<iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

/*
 * @brief Class ObstacleDetector
 *        To detect presence of obstacles nearby
 */

class ObstacleDetector {
 private:
  bool obstacleFlag;

 public:
  /*
   * @brief Constructor for class
   *        Initializes value of obstacle flag
   * @param none
   * @return void
   */
  ObstacleDetector();

  /* @brief destructor for class ObstacleDetector
   *        To destroy the object
   * @param none
   * @return void
   */
  ~ObstacleDetector();

  /* @brief function obstacleScanner
   *        To detect presence of obstacles nearby using laserscan
   * @param msg of type sensor_msgs::LaserScan
   * @return void
   */
  void obstacleScanner(const sensor_msgs::LaserScan::ConstPtr& msg);

  /* @brief function detector
   *        To fetch value of obstacleFlag
   * @param none
   * @return void
   */
  bool detector();
};

#endif  // INCLUDE_OBSTACLEDETECTOR_HPP_
