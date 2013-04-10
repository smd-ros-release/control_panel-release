/*
 * Copyright (c) 2011, 2012 Matt Richard, Scott K Logan.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


/**
 * \file   globals.h
 * \date   July 2011
 * \author Matt Richard
 * \brief  Global constants used by many classes.
 */
#ifndef CONTROL_PANEL_GLOBALS_H
#define CONTROL_PANEL_GLOBALS_H

#define PI 3.14159265358979323846

#include <string>


namespace Globals
{

const double TWO_PI       = (2.0 * PI);
const double PI_OVER_TWO  = (PI / 2.0);
const double PI_OVER_FOUR = (PI / 4.0);

const double RAD_TO_DEG = (180.0 / PI);
const double DEG_TO_RAG = (PI / 180.0);

const char DegreesSymbol = 176;


// Common topic names
const std::string DEFAULT_CAMERA_TOPIC     = "camera/image_raw";
const std::string DEFAULT_CONTROL_TOPIC    = "cmd_vel";
const std::string DEFAULT_DIAGNOSTIC_TOPIC = "diagnostics";
const std::string DEFAULT_GPS_TOPIC        = "gps";
const std::string DEFAULT_IMU_TOPIC        = "imu/data";
const std::string DEFAULT_JOINT_TOPIC      = "joint_states";
const std::string DEFAULT_JOYSTICK_TOPIC   = "joy";
const std::string DEFAULT_LASER_TOPIC      = "scan";
const std::string DEFAULT_MAP_TOPIC        = "map";
const std::string DEFAULT_ODOMETRY_TOPIC   = "odom";
const std::string DEFAULT_POSE_TOPIC       = "robot_pose_ekf/odom_combined";
const std::string DEFAULT_RANGE_TOPIC      = "range";


// Connection status with a robot
enum ConnectionStatus
{
	Disconnected,
	Connecting,
	Connected
};


enum RCMode
{
    Disabled,
    Keyboard,
    Joystick
};

// Status of a component received from the diagnostic node
enum DiagnosticStatus
{
    Ok,
    Warn,
    Error
};

} // end namespace

#endif // CONTROL_PANEL_GLOBALS_H
