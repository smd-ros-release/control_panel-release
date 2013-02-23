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
 * \file   robot_config.h
 * \date   Dec 5, 2011
 * \author Scott K Logan, Matt Richard
 * \brief  RobotConfig is the set of data that defines a robot's physical and sensory configuration.
 */
#ifndef CONTROL_PANEL_ROBOT_CONFIG_H
#define CONTROL_PANEL_ROBOT_CONFIG_H

#include <vector>
#include <QObject>
#include <QFile>
#include <QDomDocument>
#include <QImage>


///////////////////////////// Sensors //////////////////////////////

struct RobotCamera
{
	public:
		/* stored data */
		QString name;
		QString topicName;

		/* constructor */
		RobotCamera() : name("Unknown Camera"), topicName("unknown_camera")
		{
		}
};

struct RobotLaser
{
	public:
		/* stored data */
		QString name;
		QString topicName;

		/* constructor */
		RobotLaser() : name("Unknown Laser"), topicName("unknown_laser")
		{
		}
};

struct RobotGPS
{
	public:
		/* stored data */
		QString name;
		QString topicName;
		bool longitude;
		bool latitude;
		bool altitude;
		bool covariance;

		/* constructor */
		RobotGPS() : name("Unknown GPS"), topicName("unknown_gps"), longitude(false), latitude(false), altitude(false), covariance(false)
		{
		}
};

struct RobotCompass
{
	public:
		/* stored data */
		QString name;
		QString topicName;

		/* constructor */
		RobotCompass() : name("Unknown Compass"), topicName("unknown_compass")
		{
		}
};

struct RobotIMU
{
	public:
		/* stored data */
		QString name;
		QString topicName;
		bool roll;
		bool pitch;
		bool yaw;
		bool angularVelocity;
		bool linearAcceleration;
		bool hideAttitude;
		bool hideHeading;
		bool hideLabels;

		/* constructor */
		RobotIMU() : name("Unknown IMU"), topicName("unknown_imu/data"), roll(false), pitch(false), yaw(false), angularVelocity(false), linearAcceleration(false), hideAttitude(false), hideHeading(false), hideLabels(false)
		{
		}
};

struct RobotRange
{
	public:
		/* stored data */
		QString name;
		QString topicName;

		/* constructor */
		RobotRange() : name("Unknown Range"), topicName("unknown_range")
		{
		}
};

struct RobotSensors
{
	public:
		/* sensor list */
		std::vector<struct RobotCamera> cameras;
		std::vector<struct RobotLaser> lasers;
		std::vector<struct RobotGPS> gps;
		std::vector<struct RobotCompass> compass;
		std::vector<struct RobotIMU> imu;
		std::vector<struct RobotRange> range;

		/* set all to defaults */
		void defaults()
		{
			cameras.clear();
			lasers.clear();
			gps.clear();
			compass.clear();
			imu.clear();
			range.clear();
		}
};

////////////////////// End Sensors //////////////////////////



//////////////////////// Joints ///////////////////////////////

/**
 * \struct RobotJoint
 * \brief  @todo Fill this out
 */
struct RobotJoint
{
	public:
		/* stored data */
		QString name;
		QString displayName;

		/**
		 * \brief Contstructor. Initializes linked list and data
		 */
		RobotJoint()
			: name("unknown_joint"),
			  displayName("Unknown Joint") { }
};

/**
 * \struct RobotJoints
 * \brief  @todo Fill this out
 */
struct RobotJoints
{
	public:
		/* stored data */
		QString topicName;
		bool position;
		bool velocity;
		bool effort;
		bool used;

		/* Joints list */
		std::vector<struct RobotJoint> joints;

		/**
		 * \brief Constructor. Initializes public members
		 */
		RobotJoints()
			: topicName("joint_states"),
			  position(false),
			  velocity(false),
			  effort(false),
			  used(false) { }

		/**
		 * \brief Destroys linked list
		 */
		void defaults()
		{
			topicName = "joint_states";
			position = false;
			velocity = false;
			effort = false;
			joints.clear();
		}
};

///////////////////// End Joints ////////////////////////////



//////////////////// Processed Data /////////////////////////

/**
 * \struct RobotDisparityImage
 * \brief  @todo Fill this out
 */
struct RobotDisparityImage
{
	public:
		/* stored data */
		QString name;
		QString topicName;

		/**
		 * \brief Constructor. Initilizes data.
		 */
		RobotDisparityImage()
			: name("Unknown Disparity Image"),
			  topicName("unknown_disparity_image") { }
};

/**
 * \struct RobotMap
 * \brief  @todo Fill this out
 */
struct RobotMap
{
	public:
		/* stored data */
		QString name;
		QString topicName;

		/**
		 * \brief Constructor. Sets member values to default values.
		 */
		RobotMap()
			: name("Unknown Map"),
			  topicName("unknown_map") { }
};

/**
 * \struct RobotOdometry
 * \brief  @todo Fill this out
 */
struct RobotOdometry
{
	public:
		/* stored data */
		QString name;
		QString topicName;
		bool position;
		bool orientation;
		bool linearVelocity;
		bool angularVelocity;
		bool hideAttitude;
		bool hideHeading;
		bool hideLabels;
		bool updateMap;

		/**
		 * Constructor. Initializes structure members
		 */
		RobotOdometry()
			: name("Unknown Odometry"),
			  topicName("unknown_odometry"),
			  position(false),
			  orientation(false),
			  linearVelocity(false),
			  angularVelocity(false),
			  hideAttitude(false),
			  hideHeading(false),
			  hideLabels(false),
			  updateMap(false) { }
};

/**
 * \struct RobotPose
 * \brief  @todo Fill this out
 */
struct RobotPose
{
	public:
		/* stored data */
		QString name;
		QString topicName;
		bool position;
		bool orientation;
		bool hideAttitude;
		bool hideHeading;
		bool hideLabels;
		bool isStamped;
		bool hasCovariance;
		bool updateMap;

		/**
		 * Constructor. Initializes structure members
		 */
		RobotPose()
			: name("Unknown Pose"),
			  topicName("unknown_pose"),
			  position(false),
			  orientation(false),
			  hideAttitude(false),
			  hideHeading(false),
			  hideLabels(false),
			  isStamped(false),
			  hasCovariance(false),
			  updateMap(false) { }
};

/**
 * \struct RobotProcessedData
 * \brief  @todo Fill this out.
 */
struct RobotProcessedData
{
	public:
		/* Processed data lists */
		std::vector <struct RobotCamera> images;
		std::vector <struct RobotDisparityImage> disparity_images;
		std::vector <struct RobotMap> maps;
		std::vector <struct RobotOdometry> odometry;
		std::vector <struct RobotPose> pose;

		/**
		 * \brief Sets all struct members to their defaults
		 */
		void defaults()
		{
			images.clear();
			disparity_images.clear();
			maps.clear();
			odometry.clear();
			pose.clear();
		}
};

///////////////////////// End Processed Data //////////////////////////


//////////////////////////// Diagnostics ////////////////////////////

struct RobotTemperature
{
	public:
		/* stored data */
		QString name;
		float minRange;
		float maxRange;
		QString units;

		/* constructor */
		RobotTemperature() : name("Unknown Temperature"), minRange(0), maxRange(0), units("Celcius")
		{
		}
};

struct RobotVoltage
{
	public:
		/* stored data */
		QString name;
		float minRange;
		float maxRange;
		float operatingVoltage;

		/* constructor */
		RobotVoltage() : name("Unknown Temperature"), minRange(0), maxRange(0), operatingVoltage(0)
		{
		}
};

struct RobotBatteryLevel
{
	public:
		/* stored data */
		QString name;
		float max;

		/* constructor */
		RobotBatteryLevel() : name("Unknown Battery Level"), max(0)
		{
		}
};

struct RobotDiagnostics
{
	public:
		/* stored data */
		QString topicName;
		bool used;

		/* diagnostics list */
		std::vector<struct RobotTemperature> temperature;
		std::vector<struct RobotVoltage> voltage;
		std::vector<struct RobotBatteryLevel> batteryLevel;

		/* constructor */
		RobotDiagnostics() : topicName("diagnostics"), used(false)
		{
		}

		/* set all to defaults */
		void defaults()
		{
			topicName = "diagnostics";
			used = false;
			temperature.clear();
			voltage.clear();
			batteryLevel.clear();
		}
};

///////////////////////////// End Diagnostics /////////////////////////////


/////////////////////////// Commands //////////////////////////////////

struct RobotCommandCustom
{
	public:
		/* stored data */
		QString name;
		QString topicName;

		/* constructor */
		RobotCommandCustom() : name("Unknown Custom Command"), topicName("unknown_custom_command")
		{
		}
};

struct RobotCommands
{
	public:
		/* stored data */
		bool used;

		/* commands list */
		std::vector<struct RobotCommandCustom> custom;

		/* constructor */
		RobotCommands() : used(false)
		{
		}

		/* set all to defaults */
		void defaults()
		{
			used = false;
			custom.clear();
		}
};

////////////////////////// End Commands ///////////////////////////////


//////////////////////////// Controls ////////////////////////////////

struct RobotDrive
{
	public:
		/* stored data */
		QString topicName;

		/* constructor */
		RobotDrive() : topicName("unknown_drive")
		{
		}
};

struct RobotControls
{
	public:
		/* stored data */
		bool used;

		/* controls list */
		std::vector<struct RobotDrive> drive;

		/* constructor */
		RobotControls() : used(false)
		{
		}

		/* set all to defaults */
		void defaults()
		{
			used = false;
			drive.clear();
		}
};

////////////////////////// End Controls /////////////////////////////



struct RobotConfig : public QObject
{
	Q_OBJECT

	public:
		/* background data */
		QString configFilePath;

		/* general data */
		QString robotName;
		QString system;
		QString driveSystem;
		QString imageFilePath;
		QImage image;
		QString nameSpace;

		/* data */
		struct RobotSensors sensors;
		struct RobotJoints joint_states;
		struct RobotProcessedData processedData;
		struct RobotDiagnostics diagnostics;
		struct RobotCommands commands;
		struct RobotControls controls;

		/* constructor */
		RobotConfig();

		/* set all to defaults */
		void defaults();

		/* load from / export to QFile */
		int loadFrom(QFile *, bool = false);
		QFile * exportData(QFile*);

		/* proper name */
		QString getRobotName();

	private:
		/* process a core configuration element */
		void processElement(QDomElement, bool);

        /* process data */
        void processSensors(QDomElement);
	QDomElement getSensors(QDomDocument &);
        void processJoints(QDomElement);
	QDomElement getJoints(QDomDocument &);
        void processProcessedData(QDomElement);
	QDomElement getProcessedData(QDomDocument &);
        void processDiagnostics(QDomElement);
	QDomElement getDiagnostics(QDomDocument &);
        void processCommands(QDomElement);
	QDomElement getCommands(QDomDocument &);
        void processControls(QDomElement);
	QDomElement getControls(QDomDocument &);

        void addCamera(QDomElement);
	QDomElement getCamera(QDomDocument &, struct RobotCamera &);
        void addLaser(QDomElement);
	QDomElement getLaser(QDomDocument &, struct RobotLaser &);
        void addGPS(QDomElement);
	QDomElement getGPS(QDomDocument &, struct RobotGPS &);
        void addCompass(QDomElement);
	QDomElement getCompass(QDomDocument &, struct RobotCompass &);
        void addIMU(QDomElement);
	QDomElement getIMU(QDomDocument &, struct RobotIMU &);
        void addRange(QDomElement);
	QDomElement getRange(QDomDocument &, struct RobotRange &);
        void addJoint(QDomElement);
	QDomElement getJoint(QDomDocument &, struct RobotJoint &);
        void addImage(QDomElement);
	QDomElement getImage(QDomDocument &, struct RobotCamera &);
        void addDisparityImage(QDomElement);
	QDomElement getDisparityImage(QDomDocument &, struct RobotDisparityImage &);
        void addMap(QDomElement);
	QDomElement getMap(QDomDocument &, struct RobotMap &);
        void addOdometry(QDomElement);
	QDomElement getOdometry(QDomDocument &, struct RobotOdometry &);
        void addPose(QDomElement);
	QDomElement getPose(QDomDocument &, struct RobotPose &);
        void addTemperature(QDomElement);
	QDomElement getTemperature(QDomDocument &, struct RobotTemperature &);
        void addVoltage(QDomElement);
	QDomElement getVoltage(QDomDocument &, struct RobotVoltage &);
        void addBatteryLevel(QDomElement);
	QDomElement getBatteryLevel(QDomDocument &, struct RobotBatteryLevel &);
        void addCommandCustom(QDomElement);
	QDomElement getCommandCustom(QDomDocument &, struct RobotCommandCustom &);
        void addDrive(QDomElement);
	QDomElement getDrive(QDomDocument &, struct RobotDrive &);
};

#endif // CONTROL_PANEL_ROBOT_CONFIG_H
