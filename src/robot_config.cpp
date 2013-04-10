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
 * \file   robot_config.cpp
 * \date   Dec 5, 2011
 * \author Scott K Logan, Matt Richard
 */
#include "control_panel/robot_config.h"
#include <QTextStream>
#include <iostream>

RobotConfig::RobotConfig()
{
	defaults();
}

void RobotConfig::defaults()
{
	configFilePath = "";
	robotName = "unknown_robot";
	system = "Unknown System";
	driveSystem = "UnknownDriveSystem";
	imageFilePath = ":/images/unknown.jpg";
	image.load(imageFilePath);
    nameSpace = "unknown_namespace";
	sensors.defaults();
    joint_states.defaults();
    processedData.defaults();
	diagnostics.defaults();
	commands.defaults();
	controls.defaults();
}

QString RobotConfig::getRobotName()
{
	if(robotName.isEmpty())
		return QString("(unnamed robot)");
	return robotName;
}

int RobotConfig::loadFrom(QFile *file, bool getComponents)
{
	defaults();

	QDomDocument doc(file->fileName());
	if(file->error() != QFile::NoError)
	{
		std::cerr << "ERROR: Could not setup DOM Doc" << std::endl;
		return -1; // File Error
	}

	doc.setContent(file);
	QDomElement root = doc.documentElement();
	if( root.tagName() != "robot" )
		return -2; // Syntax Error

	configFilePath = file->fileName();
	QDomNode n = root.firstChild();
	while(!n.isNull())
	{
		QDomElement e = n.toElement(); // try to convert the node to an element.
		processElement(e, getComponents);
		n = n.nextSibling();
	}

	return 0; // OK
}

QFile * RobotConfig::exportData(QFile *file)
{
	QDomDocument doc("");

	QDomElement root = doc.createElement("robot");
	doc.appendChild(root);

	QDomText txt;
	QDomElement e;

	/* Basic Elements */
	e = doc.createElement("robotName");
	txt = doc.createTextNode(robotName);
	e.appendChild(txt);
	root.appendChild(e);

	e = doc.createElement("system");
	txt = doc.createTextNode(system);
	e.appendChild(txt);
	root.appendChild(e);

	e = doc.createElement("driveSystem");
	txt = doc.createTextNode(driveSystem);
	e.appendChild(txt);
	root.appendChild(e);

	e = doc.createElement("imageFile");
	txt = doc.createTextNode(imageFilePath);
	e.appendChild(txt);
	root.appendChild(e);

	e = doc.createElement("nameSpace");
	txt = doc.createTextNode(nameSpace);
	e.appendChild(txt);
	root.appendChild(e);

	root.appendChild(getSensors(doc));
	if( joint_states.used )
		root.appendChild(getJoints(doc));
	root.appendChild(getProcessedData(doc));
	if( diagnostics.used )
		root.appendChild(getDiagnostics(doc));
	if( commands.used )
		root.appendChild(getCommands(doc));
	if( controls.used )
		root.appendChild(getControls(doc));

	QTextStream stream;
	stream.setDevice(file);
	doc.save(stream, 2);

	return file;
}

void RobotConfig::processElement(QDomElement e, bool getComponents)
{
    if(e.tagName() == "robotName")
        robotName = e.text();
    else if(e.tagName() == "system")
        system = e.text();
    else if(e.tagName() == "driveSystem")
        driveSystem = e.text();
    else if(e.tagName() == "imageFile")
    {
        if(!e.isNull() && !e.text().isEmpty())
        {
            imageFilePath = e.text();
            if(imageFilePath[0] != '/' && imageFilePath[0] != ':')
                imageFilePath.prepend(":/images/");
            image.load(imageFilePath);
        }
    }
    else if(e.tagName() == "nameSpace")
        nameSpace = e.text();
    else if(e.tagName() == "sensors")
    {
        if(!e.isNull() && !e.text().isEmpty() && getComponents)
            processSensors(e);
    }
    else if(e.tagName() == "joints")
    {
        if(!e.isNull() && !e.text().isEmpty() && getComponents)
            processJoints(e);
    }
    else if(e.tagName() == "processedData")
    {
        if(!e.isNull() && !e.text().isEmpty() && getComponents)
            processProcessedData(e);
    }
    else if(e.tagName() == "diagnostics")
    {
        if(!e.isNull() && !e.text().isEmpty() && getComponents)
            processDiagnostics(e);
    }
    else if(e.tagName() == "commands")
    {
        if(!e.isNull() && !e.text().isEmpty() && getComponents)
            processCommands(e);
    }
    else if(e.tagName() == "controls")
    {
        if(!e.isNull() && !e.text().isEmpty() && getComponents)
            processControls(e);
    }
    else
        std::cerr << "WARNING: Unknown tag " << e.tagName().toStdString() << std::endl;
}

////////////////////////// Sensors /////////////////////////////

void RobotConfig::processSensors(QDomElement e)
{
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		e = n.toElement();
		if(e.tagName() == "camera")
			addCamera(e);
		else if(e.tagName() == "laser")
			addLaser(e);
		else if(e.tagName() == "gps")
			addGPS(e);
		else if(e.tagName() == "compass")
			addCompass(e);
		else if(e.tagName() == "imu")
			addIMU(e);
		else if(e.tagName() == "range")
			addRange(e);
		else
			std::cerr << "WARNING: Unknown sensor tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
}

QDomElement RobotConfig::getSensors(QDomDocument &doc)
{
	QDomElement root = doc.createElement("sensors");

	for(unsigned int i = 0; i < sensors.cameras.size(); i++)
		root.appendChild(getCamera(doc, sensors.cameras[i]));
	for(unsigned int i = 0; i < sensors.lasers.size(); i++)
		root.appendChild(getLaser(doc, sensors.lasers[i]));
	for(unsigned int i = 0; i < sensors.gps.size(); i++)
		root.appendChild(getGPS(doc, sensors.gps[i]));
	for(unsigned int i = 0; i < sensors.compass.size(); i++)
		root.appendChild(getCompass(doc, sensors.compass[i]));
	for(unsigned int i = 0; i < sensors.imu.size(); i++)
		root.appendChild(getIMU(doc, sensors.imu[i]));
	for(unsigned int i = 0; i < sensors.range.size(); i++)
		root.appendChild(getRange(doc, sensors.range[i]));

	return root;
}

void RobotConfig::addCamera(QDomElement e)
{
	struct RobotCamera new_cam;
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		e = n.toElement();
		if(e.tagName() == "name")
			new_cam.name = e.text();
		else if(e.tagName() == "topicName")
			new_cam.topicName = e.text();
		else
			std::cerr << "WARNING: Unknown sensor tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
	sensors.cameras.insert(sensors.cameras.begin(), new_cam);
}

QDomElement RobotConfig::getCamera(QDomDocument &doc, struct RobotCamera &cam)
{
	QDomElement root = doc.createElement("camera");

	QDomElement e = doc.createElement("name");
	QDomText txt = doc.createTextNode(cam.name);
	e.appendChild(txt);
	root.appendChild(e);

	e = doc.createElement("topicName");
	txt = doc.createTextNode(cam.topicName);
	e.appendChild(txt);
	root.appendChild(e);

	return root;
}

void RobotConfig::addLaser(QDomElement e)
{
	struct RobotLaser new_laser;
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		e = n.toElement();
		if(e.tagName() == "name")
			new_laser.name = e.text();
		else if(e.tagName() == "topicName")
			new_laser.topicName = e.text();
		else
			std::cerr << "WARNING: Unknown sensor tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
	sensors.lasers.insert(sensors.lasers.begin(), new_laser);
}

QDomElement RobotConfig::getLaser(QDomDocument &doc, struct RobotLaser &laser)
{
	QDomElement root = doc.createElement("laser");

	QDomElement e = doc.createElement("name");
	QDomText txt = doc.createTextNode(laser.name);
	e.appendChild(txt);
	root.appendChild(e);

	e = doc.createElement("topicName");
	txt = doc.createTextNode(laser.topicName);
	e.appendChild(txt);
	root.appendChild(e);

	return root;
}

void RobotConfig::addGPS(QDomElement e)
{
	struct RobotGPS new_gps;
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		e = n.toElement();
		if(e.tagName() == "name")
			new_gps.name = e.text();
		else if(e.tagName() == "topicName")
			new_gps.topicName = e.text();
		else if(e.tagName() == "longitude")
			new_gps.longitude = true;
		else if(e.tagName() == "latitude")
			new_gps.latitude = true;
		else if(e.tagName() == "altitude")
			new_gps.altitude = true;
		else if(e.tagName() == "positionCovariance")
			new_gps.covariance = true;
		else
			std::cerr << "WARNING: Unknown sensor tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
	sensors.gps.insert(sensors.gps.begin(), new_gps);
}

QDomElement RobotConfig::getGPS(QDomDocument &doc, struct RobotGPS &gps)
{
	QDomElement root = doc.createElement("gps");

	QDomElement e = doc.createElement("name");
	QDomText txt = doc.createTextNode(gps.name);
	e.appendChild(txt);
	root.appendChild(e);

	e = doc.createElement("topicName");
	txt = doc.createTextNode(gps.topicName);
	e.appendChild(txt);
	root.appendChild(e);

	if(gps.longitude)
	{
		e = doc.createElement("longitude");
		root.appendChild(e);
	}
	if(gps.latitude)
	{
		e = doc.createElement("latitude");
		root.appendChild(e);
	}
	if(gps.altitude)
	{
		e = doc.createElement("altitude");
		root.appendChild(e);
	}
	if(gps.covariance)
	{
		e = doc.createElement("positionCovariance");
		root.appendChild(e);
	}

	return root;
}

void RobotConfig::addCompass(QDomElement e)
{
	struct RobotCompass new_compass;
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		e = n.toElement();
		if(e.tagName() == "name")
			new_compass.name = e.text();
		else if(e.tagName() == "topicName")
			new_compass.topicName = e.text();
		else
			std::cerr << "WARNING: Unknown sensor tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
	sensors.compass.insert(sensors.compass.begin(), new_compass);
}

QDomElement RobotConfig::getCompass(QDomDocument &doc, struct RobotCompass &compass)
{
	QDomElement root = doc.createElement("compass");

	QDomElement e = doc.createElement("name");
	QDomText txt = doc.createTextNode(compass.name);
	e.appendChild(txt);
	root.appendChild(e);

	e = doc.createElement("topicName");
	txt = doc.createTextNode(compass.topicName);
	e.appendChild(txt);
	root.appendChild(e);

	return root;
}

void RobotConfig::addIMU(QDomElement e)
{
	struct RobotIMU new_imu;
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		e = n.toElement();
		if(e.tagName() == "name")
			new_imu.name = e.text();
		else if(e.tagName() == "topicName")
			new_imu.topicName = e.text();
        else if(e.tagName() == "roll")
            new_imu.roll = true;
        else if(e.tagName() == "pitch")
            new_imu.pitch = true;
        else if(e.tagName() == "yaw")
            new_imu.yaw = true;
		else if(e.tagName() == "angularVelocity")
			new_imu.angularVelocity = true;
		else if(e.tagName() == "linearAcceleration")
			new_imu.linearAcceleration = true;
		else if(e.tagName() == "hideAttitude")
			new_imu.hideAttitude = true;
		else if(e.tagName() == "hideHeading")
			new_imu.hideHeading = true;
		else if(e.tagName() == "hideLabels")
			new_imu.hideLabels = true;
		else
			std::cerr << "WARNING: Unknown sensor tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
	sensors.imu.insert(sensors.imu.begin(), new_imu);
}

QDomElement RobotConfig::getIMU(QDomDocument &doc, struct RobotIMU &imu)
{
	QDomElement root = doc.createElement("imu");

	QDomElement e = doc.createElement("name");
	QDomText txt = doc.createTextNode(imu.name);
	e.appendChild(txt);
	root.appendChild(e);

	e = doc.createElement("topicName");
	txt = doc.createTextNode(imu.topicName);
	e.appendChild(txt);
	root.appendChild(e);

	if(imu.roll)
	{
		e = doc.createElement("roll");
		root.appendChild(e);
	}
	if(imu.pitch)
	{
		e = doc.createElement("pitch");
		root.appendChild(e);
	}
	if(imu.yaw)
	{
		e = doc.createElement("yaw");
		root.appendChild(e);
	}
	if(imu.angularVelocity)
	{
		e = doc.createElement("angularVelocity");
		root.appendChild(e);
	}
	if(imu.linearAcceleration)
	{
		e = doc.createElement("linearAcceleration");
		root.appendChild(e);
	}
	if(imu.hideAttitude)
	{
		e = doc.createElement("hideAttitude");
		root.appendChild(e);
	}
	if(imu.hideHeading)
	{
		e = doc.createElement("hideHeading");
		root.appendChild(e);
	}
	if(imu.hideLabels)
	{
		e = doc.createElement("hideLabels");
		root.appendChild(e);
	}

	return root;
}

void RobotConfig::addRange(QDomElement e)
{
	struct RobotRange new_range;
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		e = n.toElement();
		if(e.tagName() == "name")
			new_range.name = e.text();
		else if(e.tagName() == "topicName")
			new_range.topicName = e.text();
		else
			std::cerr << "WARNING: Unknown sensor tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
	sensors.range.insert(sensors.range.begin(), new_range);
}

QDomElement RobotConfig::getRange(QDomDocument &doc, struct RobotRange &range)
{
	QDomElement root = doc.createElement("range");

	QDomElement e = doc.createElement("name");
	QDomText txt = doc.createTextNode(range.name);
	e.appendChild(txt);
	root.appendChild(e);

	e = doc.createElement("topicName");
	txt = doc.createTextNode(range.topicName);
	e.appendChild(txt);
	root.appendChild(e);

	return root;
}

/////////////////////////// End Sensors /////////////////////////////



////////////////////////// Joints //////////////////////////
void RobotConfig::processJoints(QDomElement e)
{
    QDomNode n = e.firstChild();
    while(!n.isNull())
    {
        e = n.toElement();
        if(e.tagName() == "topicName")
            joint_states.topicName = e.text();
        else if(e.tagName() == "position")
            joint_states.position = true;
        else if(e.tagName() == "velocity")
            joint_states.velocity = true;
        else if(e.tagName() == "effort")
            joint_states.effort = true;
        else if(e.tagName() == "joint")
            addJoint(e);
        else
            std::cerr << "WARNING: Unknown joints tag " << e.tagName().toStdString() << std::endl;
        n = n.nextSibling();
    }
}

QDomElement RobotConfig::getJoints(QDomDocument &doc)
{
	QDomElement root = doc.createElement("joints");

	QDomElement e = doc.createElement("topicName");
	QDomText txt = doc.createTextNode(joint_states.topicName);
	e.appendChild(txt);
	root.appendChild(e);

	if(joint_states.position)
	{
		e = doc.createElement("position");
		root.appendChild(e);
	}
	if(joint_states.velocity)
	{
		e = doc.createElement("velocity");
		root.appendChild(e);
	}
	if(joint_states.effort)
	{
		e = doc.createElement("effort");
		root.appendChild(e);
	}

	for(unsigned int i = 0; i < joint_states.joints.size(); i++)
		root.appendChild(getJoint(doc, joint_states.joints[i]));

	return root;
}

void RobotConfig::addJoint(QDomElement e)
{
    struct RobotJoint new_joint;
    QDomNode n = e.firstChild();
    while(!n.isNull())
    {
        e = n.toElement();
        if(e.tagName() == "name")
            new_joint.name = e.text();
        else if(e.tagName() == "displayName")
            new_joint.displayName = e.text();
        else
            std::cerr << "WARNING: Unknown joint tag " << e.tagName().toStdString() << std::endl;
        n = n.nextSibling();
    }
    joint_states.joints.push_back(new_joint);
    //joint_states.joints.insert(joint_states.joints.begin(), new_joint);
    joint_states.used = true;
}

QDomElement RobotConfig::getJoint(QDomDocument &doc, struct RobotJoint &joint)
{
	QDomElement root = doc.createElement("joint");

	QDomElement e = doc.createElement("name");
	QDomText txt = doc.createTextNode(joint.name);
	e.appendChild(txt);
	root.appendChild(e);

	e = doc.createElement("displayName");
	txt = doc.createTextNode(joint.displayName);
	e.appendChild(txt);
	root.appendChild(e);

	return root;
}
////////////////////////// End Joints //////////////////////////////


/////////////////////////// Processed Data ///////////////////////////
void RobotConfig::processProcessedData(QDomElement e)
{
    QDomNode n = e.firstChild();
    while(!n.isNull())
    {
        e = n.toElement();
        if(e.tagName() == "image")
            addImage(e);
        else if(e.tagName() == "disparityImage")
            addDisparityImage(e);
        else if(e.tagName() == "map")
            addMap(e);
        else if(e.tagName() == "odometry")
            addOdometry(e);
        else if(e.tagName() == "pose")
            addPose(e);
        else
            std::cerr << "WARNING: Unknown processed data tag " << e.tagName().toStdString() << std::endl;
        n = n.nextSibling();
    }
}

QDomElement RobotConfig::getProcessedData(QDomDocument &doc)
{
	QDomElement root = doc.createElement("processedData");

	for(unsigned int i = 0; i < processedData.images.size(); i++)
		root.appendChild(getImage(doc, processedData.images[i]));
	for(unsigned int i = 0; i < processedData.disparity_images.size(); i++)
		root.appendChild(getDisparityImage(doc, processedData.disparity_images[i]));
	for(unsigned int i = 0; i < processedData.maps.size(); i++)
		root.appendChild(getMap(doc, processedData.maps[i]));
	for(unsigned int i = 0; i < processedData.odometry.size(); i++)
		root.appendChild(getOdometry(doc, processedData.odometry[i]));
	for(unsigned int i = 0; i < processedData.pose.size(); i++)
		root.appendChild(getPose(doc, processedData.pose[i]));

	return root;
}

void RobotConfig::addImage(QDomElement e)
{
    struct RobotCamera new_image;
    QDomNode n = e.firstChild();
    while(!n.isNull())
    {
        e = n.toElement();
        if(e.tagName() == "name")
            new_image.name = e.text();
        else if(e.tagName() == "topicName")
            new_image.topicName = e.text();
        else
            std::cerr << "WARNING: Unknown processed data tag " << e.tagName().toStdString() << std::endl;
        n = n.nextSibling();
    }
    processedData.images.insert(processedData.images.begin(), new_image);
}

QDomElement RobotConfig::getImage(QDomDocument &doc, struct RobotCamera &image)
{
	QDomElement root = doc.createElement("image");

	QDomElement e = doc.createElement("name");
	QDomText txt = doc.createTextNode(image.name);
	e.appendChild(txt);
	root.appendChild(e);

	e = doc.createElement("topicName");
	txt = doc.createTextNode(image.topicName);
	e.appendChild(txt);
	root.appendChild(e);

	return root;
}

void RobotConfig::addDisparityImage(QDomElement e)
{
    struct RobotDisparityImage new_disparity;
    QDomNode n = e.firstChild();
    while(!n.isNull())
    {
        e = n.toElement();
        if(e.tagName() == "name")
            new_disparity.name = e.text();
        else if(e.tagName() == "topicName")
            new_disparity.topicName = e.text();
        else
            std::cerr << "WARNING: Unknown processed data tag " << e.tagName().toStdString() << std::endl;
        n = n.nextSibling();
    }
    processedData.disparity_images.insert(processedData.disparity_images.begin(), new_disparity);
}

QDomElement RobotConfig::getDisparityImage(QDomDocument &doc, struct RobotDisparityImage &disparity)
{
	QDomElement root = doc.createElement("disparityImage");

	QDomElement e = doc.createElement("name");
	QDomText txt = doc.createTextNode(disparity.name);
	e.appendChild(txt);
	root.appendChild(e);

	e = doc.createElement("topicName");
	txt = doc.createTextNode(disparity.topicName);
	e.appendChild(txt);
	root.appendChild(e);

	return root;
}

void RobotConfig::addMap(QDomElement e)
{
    struct RobotMap new_map;
    QDomNode n = e.firstChild();
    while(!n.isNull())
    {
        e = n.toElement();
        if(e.tagName() == "name")
            new_map.name = e.text();
        else if(e.tagName() == "topicName")
            new_map.topicName = e.text();
        else
            std::cerr << "WARNING: Unknown processed data tag " << e.tagName().toStdString() << std::endl;
        n = n.nextSibling();
    }
    processedData.maps.insert(processedData.maps.begin(), new_map);
}

QDomElement RobotConfig::getMap(QDomDocument &doc, struct RobotMap &map)
{
	QDomElement root = doc.createElement("map");

	QDomElement e = doc.createElement("name");
	QDomText txt = doc.createTextNode(map.name);
	e.appendChild(txt);
	root.appendChild(e);

	e = doc.createElement("topicName");
	txt = doc.createTextNode(map.topicName);
	e.appendChild(txt);
	root.appendChild(e);

	return root;
}

void RobotConfig::addOdometry(QDomElement e)
{
    struct RobotOdometry new_odometry;
    QDomNode n = e.firstChild();
    while(!n.isNull())
    {
        e = n.toElement();
        if(e.tagName() == "name")
            new_odometry.name = e.text();
        else if(e.tagName() == "topicName")
            new_odometry.topicName = e.text();
        else if(e.tagName() == "position")
            new_odometry.position = true;
        else if(e.tagName() == "orientation")
            new_odometry.orientation = true;
        else if(e.tagName() == "linearVelocity")
            new_odometry.linearVelocity = true;
        else if(e.tagName() == "angularVelocity")
            new_odometry.angularVelocity = true;
        else if(e.tagName() == "hideAttitude")
            new_odometry.hideAttitude = true;
        else if(e.tagName() == "hideHeading")
            new_odometry.hideHeading = true;
        else if(e.tagName() == "hideLabels")
            new_odometry.hideLabels = true;
        else if(e.tagName() == "updateMap")
            new_odometry.updateMap = true;
        else
            std::cerr << "WARNING: Unknown processed data tag " << e.tagName().toStdString() << std::endl;
        n = n.nextSibling();
    }
    processedData.odometry.insert(processedData.odometry.begin(), new_odometry);
}

QDomElement RobotConfig::getOdometry(QDomDocument &doc, struct RobotOdometry &odometry)
{
	QDomElement root = doc.createElement("odometry");

	QDomElement e = doc.createElement("name");
	QDomText txt = doc.createTextNode(odometry.name);
	e.appendChild(txt);
	root.appendChild(e);

	e = doc.createElement("topicName");
	txt = doc.createTextNode(odometry.topicName);
	e.appendChild(txt);
	root.appendChild(e);

	if(odometry.position)
	{
		e = doc.createElement("position");
		root.appendChild(e);
	}
	if(odometry.orientation)
	{
		e = doc.createElement("orientation");
		root.appendChild(e);
	}
	if(odometry.linearVelocity)
	{
		e = doc.createElement("linearVelocity");
		root.appendChild(e);
	}
	if(odometry.angularVelocity)
	{
		e = doc.createElement("angularVelocity");
		root.appendChild(e);
	}
	if(odometry.hideAttitude)
	{
		e = doc.createElement("hideAttitude");
		root.appendChild(e);
	}
	if(odometry.hideHeading)
	{
		e = doc.createElement("hideHeading");
		root.appendChild(e);
	}
	if(odometry.hideLabels)
	{
		e = doc.createElement("hideLabels");
		root.appendChild(e);
	}
	if(odometry.updateMap)
	{
		e = doc.createElement("updateMap");
		root.appendChild(e);
	}

	return root;
}

void RobotConfig::addPose(QDomElement e)
{
    struct RobotPose new_pose;
    QDomNode n = e.firstChild();
    while(!n.isNull())
    {
        e = n.toElement();
        if(e.tagName() == "name")
            new_pose.name = e.text();
        else if(e.tagName() == "topicName")
            new_pose.topicName = e.text();
        else if(e.tagName() == "position")
            new_pose.position = true;
        else if(e.tagName() == "orientation")
            new_pose.orientation = true;
        else if(e.tagName() == "hideAttitude")
            new_pose.hideAttitude = true;
        else if(e.tagName() == "hideHeading")
            new_pose.hideHeading = true;
        else if(e.tagName() == "hideLabels")
            new_pose.hideLabels = true;
        else if(e.tagName() == "isStamped")
            new_pose.isStamped = true;
        else if(e.tagName() == "hasCovariance")
	{
            new_pose.isStamped = true;
            new_pose.hasCovariance = true;
	}
        else if(e.tagName() == "updateMap")
            new_pose.updateMap = true;
        else
            std::cerr << "WARNING: Unknown processed data tag " << e.tagName().toStdString() << std::endl;
        n = n.nextSibling();
    }
    processedData.pose.insert(processedData.pose.begin(), new_pose);
}

QDomElement RobotConfig::getPose(QDomDocument &doc, struct RobotPose &pose)
{
	QDomElement root = doc.createElement("pose");

	QDomElement e = doc.createElement("name");
	QDomText txt = doc.createTextNode(pose.name);
	e.appendChild(txt);
	root.appendChild(e);

	e = doc.createElement("topicName");
	txt = doc.createTextNode(pose.topicName);
	e.appendChild(txt);
	root.appendChild(e);

	if(pose.position)
	{
		e = doc.createElement("position");
		root.appendChild(e);
	}
	if(pose.orientation)
	{
		e = doc.createElement("orientation");
		root.appendChild(e);
	}
	if(pose.hideAttitude)
	{
		e = doc.createElement("hideAttitude");
		root.appendChild(e);
	}
	if(pose.hideHeading)
	{
		e = doc.createElement("hideHeading");
		root.appendChild(e);
	}
	if(pose.hideLabels)
	{
		e = doc.createElement("hideLabels");
		root.appendChild(e);
	}
	if(pose.isStamped || pose.hasCovariance)
	{
		e = doc.createElement("isStamped");
		root.appendChild(e);
	}
	if(pose.hasCovariance)
	{
		e = doc.createElement("hasCovariance");
		root.appendChild(e);
	}
	if(pose.updateMap)
	{
		e = doc.createElement("updateMap");
		root.appendChild(e);
	}

	return root;
}

/////////////////////// End Processed Data //////////////////////


//////////////////////// Diagnostics /////////////////////////
void RobotConfig::processDiagnostics(QDomElement e)
{
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		e = n.toElement();
		if(e.tagName() == "topicName")
			diagnostics.topicName = e.text();
		else if(e.tagName() == "temperature")
			addTemperature(e);
		else if(e.tagName() == "voltage")
			addVoltage(e);
		else if(e.tagName() == "batteryLevel")
			addBatteryLevel(e);
		else
			std::cerr << "WARNING: Unknown diagnostic tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
}

QDomElement RobotConfig::getDiagnostics(QDomDocument &doc)
{
	QDomElement root = doc.createElement("diagnostics");

	QDomElement e = doc.createElement("topicName");
	QDomText txt = doc.createTextNode(diagnostics.topicName);
	e.appendChild(txt);
	root.appendChild(e);

	for(unsigned int i = 0; i < diagnostics.temperature.size(); i++)
		root.appendChild(getTemperature(doc, diagnostics.temperature[i]));
	for(unsigned int i = 0; i < diagnostics.voltage.size(); i++)
		root.appendChild(getVoltage(doc, diagnostics.voltage[i]));
	for(unsigned int i = 0; i < diagnostics.batteryLevel.size(); i++)
		root.appendChild(getBatteryLevel(doc, diagnostics.batteryLevel[i]));

	return root;
}

void RobotConfig::addTemperature(QDomElement e)
{
	struct RobotTemperature new_temperature;
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		e = n.toElement();
		if(e.tagName() == "name")
			new_temperature.name = e.text();
		else if(e.tagName() == "minRange")
			new_temperature.minRange = e.text().toFloat();
		else if(e.tagName() == "maxRange")
			new_temperature.maxRange = e.text().toFloat();
		else if(e.tagName() == "units")
			new_temperature.units = e.text();
		else
			std::cerr << "WARNING: Unknown diagnostic tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
	diagnostics.temperature.insert(diagnostics.temperature.begin(), new_temperature);
	diagnostics.used = true;
}

QDomElement RobotConfig::getTemperature(QDomDocument &doc, struct RobotTemperature &temperature)
{
	QDomElement root = doc.createElement("temperature");

	QDomElement e = doc.createElement("name");
	QDomText txt = doc.createTextNode(temperature.name);
	e.appendChild(txt);
	root.appendChild(e);

	e = doc.createElement("minRange");
	txt = doc.createTextNode(QString::number(temperature.minRange));
	e.appendChild(txt);
	root.appendChild(e);

	e = doc.createElement("maxRange");
	txt = doc.createTextNode(QString::number(temperature.maxRange));
	e.appendChild(txt);
	root.appendChild(e);

	e = doc.createElement("units");
	txt = doc.createTextNode(temperature.units);
	e.appendChild(txt);
	root.appendChild(e);

	return root;
}

void RobotConfig::addVoltage(QDomElement e)
{
	struct RobotVoltage new_voltage;
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		e = n.toElement();
		if(e.tagName() == "name")
			new_voltage.name = e.text();
		else if(e.tagName() == "minRange")
			new_voltage.minRange = e.text().toFloat();
		else if(e.tagName() == "maxRange")
			new_voltage.maxRange = e.text().toFloat();
		else if(e.tagName() == "operatingVoltage")
			new_voltage.operatingVoltage = e.text().toFloat();
		else
			std::cerr << "WARNING: Unknown diagnostic tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
	diagnostics.voltage.insert(diagnostics.voltage.begin(), new_voltage);
	diagnostics.used = true;
}

QDomElement RobotConfig::getVoltage(QDomDocument &doc, struct RobotVoltage &voltage)
{
	QDomElement root = doc.createElement("voltage");

	QDomElement e = doc.createElement("name");
	QDomText txt = doc.createTextNode(voltage.name);
	e.appendChild(txt);
	root.appendChild(e);

	e = doc.createElement("minRange");
	txt = doc.createTextNode(QString::number(voltage.minRange));
	e.appendChild(txt);
	root.appendChild(e);

	e = doc.createElement("maxRange");
	txt = doc.createTextNode(QString::number(voltage.maxRange));
	e.appendChild(txt);
	root.appendChild(e);

	e = doc.createElement("operatingVoltage");
	txt = doc.createTextNode(QString::number(voltage.operatingVoltage));
	e.appendChild(txt);
	root.appendChild(e);

	return root;
}

void RobotConfig::addBatteryLevel(QDomElement e)
{
	struct RobotBatteryLevel new_batteryLevel;
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		e = n.toElement();
		if(e.tagName() == "name")
			new_batteryLevel.name = e.text();
		else if(e.tagName() == "max")
			new_batteryLevel.max = e.text().toFloat();
		else
			std::cerr << "WARNING: Unknown diagnostic tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
	diagnostics.batteryLevel.insert(diagnostics.batteryLevel.begin(), new_batteryLevel);
	diagnostics.used = true;
}

QDomElement RobotConfig::getBatteryLevel(QDomDocument &doc, struct RobotBatteryLevel &batteryLevel)
{
	QDomElement root = doc.createElement("batteryLevel");

	QDomElement e = doc.createElement("name");
	QDomText txt = doc.createTextNode(batteryLevel.name);
	e.appendChild(txt);
	root.appendChild(e);

	e = doc.createElement("max");
	txt = doc.createTextNode(QString::number(batteryLevel.max));
	e.appendChild(txt);
	root.appendChild(e);

	return root;
}

////////////////////////// End Diagnostics //////////////////////////

void RobotConfig::processCommands(QDomElement e)
{
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		e = n.toElement();
		if(e.tagName() == "custom")
			addCommandCustom(e);
		else if(e.tagName() == "takeoff")
			addCommandCustom(e);
		else if(e.tagName() == "land")
			addCommandCustom(e);
		else
			std::cerr << "WARNING: Unknown command tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
}

QDomElement RobotConfig::getCommands(QDomDocument &doc)
{
	QDomElement root = doc.createElement("commands");

	for(unsigned int i = 0; i < commands.custom.size(); i++)
		root.appendChild(getCommandCustom(doc, commands.custom[i]));

	return root;
}

void RobotConfig::addCommandCustom(QDomElement e)
{
	struct RobotCommandCustom new_custom;
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		QDomElement ee = n.toElement();
		if(ee.tagName() == "name" && e.tagName() == "custom")
			new_custom.name = ee.text();
		else if(ee.tagName() == "topicName")
			new_custom.topicName = ee.text();
		else
			std::cerr << "WARNING: Unknown command tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
	if(e.tagName() == "takeoff" || e.tagName() == "land")
		new_custom.name = e.text();
	commands.custom.insert(commands.custom.begin(), new_custom);
	commands.used = true;
}

QDomElement RobotConfig::getCommandCustom(QDomDocument &doc, struct RobotCommandCustom &custom)
{
	QDomElement root;
	if(custom.name == "takeoff" || custom.name == "land")
		root = doc.createElement(custom.name);
	else
		root = doc.createElement("custom");

	QDomElement e;
	QDomText txt;
	if( custom.name != "takeoff" && custom.name != "land" )
	{
		e = doc.createElement("name");
		txt = doc.createTextNode(custom.name);
		e.appendChild(txt);
		root.appendChild(e);
	}

	e = doc.createElement("topicName");
	txt = doc.createTextNode(custom.topicName);
	e.appendChild(txt);
	root.appendChild(e);

	return root;
}

void RobotConfig::processControls(QDomElement e)
{
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		e = n.toElement();
		if(e.tagName() == "drive")
			addDrive(e);
		else
			std::cerr << "WARNING: Unknown control tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
}

QDomElement RobotConfig::getControls(QDomDocument &doc)
{
	QDomElement root = doc.createElement("controls");

	for(unsigned int i = 0; i < controls.drive.size(); i++)
		root.appendChild(getDrive(doc, controls.drive[i]));

	return root;
}

void RobotConfig::addDrive(QDomElement e)
{
	struct RobotDrive new_drive;
	QDomNode n = e.firstChild();
	while(!n.isNull())
	{
		e = n.toElement();
		if(e.tagName() == "topicName")
			new_drive.topicName = e.text();
		else
			std::cerr << "WARNING: Unknown control tag " << e.tagName().toStdString() << std::endl;
		n = n.nextSibling();
	}
	controls.drive.insert(controls.drive.begin(), new_drive);
	controls.used = true;
}

QDomElement RobotConfig::getDrive(QDomDocument &doc, struct RobotDrive &drive)
{
	QDomElement root = doc.createElement("drive");

	QDomElement e = doc.createElement("topicName");
	QDomText txt = doc.createTextNode(drive.topicName);
	e.appendChild(txt);
	root.appendChild(e);

	return root;
}

