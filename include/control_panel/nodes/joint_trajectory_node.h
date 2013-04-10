/* @todo Add license here */

/**
 * \file   joint_trajectory_node.h
 * \date   April 16, 2012
 * \author Matt Richard
 */

#include <QObject>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <string>

class JointTrajectoryNode : public QObject
{
	Q_OBJECT

	public:
		JointTrajectoryNode(ros::NodeHandle *nh_ptr);
		//JointTrajectoryNode(const QStringList &joint_names_list, ros::NodeHandle *nh_ptr);
		void addJoint(const std::string &joint_name);
		void advertise();
		std::string getTopic() const { return topic_name; }
		int jointIndex(const std::string &joint_name);
		void setAcceleration(const std::string &joint_name, double acc);
		void setPosition(const std::string &joint_name, double pos);
		void setTopic(const std::string &topic) { topic_name = topic; }
		void setVelocity(const std::string &joint_name, double vel);
		void unadvertise();

	public slots:
		void publish();

	private:
		std::string topic_name;
		ros::NodeHandle *nh;
		ros::Publisher joint_traject_pub;
		trajectory_msgs::JointTrajectory joint_traject_msg;
};
