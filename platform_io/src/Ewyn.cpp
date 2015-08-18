#include "string.h"

//#define USE_USBCON 1
#include <ros.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/time.h>
#include <QueueList.h>

//#include "LineSensor.h"
#include "Motor.h"
//#include "QTRSensors.h"
#include "QuadratureEncoder.h"
#include "RosLogger.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <Timer5.h>

ros::NodeHandle  	nh;
//LineSensor*			lineSensor;
Motor*				motor;
nav_msgs::Odometry	odom;
ros::Publisher odom_pub("odom", &odom);
QuadratureEncoder*	quadratureEncoder;
tf::TransformBroadcaster odom_broadcaster;
RosLogger*			rlog;

// Odometry position.
double x = 0.0;
double y = 0.0;
double th = 0.0;

// Odometry velocities.
double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

// For odometry.
ros::Time last_time; // = nh.now();
ros::Time current_time; // = nh.now();

int loopCounter = 1;

static inline geometry_msgs::Quaternion setRPY(const double& roll, const double& pitch, const double& yaw) {
	double halfYaw = double(yaw) * double(0.5);  
	double halfPitch = double(pitch) * double(0.5);  
	double halfRoll = double(roll) * double(0.5);  
	double cosYaw = cos(halfYaw);
	double sinYaw = sin(halfYaw);
	double cosPitch = cos(halfPitch);
	double sinPitch = sin(halfPitch);
	double cosRoll = cos(halfRoll);
	double sinRoll = sin(halfRoll);
	geometry_msgs::Quaternion q;
	q.x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw; // x
	q.y =cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw; // y
	q.z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw; // z
	q.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw; // w
	return q;
}

static const double QUATERNION_TOLERANCE = 0.1f;

static inline double length2(const geometry_msgs::Quaternion& bt) {
	return bt.x * bt.x + bt.y * bt.y + bt.z * bt.z + bt.w * bt.w;
}

static inline double length(const geometry_msgs::Quaternion& bt) {
	return sqrt(length2(bt));
}

static inline geometry_msgs::Quaternion normalize(geometry_msgs::Quaternion& bt) {
	geometry_msgs::Quaternion result;
	double l = length(bt);
	result.x = l != 0 ? bt.x / l : 0.0;
	result.y = l != 0 ? bt.y / l : 0.0;
	result.z = l != 0 ? bt.z / l : 0.0;
	result.w = l != 0 ? bt.w / l : 0.0;
	return result;
}

static inline void quaternionTFToMsg(const geometry_msgs::Quaternion& bt, geometry_msgs::Quaternion& msg)  {
	if (fabs(length2(bt) - 1 ) > QUATERNION_TOLERANCE)  {
		nh.loginfo("TF to MSG: Quaternion Not Properly Normalized");
		geometry_msgs::Quaternion bt_temp = bt; 
		bt_temp = normalize(bt_temp);
		msg.x = bt_temp.x; msg.y = bt_temp.y; msg.z = bt_temp.z;  msg.w = bt_temp.w;
	} else {
		msg.x = bt.x; msg.y = bt.y; msg.z = bt.z;  msg.w = bt.w;
	}
};

static inline geometry_msgs::Quaternion createQuaternionMsgFromYaw(double yaw) {
	geometry_msgs::Quaternion q;
	q = setRPY(0.0, 0.0, yaw);
	geometry_msgs::Quaternion q_msg;
	quaternionTFToMsg(q, q_msg);
	return q_msg;
}

void setup() {
	nh.initNode();
	last_time = nh.now();
	current_time = nh.now();

	//Serial.begin(38400);
	//#####delay(1000);

	rlog = new RosLogger(nh);
	//#####quadratureEncoder = new QuadratureEncoder();
//	lineSensor = new LineSensor();
//	lineSensor->calibrate();

	nh.advertise(odom_pub);
	motor = new Motor(nh);

	/*
	Motor::Command c;
	c.direction = Motor::STOP; motor->enqueue(c);
	c.direction = Motor::BACKWARD; motor->enqueue(c);
	c.direction = Motor::FORWARD; motor->enqueue(c);
	c.direction = Motor::STOP; motor->enqueue(c);
	c.direction = Motor::RIGHT_TURN; motor->enqueue(c);
	c.direction = Motor::STOP; motor->enqueue(c);
	c.direction = Motor::BACKWARD; motor->enqueue(c);
	c.direction = Motor::STOP; motor->enqueue(c);
	c.direction = Motor::LEFT_TURN; motor->enqueue(c);
	c.direction = Motor::STOP; motor->enqueue(c);
	*/

	nh.subscribe(Motor::sub);

	while (!nh.connected()) { nh.spinOnce(); }
	
	rlog->info("START UP...");
}

char buffer[128];

void loop() {
	nh.spinOnce();
	current_time = nh.now();

	vx = motor->leftVelocity();
	vy = motor->rightVelocity();
	vth = motor->zVelocity();
	double dt = (current_time.toNsec() - last_time.toNsec()) / (1000000000.0);
	double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
	double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
	double delta_th = vth * dt;

	x += delta_x;
	y += delta_y;
	th = delta_th;

	geometry_msgs::Quaternion odom_quat = createQuaternionMsgFromYaw(th);
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    //odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    static unsigned long odomSeq = 0;
    odom.header.seq = odomSeq++;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = odom_quat.x;
    odom.pose.pose.orientation.y = odom_quat.y;
    odom.pose.pose.orientation.z = odom_quat.z;
    odom.pose.pose.orientation.w = odom_quat.w;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = vth;
    odom.twist.twist.angular.y = vth;
    odom.twist.twist.angular.z = vth;

    for (int i = 0; i < 36; i++) {
    	odom.pose.covariance[i] = 0.0;
    	odom.twist.covariance[i] = 0.0;
    }

    //publish the message
    odom_pub.publish(&odom);

	// lineSensor->read();
	// const LineSensor::TSensorArray& values = lineSensor->sensorValues();

	// sprintf(buffer, "Position: %d, values: L> %d, %d, %d, %d, %d, %d, %d, %d <R",
	// 	lineSensor->position(),
	// 	values[0],
	// 	values[1],
	// 	values[2],
	// 	values[3],
	// 	values[4],
	// 	values[5],
	// 	values[6],
	// 	values[7]
	// 	);
	
	// rlog->info("Number: %d, quad: %d", loopCounter++, QuadratureEncoder::Counter());
	// rlog->info(buffer);
	//rlog->info("Queue len: %d", motor->queueLength());
	//quadratureEncoder->info();
	motor->run();
	last_time = current_time;
	//delay(10);
}