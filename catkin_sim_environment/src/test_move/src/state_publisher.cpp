#include <string>
#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    static tf::TransformBroadcaster broadcaster;
    tf::Transform transform;

    ros::Rate loop_rate(30);

    const double degree = M_PI/180;

    // robot state
    double tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;

    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    geometry_msgs::Point robotPosition;
    robotPosition.x = 0.0;
    robotPosition.y = 0.0;
    robotPosition.z = 0.5;
    transform.setOrigin(tf::Vector3(robotPosition.x,robotPosition.y,robotPosition.z));

    geometry_msgs::Quaternion attitude;
    attitude.x = 0.0;
    attitude.y = 0.0;
    attitude.z = 0.0;
    attitude.w = 1.0;
    transform.setRotation(tf::Quaternion(attitude.x,attitude.y,attitude.z,attitude.w));

    broadcaster.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"world","basic_quadrotor_1/base_link"));

    gazebo_msgs::ModelState modelState;
    modelState.model_name = (std::string)"basic_quadrotor_1";
    modelState.reference_frame = (std::string)"world";

    gazebo_msgs::SetModelState srv;

    while (ros::ok()) {
        double radius = 3.0;
       	double ascent = 0.05;
        robotPosition.x  = cos(angle)*radius - 4;
        //robotPosition.y  = 0.0;
        robotPosition.y = sin(angle)*radius;
	robotPosition.z = 1.0;	
	//robotPosition.z = tan(ascent)*radius*angle;
        attitude = tf::createQuaternionMsgFromYaw(angle+M_PI/2);
	//attitude = tf::createQuaternionMsgFromYaw(M_PI/2);

        geometry_msgs::Pose pose;
        pose.position = robotPosition;
        pose.orientation = attitude;
        modelState.pose = pose;
        transform.setOrigin(tf::Vector3(robotPosition.x,robotPosition.y,robotPosition.z));
        transform.setRotation(tf::Quaternion(attitude.x,attitude.y,attitude.z,attitude.w));
        broadcaster.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"world","basic_quadrotor_1/base_link"));

        srv.request.model_state = modelState;
        if(client.call(srv))
        {
            //this means it was a success
        }
        else{
            //this means it was not a success
        }
        // Create new robot state
        tilt += tinc;
        if (tilt<-.5 || tilt>0) tinc *= -1;
        height += hinc;
        if (height>.2 || height<0) hinc *= -1;
        swivel += degree;
        angle += degree/4;

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}

