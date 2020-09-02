#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>

using namespace std;

#define safety_distance 0.5
#define translation_error 0.1
#define kp 0.5
#define ki 0
#define kd 0

class translation {
private:

    ros::NodeHandle n;

    // communication with odometry
    ros::Subscriber sub_odometry;

    // communication with decision
    ros::Publisher pub_translation_done;
    ros::Subscriber sub_translation_to_do;

    // communication with cmd_vel to send command to the mobile robot
    ros::Publisher pub_cmd_vel;

    // communication with obstacle_detection
    ros::Subscriber sub_obstacle_detection;

    bool new_translation_to_do;//to check if a new /translation_to_do is available or not
    bool init_odom;//to check if new data from odometry are available
    bool display_odom;
    bool init_obstacle;//to check if the first "closest_obstacle" has been published or not
    bool display_obstacle;

    geometry_msgs::Point start_position;
    geometry_msgs::Point current_position;

    float translation_to_do;

    float error_integral;
    float error_previous;

    int cond_translation;

    geometry_msgs::Point closest_obstacle;

public:

translation() {

    // communication with cmd_vel
    pub_cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // communication with odometry
    sub_odometry = n.subscribe("odom", 1, &translation::odomCallback, this);
    cond_translation = false;

    // communication with decision
    pub_translation_done = n.advertise<std_msgs::Float32>("translation_done", 1);
    sub_translation_to_do = n.subscribe("translation_to_do", 1, &translation::translation_to_doCallback, this);//this is the translation that has to be performed

    // communication with obstacle_detection
    sub_obstacle_detection = n.subscribe("closest_obstacle", 1, &translation::closest_obstacleCallback, this);

    error_integral = 0;
    error_previous = 0;

    new_translation_to_do = false;
    init_odom = false;
    display_odom = false;
    init_obstacle = false;
    display_obstacle = false;

    //INFINTE LOOP TO COLLECT LASER DATA AND PROCESS THEM
    ros::Rate r(10);// this node will run at 10hz
    while (ros::ok()) {
        ros::spinOnce();//each callback is called once to collect new data: laser + robot_moving
        update();//processing of data
        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }

}

//UPDATE: main processing
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update() {

    ROS_INFO("new_odom: %i, cond_translation: %i, init_obstacle: %i", init_odom, cond_translation, init_obstacle);
    // we receive a new /translation_to_do
    if ( new_translation_to_do && init_odom && init_obstacle ) {
        new_translation_to_do = false;
        ROS_INFO("\n(translation_node) processing the /translation_to_do received from the decision node");
        ROS_INFO("(translation_node) translation_to_do: %f", translation_to_do);
        ROS_INFO("wait for obstacle_detection_node");

        start_position.x = current_position.x;
        start_position.y = current_position.y;

        cond_translation = true;
    }

    //we are performing a translation

    if ( init_odom && cond_translation && init_obstacle ) {
        float translation_done = distancePoints( start_position, current_position );
        float error = translation_to_do - translation_done;

        // the error is the minimum between the difference between /translation_to_do and /translation_done and the distance with the closest obstacle
        if ( fabs(error) > closest_obstacle.x )
            error = closest_obstacle.x;

        bool obstacle_detected = ( fabs(closest_obstacle.x) < safety_distance );

        if ( obstacle_detected )
            ROS_WARN("obstacle detected: (%f, %f)", closest_obstacle.x, closest_obstacle.y);

        cond_translation = ( fabs(error) > translation_error ) && !obstacle_detected;
        float translation_speed = 0;
        if ( cond_translation ) {
            //TO COMPLETE
            //Implementation of a PID controller for translation_to_do;
            //rotation_speed = kp*error + ki * error + kp * error_derivation;

            float error_derivation;//To complete
            error_derivation = error - error_previous;
            ROS_INFO("error_derivaion: %f", error_derivation);

            //error_integral = ...;//To complete
            error_integral += error;
            ROS_INFO("error_integral: %f", error_integral);

            //control of translation with a PID controller
            translation_speed = kp * error + ki * error_integral + kd * error_derivation;
            ROS_INFO("(translation_node) translation_done: %f, translation_to_do: %f -> translation_speed: %f", translation_done, translation_to_do, translation_speed);
        }
        else {
            ROS_INFO("(translation_node) translation_done: %f, translation_to_do: %f -> translation_speed: %f", translation_done, translation_to_do, translation_speed);
            float translation_done = distancePoints(start_position, current_position);
            ROS_INFO("(translation_node) final translation_done: %f", translation_done);
            ROS_INFO("(translation_node) waiting for a /translation_to_do");

            std_msgs::Float32 msg_translation_done;
            msg_translation_done.data = translation_done;

            pub_translation_done.publish(msg_translation_done);
            init_obstacle = false;
        }

        geometry_msgs::Twist twist;
        twist.linear.x = translation_speed;//we perform a translation on the x-axis
        twist.linear.y = 0;
        twist.linear.z = 0;

        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0;

        pub_cmd_vel.publish(twist);
    }
    if ( !display_odom && !init_odom ) {
        ROS_INFO("wait for odom");
        display_odom = true;
    }
    if ( display_odom && init_odom )  {
        ROS_INFO("odom is ok");
        display_odom = false;
    }
    if ( !display_obstacle && !init_obstacle ) {
        ROS_INFO("wait for obstacle_detection_node");
        display_obstacle = true;
    }
    if ( display_obstacle && init_obstacle ) {
        ROS_INFO("obstacle_detection_node is ok");
        display_obstacle = false;
    }

}// update

//CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void odomCallback(const nav_msgs::Odometry::ConstPtr& o) {

    init_odom = true;
    current_position.x = o->pose.pose.position.x;
    current_position.y = o->pose.pose.position.y;
    current_position.z = o->pose.pose.position.z;

}

void translation_to_doCallback(const std_msgs::Float32::ConstPtr & r) {
// process the translation to do received from the decision node

    new_translation_to_do = true;
    translation_to_do = r->data;

}

void closest_obstacleCallback(const geometry_msgs::Point::ConstPtr& obs) {

    init_obstacle = true;
    closest_obstacle.x = obs->x;
    closest_obstacle.y = obs->y;
    closest_obstacle.z = obs->z;

}//closest_obstacleCallback

// Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

}

};


int main(int argc, char **argv){

    ROS_INFO("(translation_node) waiting for a /translation_to_do");
    ros::init(argc, argv, "translation");

    translation bsObject;

    ros::spin();

    return 0;
}


