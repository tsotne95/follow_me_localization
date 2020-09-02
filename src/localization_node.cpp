//localization using lidar data
// written by O. Aycard

#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/ColorRGBA.h"
#include <cmath>
#include <fstream>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include "std_msgs/Float32.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cstdlib>
#include <ctime>

using namespace std;

#define angle_resolution 5 //in degrees
#define distance_to_travel 1
#define angle_to_travel 20

#define hypo_size 15//hypothesis

class localization
{

private:
    ros::NodeHandle n;

    ros::Subscriber sub_scan;
    ros::Subscriber sub_odometry;
    ros::Subscriber sub_pose;
    ros::Publisher pub_localization_marker;

    // to store, process and display laserdata
    int nb_beams;
    float range_min, range_max;
    float angle_min, angle_max, angle_inc;
    float range[1000];
    geometry_msgs::Point current_scan[1000];
    bool valid[1000];
    bool init_laser;

    //to store the map
    nav_msgs::GetMap::Response resp;
    geometry_msgs::Point min, max;
    float cell_size;
    int width_max;
    int height_max;

    // GRAPHICAL DISPLAY
    int nb_pts;
    geometry_msgs::Point display[1000];
    std_msgs::ColorRGBA colors[1000];

    bool init_odom;
    geometry_msgs::Point odom_current;
    float odom_current_orientation;
    geometry_msgs::Point odom_last;
    float odom_last_orientation;

    //to store the localization
    geometry_msgs::Point map[hypo_size];
    float map_orientation[hypo_size];
    bool localization_initialized;

    float distance_traveled;
    float previous_distance_traveled;
    float angle_traveled;
    float previous_angle_traveled;

public:
    localization()
    {

        sub_scan = n.subscribe("scan", 1, &localization::scanCallback, this);
        sub_odometry = n.subscribe("odom", 1, &localization::odomCallback, this);
        sub_pose = n.subscribe("initialpose", 1, &localization::poseCallback, this);
        pub_localization_marker = n.advertise<visualization_msgs::Marker>("localization_marker", 1); // Preparing a topic to publish our results. This will be used by the visualization tool rviz

        // get map via RPC
        nav_msgs::GetMap::Request req;
        ROS_INFO("Requesting the map...");
        while (!ros::service::call("static_map", req, resp))
        {
            ROS_WARN("Request for map failed; trying again...");
            ros::Duration d(0.5);
            d.sleep();
        }

        init_odom = false;
        init_laser = false;
        localization_initialized = false;

        width_max = resp.map.info.width;
        height_max = resp.map.info.height;
        cell_size = resp.map.info.resolution;
        min.x = resp.map.info.origin.position.x;
        min.y = resp.map.info.origin.position.y;
        max.x = min.x + width_max * cell_size;
        max.y = min.y + height_max * cell_size;

        ROS_INFO("map loaded");
        ROS_INFO("Map: (%f, %f) -> (%f, %f) with size: %f", min.x, min.y, max.x, max.y, cell_size);

        //INFINTE LOOP TO COLLECT LASER DATA AND PROCESS THEM
        ros::Rate r(10); // this node will work at 10hz
        while (ros::ok())
        {
            ros::spinOnce(); //each callback is called once
            update();
            r.sleep(); //we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
        }
    }

    //UPDATE: main processing of laser data
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void update()
    {

        if (init_odom && init_laser)
        {
            if (!localization_initialized)
            {
                initialize_localization();
                localization_initialized = true;
            }
            previous_distance_traveled = distance_traveled;
            previous_angle_traveled = angle_traveled;

            distance_traveled = distancePoints(odom_current, odom_last);
            angle_traveled = fabs(odom_current_orientation - odom_last_orientation);
            if (angle_traveled < -M_PI)
                angle_traveled += 2 * M_PI;
            if (angle_traveled > M_PI)
                angle_traveled -= 2 * M_PI;

            if ((distance_traveled != previous_distance_traveled) || (angle_traveled != previous_angle_traveled))
                ROS_INFO("distance_traveled = %f, angle_traveled = %f since last localization", distance_traveled, angle_traveled * 180 / M_PI);

            if ((distance_traveled > distance_to_travel) || (fabs(angle_traveled * 180 / M_PI) > angle_to_travel))
                relocalize();
        }

    } // update

    void initialize_localization()
    {

        ROS_INFO("initialize localization");

        ROS_INFO("possible positions: (%f, %f) -> (%f, %f)", min.x, min.y, max.x, max.y);

        odom_last = odom_current;
        odom_last_orientation = odom_current_orientation;

        int score_max = 0;
        /*loop over all the possible positions (x, y, theta) {
        if ( cell_value(loop_x, loop_y) == 0 ) { // robair can only be at a free cell
            int score_current = compute_score(loop_x, loop_y, o);
            ROS_INFO("(%f, %f, %f): score = %i", loop_x, loop_y, o, score_current*180/M_PI);
            //we store the maximum score over all the possible positions
            populateMarkerTopic();
        }
    }*/

        // float x,y,o;

        // for(x=min.x;x<=max.x;x+=0.05)
        //     for(y=min.y;y<=max.y;y+=0.05)
        //         {
        //             ROS_INFO("(x=%f, y=%f)", x, y);
        //         if(cell_value(x,y)==0)
        //         {
        //             for(o=0;o<=360;o+=5)
        //                 {

        //                     float theta=o*M_PI/180;
        //                     int curr_score=compute_score(x,y,theta);

        //                     ROS_INFO("(%f, %f, %f): score = %i", x, y, o, curr_score);

        //                     if(curr_score>score_max)
        //                     {
        //                         //ROS_INFO("(%f, %f, %f): current max score now will be = %i", x, y, o, curr_score);
        //                         score_max=curr_score;
        //                         map.x=x;
        //                         map.y=y;

        //                         map_orientation=theta;

        //                         populateMarkerTopic();
        //                     }
        //                 }
        //         }
        //         }

        // ROS_INFO("(%f, %f, %f): position with max score is = %i", odom_current.x, odom_current.y, odom_current_orientation*180/M_PI, score_max);

        ROS_INFO("intial position 0 is: %f, %f %f", map[0].x, map[0].y, map_orientation[0] * 180 / M_PI);

        int i;
        srand(time(0));

        for (i = 1; i < hypo_size; i++)
        {

            float scale = rand() / (float)RAND_MAX;     /* [0, 1.0] */
            map[i].x = min.x + scale * (max.x - min.x); // [min.x,max.x] range
            map[i].y = min.y + scale * (max.y - min.y);

            float min_orientation = 0;
            float max_orientation = 2 * M_PI;

            map_orientation[i] = min_orientation + scale * (max_orientation - min_orientation);
            ROS_INFO("intial position %d is: %f, %f %f", i, map[i].x, map[i].y, map_orientation[i] * 180 / M_PI);
        }

        ROS_INFO("initialize localization done");

    } //initialize_localization

    void relocalize()
    {
        ROS_INFO("relocalize");

        //ROS_INFO("before predicted position:(%f, %f, %f)", map.x, map.y, map_orientation * 180 / M_PI);

        //we predict the position of the robot after a motion of 1m or 20 degres

        int i = 0;

        for (int i = 0; i < hypo_size; i++)
        {

            map[i].x += distance_traveled * cos(map_orientation[i]);
            map[i].y += distance_traveled * sin(map_orientation[i]);

            //to be sure that our predicted position is in the map [width,height] range
            map[i].x = std::min(max.x, map[i].x);
            map[i].x = std::max(min.x, map[i].x);

            map[i].y = std::min(max.y, map[i].y);
            map[i].y = std::max(min.y, map[i].y);

            map_orientation[i] += (odom_current_orientation - odom_last_orientation);

            if (map_orientation[i] < -M_PI)
                map_orientation[i] += 2 * M_PI;
            if (map_orientation[i] > M_PI)
                map_orientation[i] -= 2 * M_PI;

            ROS_INFO("predicted position for i = %d :(%f, %f, %f)", i, map[i].x, map[i].y, map_orientation[i] * 180 / M_PI);

            // loop over all the cells
            int score_max = compute_score(map[i].x, map[i].y, map_orientation[i]);
            ROS_INFO("the score of predicted position of i=%d is %d",i, score_max);
            populateMarkerTopic();

            odom_last = odom_current;
            odom_last_orientation = odom_current_orientation;

            float min_x = map[i].x - 0.5;
            float max_x = map[i].x + 0.5;

            float min_y = map[i].y - 0.5;
            float max_y = map[i].y + 0.5;

            float min_orientation = map_orientation[i] * 180 / M_PI - 20;
            float max_orientation = map_orientation[i] * 180 / M_PI + 20;

            float x, y, x1, y1;
            x = min.x;
            y = min.y;
            x1 = max.x;
            y1 = max.y;

            min_x = std::max(min_x, x);
            min_x = std::min(min_x, x1);

            min_y = std::max(min_y, y);
            min_y = std::min(min_y, y1);

            max_x = std::min(max_x, x1);
            max_x = std::max(max_x, x);
            max_y = std::min(max_y, y1);
            max_y = std::min(max_y, y);

            ROS_INFO("possible positions: (%f, %f) -> (%f, %f) -> (%f, %f)", min_x, min_y, max_x, max_y, min_orientation, max_orientation);

            /*loop over all the possible positions (x, y, theta) in the square of 1m around the predicted position. The possible orientations are between -PI/6 and +PI/6 radians around the predicted orientation {
        if ( cell_value(loop_x, loop_y) == 0 ) { // robair can only be at a free cell
            int score_current = compute_score(loop_x, loop_y, o);
            ROS_INFO("(%f, %f, %f): score = %i", loop_x, loop_y, o, score_current*180/M_PI);
            //we store the maximum score over all the possible positions
            populateMarkerTopic();
        }
    }*/

            score_max = 0;
            for (; min_x <= max_x; min_x += 0.05)
                for (; min_y <= max.y; min_y += 0.05)
                    if (!cell_value(min_x, min_y))
                    {
                        for (; min_orientation <= max_orientation; min_orientation += 5) //
                        {
                            float theta = min_orientation * M_PI / 180;
                            int curr_score = compute_score(min_x, min_y, theta);

                            //ROS_INFO("(%f, %f, %f): score = %i", min_x, min_y, theta, curr_score);

                            if (curr_score > score_max)
                            {
                                //ROS_INFO("(%f, %f, %f): maxscore = %i", min_x, min_y, theta, curr_score);
                                score_max = curr_score;
                                map[i].x = min_x;
                                map[i].y = min_y;

                                map_orientation[i] = theta;

                                populateMarkerTopic();
                            }
                        }
                    }
        }
        //ROS_INFO("(%f, %f, %f): maxscore = %i", map.x, map.y, map_orientation * 180 / M_PI, score_max);

        ROS_INFO("local localization done");
    }

    int compute_score(float x, float y, float o)
    {
        //compute the score of the position (x, y, o)
        //for each hit of the laser, we compute its position in the map and check if it is occupied or not
        float uncertainty = 0.05;

        nb_pts = 0;
        int score_current = 0;
        //loop over the hits of the laser
        float beam_angle = angle_min;
        for (int loop = 0; loop < nb_beams; loop++, beam_angle += angle_inc)
        {
            //    if ( valid[loop] ) {
            if (valid[loop])
            {
                geometry_msgs::Point hit;
                //hit.x = ...;
                //hit.y = ..;

                hit.x = x + range[loop] * cos(o + beam_angle);
                hit.y = y + range[loop] * sin(o + beam_angle);

                // we add the current hit to the hits to display
                display[nb_pts].x = hit.x;
                display[nb_pts].y = hit.y;
                display[nb_pts].z = 0;

                bool cell_occupied = false;
                //loop over the positions surronding the current hit of the laser and test if one of this cell is occupied
                for (float loop_x = hit.x - uncertainty; loop_x <= hit.x + uncertainty; loop_x += uncertainty)
                    for (float loop_y = hit.y - uncertainty; loop_y <= hit.y + uncertainty; loop_y += uncertainty)
                        //test if the current hit of the laser corresponds to an occupied cell
                        //cell_occupied = cell_occupied || ( cell_value(loop_x, loop_y) == 100 );
                        if (cell_value(loop_x, loop_y) == 100)
                            cell_occupied = true;

                if (cell_occupied)
                {
                    score_current++;

                    // when matching is ok: the hit of the laser is green
                    colors[nb_pts].r = 0;
                    colors[nb_pts].g = 1;
                    colors[nb_pts].b = 0;
                    colors[nb_pts].a = 1.0;
                }
                //the current hit of the laser corresponds to a free cell
                else
                {
                    // when matching is not ok: the hit of the laser is red
                    colors[nb_pts].r = 1;
                    colors[nb_pts].g = 0;
                    colors[nb_pts].b = 0;
                    colors[nb_pts].a = 1.0;
                }
                nb_pts++;
            }
        }

        return (score_current);
    }

    int cell_value(float x, float y)
    {
        //returns the value of the cell corresponding to the position (x, y) in the map
        //returns 100 if cell(x, y) is occupied, 0 if cell(x, y) is free

        if ((min.x <= x) && (x <= max.x) && (min.y <= y) && (y <= max.y))
        {
            float x_cell = (x - min.x) / cell_size;
            float y_cell = (y - min.y) / cell_size;
            int x_int = x_cell;
            int y_int = y_cell;
            //ROS_INFO("cell[%f = %d][%f = %d] = %d", x_cell, x_int, y_cell, y_int, map[x_int][y_int]);
            return (resp.map.data[width_max * y_int + x_int]);
        }
        else
            return (-1);
    }

    //CALLBACK
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
    {

        init_laser = true;
        // store the important data related to laserscanner
        range_min = scan->range_min;
        range_max = scan->range_max;
        angle_min = scan->angle_min;
        angle_max = scan->angle_max;
        angle_inc = scan->angle_increment;
        nb_beams = ((-1 * angle_min) + angle_max) / angle_inc;

        /*ROS_INFO("New data of laser received");
    ROS_INFO("range_min, range_max: %f, %f", range_min, range_max);
    ROS_INFO("angle_min: %f", angle_min*180/M_PI);
    ROS_INFO("angle_max: %f", angle_max*180/M_PI);
    ROS_INFO("angle_increment: %f", angle_inc*180/M_PI);
    ROS_INFO("number_of_beams: %d", nb_beams);*/
        //std::ofstream myfile("example.txt", ios::out | ios::binary|ios::app);

        // store the range and the coordinates in cartesian framework of each hit
        float beam_angle = angle_min;
        for (int loop = 0; loop < nb_beams; loop++, beam_angle += angle_inc)
        {
            if ((scan->ranges[loop] < range_max) && (scan->ranges[loop] > range_min))
            {
                range[loop] = scan->ranges[loop];
                valid[loop] = true;
            }
            else
            {
                range[loop] = range_max;
                valid[loop] = false;
            }

            //transform the scan in cartesian framework
            current_scan[loop].x = range[loop] * cos(beam_angle);
            current_scan[loop].y = range[loop] * sin(beam_angle);
            current_scan[loop].z = 0.0;

            //myfile << current_scan[loop].x<<" "<<current_scan[loop].y<<"\n";
            //ROS_INFO("scan coordinates: x= %f y=%f",current_scan[loop].x,current_scan[loop].y);
        }
        //myfile.close();
    } //scanCallback

    void odomCallback(const nav_msgs::Odometry::ConstPtr &o)
    {
        //ROS_INFO("odomCallback");
        init_odom = true;
        odom_current.x = o->pose.pose.position.x;
        odom_current.y = o->pose.pose.position.y;
        odom_current_orientation = tf::getYaw(o->pose.pose.orientation);

    } //odomCallback

    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped &pose)
    {
        //ROS_INFO("poseCallback");
        ROS_INFO("poseCallback %f %f %f", pose.pose.pose.position.x, pose.pose.pose.position.y, tf::getYaw(pose.pose.pose.orientation) * 180 / M_PI);
        map[0].x = pose.pose.pose.position.x;
        map[0].y = pose.pose.pose.position.y;
        map_orientation[0] = tf::getYaw(pose.pose.pose.orientation);

    } //poseCallback

    //GRAPHICAL_DISPLAY
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void populateMarkerTopic()
    {

        visualization_msgs::Marker marker;

        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "example";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.orientation.w = 1;

        marker.scale.x = 0.05;
        marker.scale.y = 0.05;

        marker.color.a = 1.0;

        for (int loop = 0; loop < nb_pts; loop++)
        {
            geometry_msgs::Point p;
            std_msgs::ColorRGBA c;

            p.x = display[loop].x;
            p.y = display[loop].y;
            p.z = display[loop].z;

            c.r = colors[loop].r;
            c.g = colors[loop].g;
            c.b = colors[loop].b;
            c.a = colors[loop].a;

            //ROS_INFO("(%f, %f, %f) with rgba (%f, %f, %f, %f)", p.x, p.y, p.z, c.r, c.g, c.b, c.a);
            marker.points.push_back(p);
            marker.colors.push_back(c);
        }

        pub_localization_marker.publish(marker);
    }

    // Distance between two points
    float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb)
    {

        return sqrt(pow((pa.x - pb.x), 2.0) + pow((pa.y - pb.y), 2.0));
    }
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "localization");

    localization bsObject;

    ros::spin();

    return 0;
}
