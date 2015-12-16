/*
 * 2015-12-14 s133961, Example ROS code for ARDrone
 */

#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

// Drone states
#define LANDED 0
#define TAKEOFF 1
#define HOVER 2
#define FLY 3
#define LANDING 4

// How to much to increase/decrease values with for each key press
#define STEP 0.15
// Maximum allowed value
#define LIMIT 0.75
// After DELTA we automatically go into hover mode
// After 2 * DELTA we land
#define DELTA 3
// Reset, land and take off time
#define RESET_TIME 2
#define LAND_TIME 4
#define TAKE_OFF_TIME 4

// This type of message is used to send commands to the drone
std_msgs::Empty emp_msg;	

// Messages used to define movement
geometry_msgs::Twist twist_msg;

// Rest of the variables
int drone_state = LANDED, new_msg = 0;
char kbd_input = '0';
double timer, watchdog;
bool first_watchdog_called = false;
bool second_watchdog_called = false;

// Callback called when published data is received
void kbd_cb(const std_msgs::CharConstPtr& msg) {
    kbd_input = msg->data;
    new_msg = 1;
    // ROS_INFO("Got command %s", &kbd_input);
}

// Set hover
void set_hover() {
    twist_msg.linear.x = 0.0; 
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = 0.0;
    twist_msg.angular.x = 0.0; 
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.0;
}

// Check values is below limit
void check_values() {
    if (twist_msg.linear.x >= LIMIT) { twist_msg.linear.x = LIMIT; }
    if (twist_msg.linear.x <= -LIMIT) { twist_msg.linear.x = -LIMIT; }
    if (twist_msg.linear.y >= LIMIT) { twist_msg.linear.y = LIMIT; }
    if (twist_msg.linear.y <= -LIMIT) { twist_msg.linear.y = -LIMIT; }
    if (twist_msg.linear.z >= LIMIT) { twist_msg.linear.z = LIMIT; }
    if (twist_msg.linear.z <= 0) { twist_msg.linear.z = 0; }
}

// Print values
void print_values() {
    ROS_INFO("Linear values (+/-):\n\tx (left/right) %.2f,\n\ty (forward/backward) %.2f,\n\tz (up/down) %.2f", twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z);
}

// Reset watchdog
void reset_watchdog() {
    watchdog = (double)ros::Time::now().toSec();
    first_watchdog_called = false;
    second_watchdog_called = false;
}

// Land
void action(int state_one, int state_two, 
            const char *msg_one, const char *msg_two, 
            ros::Publisher &pub_action, ros::Publisher &pub_twist,
            int time, ros::Rate loop_rate) {

    // Set timer to now
    timer = (double)ros::Time::now().toSec();

    // Set all values to zero, hover
    set_hover();

    // Set state and give message during action
    drone_state = state_one;
    ROS_INFO("%s", msg_one);

    // Give the drone time to perform action
    while ((double)ros::Time::now().toSec()< timer + time) { 
        pub_twist.publish(twist_msg);
        pub_action.publish(emp_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Set state and give message after action
    drone_state = state_two;
    ROS_INFO("%s", msg_two);
}

// Steer
void steer(int state, const char* msg, ros::Publisher &pub) {
    kbd_input = '0';
    reset_watchdog();
    check_values();
    drone_state = state;
    pub.publish(twist_msg);
    ROS_INFO("%s", msg);
    print_values();
}

// Good old main
int main(int argc, char** argv) {
    // Initializing stuff
    ros::init(argc, argv,"AR_fly");
    ros::NodeHandle node;

    // One ROS loop is 50 ms
    ros::Rate loop_rate(50);

    // Subscriber and publishers
    ros::Publisher pub_takeoff;
    ros::Publisher pub_land;
    ros::Publisher pub_twist;
    ros::Publisher pub_reset;
    ros::Subscriber sub;

    set_hover();

    // Subscribe to commands send from keyboard node
    sub = node.subscribe("/ardrone/kbd_commands", 1, kbd_cb);

    // Set up what topics we publishes to the drone
    pub_twist = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    pub_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1);
    pub_reset = node.advertise<std_msgs::Empty>("/ardrone/reset", 1);

    // Wait until we recieve a command
    ROS_INFO("Waiting for input");
    while (!new_msg) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    watchdog = (double)ros::Time::now().toSec();

    // Main ROS loop
    while (ros::ok() && new_msg) {
        
        // Reset
        if (kbd_input == 'R') {
            ROS_INFO("Reset start");

            // If drone is not landed, land it before reset
            if (drone_state != LANDED) {
            action(LANDING, LANDED, 
                   "Landing", "Land", 
                   pub_land, pub_twist,
                   LAND_TIME, loop_rate);
            }

            // Reset values to zero, hover
            timer = (double)ros::Time::now().toSec();
            set_hover();
            ros::spinOnce();
            loop_rate.sleep();
            drone_state = LANDED;

            // Give the drone time to reset
            while ((double)ros::Time::now().toSec()< timer + RESET_TIME) { 
                pub_reset.publish(emp_msg);
                ros::spinOnce();
                loop_rate.sleep();
            }

            kbd_input = '0';
            ROS_INFO("Reset done");
        }

        // Take off
        if (drone_state == LANDED && kbd_input == 'T') {
            action(TAKEOFF, HOVER, 
                   "Take off", "Hover, ready to fly", 
                   pub_land, pub_twist,
                   TAKE_OFF_TIME, loop_rate);

            // Reset watchdog
            watchdog = (double)ros::Time::now().toSec();
        }

        // Quit 
        if (kbd_input == 'Q') {
            if (drone_state != LANDED) {
            action(LANDING, LANDED, 
                   "Landing", "Landed", 
                   pub_land, pub_twist,
                   LAND_TIME, loop_rate);
            }

            ros::spinOnce();
            loop_rate.sleep();
            ROS_INFO("Quit");
            exit(0);
        }

        // Land 
        if ((drone_state != LANDED && drone_state != LANDING) && 
                kbd_input == 'L') {  

            action(LANDING, LANDED, 
                   "Landing", "Land", 
                   pub_land, pub_twist,
                   LAND_TIME, loop_rate);

            if (kbd_input == 'q') {
                ros::spinOnce();
                loop_rate.sleep();
                ROS_INFO("Quit");
                exit(0);
            }
        }

        // Hover
        if ((drone_state == HOVER || drone_state == FLY ) 
                && kbd_input == 'x') {
            set_hover();
            steer(HOVER, "Hover", pub_twist);
        }

        // Left
        if ((drone_state == HOVER || drone_state == FLY) &&
                kbd_input == 'h') {
            twist_msg.linear.y += STEP;
            steer(FLY, "Left", pub_twist);
        }

        // Right
        if ((drone_state == HOVER || drone_state == FLY) &&
                kbd_input == 'l') {
            twist_msg.linear.y -= STEP;
            steer(FLY, "Right", pub_twist);
        }

        // Forward
        if ((drone_state == HOVER || drone_state == FLY) &&
                kbd_input == 'k') {
            twist_msg.linear.x += STEP;
            steer(FLY, "Forward", pub_twist);
        }

        // Backward
        if ((drone_state == HOVER || drone_state == FLY) &&
                kbd_input == 'j') {
            twist_msg.linear.x -= STEP;
            steer(FLY, "Backward", pub_twist);
        }

        // Up
        if ((drone_state == HOVER || drone_state == FLY) &&
                kbd_input == 'a') {
            twist_msg.linear.z += STEP;
            steer(FLY, "Up", pub_twist);
        }

        // Down
        if ((drone_state == HOVER || drone_state == FLY) &&
                kbd_input == 'z') {
            twist_msg.linear.z -= STEP;
            steer(FLY, "Down", pub_twist);
        }

        // No keypress for DELTA time, hover
        if (drone_state != LANDED && !first_watchdog_called &&
                (((double)ros::Time::now().toSec()) - watchdog) > DELTA) {
            ROS_INFO("Watchdog timer: DELTA reached - hover");
            first_watchdog_called = true;
            set_hover();
            kbd_input = '0';
        }
        
        // No keypress for 2 * DELTA time, land
        if (drone_state != LANDED && !second_watchdog_called &&
                (((double)ros::Time::now().toSec()) - watchdog) > (2 * DELTA)) {
            ROS_INFO("Watchdog timer: 2 * DELTA reached - land");
            second_watchdog_called = true;
            set_hover();

            timer = (double)ros::Time::now().toSec();
            drone_state = LANDING;
            ROS_INFO("Landing");

            // Give the drone time to land 
            while ((double)ros::Time::now().toSec()< timer + LAND_TIME) { 
                pub_twist.publish(twist_msg);
                pub_land.publish(emp_msg);
                ros::spinOnce();
                loop_rate.sleep();
            }

            drone_state = LANDED;
            ROS_INFO("Landed");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    exit(0);
}
