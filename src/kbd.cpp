/*
 * 2015-12-15 s133961, Example ROS keyboard control for ARDrone
 */

#include <iostream>
#include <signal.h>
#include <termios.h>
#include <ros/ros.h>
#include <std_msgs/Char.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_SPACE 0x70

int kfd = 0;
struct termios cooked, raw;

class keyboard_controller {
    public:
        keyboard_controller();
        void key_loop();

    private:
        ros::NodeHandle _node;
        ros::Publisher _pub;
        std_msgs::Char _msg;
};

keyboard_controller::keyboard_controller() {
    _pub = _node.advertise<std_msgs::Char>("/ardrone/kbd_commands", 1);
}

// Signal handler
// TODO does not send 'Q' to drone on Ctrl+C
void quit(int sig) {
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

int main(int argc, char** argv) {
    ros::init(argc, argv,"AR_kbd");
    keyboard_controller kc;

    signal(SIGINT, quit);

    kc.key_loop();

    return(0);
}

void keyboard_controller::key_loop() {
    char char_in;
    bool dirty = false;

    // Get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);

    // Setting a new line, then end of file                         
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard, use arrow keys to control drone");
    puts("'Q' to quit, 'T' to take off, 'L' to land, 'R' to reset");
    puts("'space' to hover, 'a' thrust +, 'z' thrust -");
    puts("---------------------------");

    for(;;)
    {
        // get the next event from the keyboard  
        if(read(kfd, &char_in, 1) < 0) {
            perror("read():");
            exit(-1);
        }

        switch(char_in) {
            case KEYCODE_L:
                _msg.data = 'h';
                dirty = true;
                break;
            case KEYCODE_R:
                _msg.data = 'l';
                dirty = true;
                break;
            case KEYCODE_U:
                _msg.data = 'k';
                dirty = true;
                break;
            case KEYCODE_D:
                _msg.data = 'j';
                dirty = true;
                break;
            case 'Q':
                _msg.data = 'Q';
                _pub.publish(_msg);    
                tcsetattr(kfd, TCSANOW, &cooked);
                ros::shutdown();
                exit(0);
            case 'T':
                _msg.data = 'T';
                dirty = true;
                break;
            case 'L':
                _msg.data = 'L';
                dirty = true;
                break;
            case 32:
                _msg.data = 'x';
                dirty = true;
                break;
            case 'a':
                _msg.data = 'a';
                dirty = true;
                break;
            case 'z':
                _msg.data = 'z';
                dirty = true;
                break;
            case 'R':
                _msg.data = 'R';
                dirty = true;
                break;
        }

        if(dirty ==true) {
            _pub.publish(_msg);    
            dirty=false;
        }
    }

    return;
}
