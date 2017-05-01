#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// transfer ascii value to hex
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77

#define KEYCODE_A 0x61
#define KEYCODE_S 0x73

#define KEYCODE_Z 0x7A
#define KEYCODE_X 0x78

#define KEYCODE_E 0x65
#define KEYCODE_R 0x72

#define KEYCODE_D 0x64
#define KEYCODE_F 0x66

#define KEYCODE_C 0x63
#define KEYCODE_V 0x76

class KeyboardTeleopNode
{
	private:
	// joint velocity
	double v_j1 = 0.02;
	double v_j2 = 0.02;
	double v_j3 = 0.02;
	double v_j4 = 0.05;
	double v_j5 = 0.05;
	double v_j6 = 0.05;

	ros::NodeHandle n_;
	ros::Publisher pub_;
	sensor_msgs::JointState cmdjs_;


    public:
        KeyboardTeleopNode()
        {
            pub_ = n_.advertise<sensor_msgs::JointState>("/joint_states", 1000);

            cmdjs_.name.clear();
            cmdjs_.name.push_back("joint_1");
            cmdjs_.name.push_back("joint_2");
            cmdjs_.name.push_back("joint_3");
			cmdjs_.name.push_back("joint_4");
			cmdjs_.name.push_back("joint_5");
			cmdjs_.name.push_back("joint_6");

			cmdjs_.position.clear();
            for(int i=0;i<6;++i){
            	cmdjs_.position.push_back(0.0f);
            }

            // needed header time info, or the rviz can not display the model states
            cmdjs_.header.stamp= ros::Time::now();
            pub_.publish(cmdjs_);


            ros::NodeHandle n_private("~");

        }
        
        ~KeyboardTeleopNode() { }
        void keyboardLoop();
        
        void stopRobot()
        {

        	cmdjs_.header.stamp= ros::Time::now();
            pub_.publish(cmdjs_);

        }
};

KeyboardTeleopNode* tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"tbk", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    KeyboardTeleopNode tbk;
    
    boost::thread t = boost::thread(boost::bind(&KeyboardTeleopNode::keyboardLoop, &tbk));
    
    ros::spin();
    
    t.interrupt();
    t.join();
    tbk.stopRobot();
    tcsetattr(kfd, TCSANOW, &cooked);
    
    return(0);
}

void KeyboardTeleopNode::keyboardLoop()
{
    char c;
    bool dirty = false;
    
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    
    puts("Reading from keyboard");
    puts("Use \"q,w, a,s, z,x, e,r, d,f, c,v\" keys to control the robot");
    
    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;
    
    for(;;)
    {
        boost::this_thread::interruption_point();
        
        // get the next event from the keyboard
        int num;
        
        if ((num = poll(&ufd, 1, 250)) < 0)
        {
            perror("poll():");
            return;
        }
        else if(num > 0)
        {
            if(read(kfd, &c, 1) < 0)
            {
                perror("read():");
                return;
            }
        }
        else
        {
            if (dirty == true)
            {
                stopRobot();
                dirty = false;
            }

            continue;
        }
        
        // change the joints position
        switch(c)
        {
            case KEYCODE_Q:
            	if(cmdjs_.position[0]<3.14){
            		cmdjs_.position[0]+=v_j1;
            	}
                dirty = true;
                break;
            case KEYCODE_W:
				if (cmdjs_.position[0] > -3.14) {
					cmdjs_.position[0] -= v_j1;
				}
                dirty = true;
                break;
            case KEYCODE_A:
				if (cmdjs_.position[1] < 1.91) {
					cmdjs_.position[1] += v_j2;
				}
                dirty = true;
                break;
            case KEYCODE_S:
				if (cmdjs_.position[1] > -1.74) {
					cmdjs_.position[1] -= v_j2;
				}
				dirty = true;
                break;
			case KEYCODE_Z:
				if (cmdjs_.position[2] < 1.1) {
				cmdjs_.position[2]+= v_j3;
				}
				dirty = true;
				break;
			case KEYCODE_X:
				if (cmdjs_.position[2] > -1.0) {
				cmdjs_.position[2]-= v_j3;
				}
				dirty = true;
				break;
			case KEYCODE_E:
				if (cmdjs_.position[3] < 3.4) {
				cmdjs_.position[3]+= v_j4;
				}
				dirty = true;
				break;
			case KEYCODE_R:
				if (cmdjs_.position[3] > -3.4) {
				cmdjs_.position[3]-= v_j4;
				}
				dirty = true;
				break;
			case KEYCODE_D:
				if (cmdjs_.position[4] < 2.0) {
				cmdjs_.position[4]+= v_j5;
				}
				dirty = true;
				break;
			case KEYCODE_F:
				if (cmdjs_.position[4] > -2.0) {
				cmdjs_.position[4]-= v_j5;
				}
				dirty = true;
				break;
			case KEYCODE_C:
				if (cmdjs_.position[5] < 6.9) {
				cmdjs_.position[5]+= v_j6;
				}
				dirty = true;
				break;
			case KEYCODE_V:
				if (cmdjs_.position[5] > -6.9) {
				cmdjs_.position[5]-= v_j6;
				}
				dirty = true;
				break;

            default:
                dirty = false;
        }
        //send /joint_states message
        cmdjs_.header.stamp= ros::Time::now();
        pub_.publish(cmdjs_);
    }
}
