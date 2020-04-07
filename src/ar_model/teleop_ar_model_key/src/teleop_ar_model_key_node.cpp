#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <signal.h>
#include <stdio.h>
#ifndef _WIN32
#include <termios.h>
#include <unistd.h>
#else
#include <windows.h>
#endif
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_B 0x62
#define KEYCODE_C 0x63
#define KEYCODE_D 0x64
#define KEYCODE_E 0x65
#define KEYCODE_F 0x66
#define KEYCODE_G 0x67
#define KEYCODE_Q 0x71
#define KEYCODE_R 0x72
#define KEYCODE_T 0x74
#define KEYCODE_V 0x76

class KeyboardReader
{
public:
  KeyboardReader()
#ifndef _WIN32
    : kfd(0)
#endif
  {
#ifndef _WIN32
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    struct termios raw;
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
#endif
  }
  void readOne(char * c)
  {
#ifndef _WIN32
    int rc = read(kfd, c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }
#else
    for(;;)
    {
      HANDLE handle = GetStdHandle(STD_INPUT_HANDLE);
      INPUT_RECORD buffer;
      DWORD events;
      PeekConsoleInput(handle, &buffer, 1, &events);
      if(events > 0)
      {
        ReadConsoleInput(handle, &buffer, 1, &events);
        if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_LEFT)
        {
          *c = KEYCODE_LEFT;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_UP)
        {
          *c = KEYCODE_UP;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_RIGHT)
        {
          *c = KEYCODE_RIGHT;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_DOWN)
        {
          *c = KEYCODE_DOWN;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x42)
        {
          *c = KEYCODE_B;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x43)
        {
          *c = KEYCODE_C;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x44)
        {
          *c = KEYCODE_D;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x45)
        {
          *c = KEYCODE_E;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x46)
        {
          *c = KEYCODE_F;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x47)
        {
          *c = KEYCODE_G;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x51)
        {
          *c = KEYCODE_Q;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x52)
        {
          *c = KEYCODE_R;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x54)
        {
          *c = KEYCODE_T;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x56)
        {
          *c = KEYCODE_V;
          return;
        }
      }
    }
#endif
  }
  void shutdown()
  {
#ifndef _WIN32
    tcsetattr(kfd, TCSANOW, &cooked);
#endif
  }
private:
#ifndef _WIN32
  int kfd;
  struct termios cooked;
#endif
};

KeyboardReader input;

class TeleopARModel
{
public:
    TeleopARModel();
    void keyLoop();
    geometry_msgs::Pose pose_;
private:

    void statesCallBack(const gazebo_msgs::ModelStates& new_states);
    ros::NodeHandle nh_;
    double linear_, angular_, l_scale_, a_scale_;
    double move_up_, move_down_;
    double stop_;
    ros::Publisher twist_pub_;
    ros::Subscriber pose_sub_;
};

TeleopARModel::TeleopARModel():
    linear_(0),
    angular_(0),
    l_scale_(0.01),
    a_scale_(0.3),
    move_up_(0),
    move_down_(0),
    stop_(0)
{
    //nh_.param("scale_angular", a_scale_, a_scale_);
    //nh_.param("scale_linear", l_scale_, l_scale_);

    twist_pub_ = nh_.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",1);
    pose_sub_ = nh_.subscribe("/gazebo/model_states",1, &TeleopARModel::statesCallBack, this);
}

// find AR_model's pose
void TeleopARModel::statesCallBack(const gazebo_msgs::ModelStates& new_states)
{
   ros::V_string::const_iterator it=std::find(new_states.name.begin(), new_states.name.end(), "ar_model");
   int index = std::distance(new_states.name.begin(), it);
   //std::cout << "index:" << index << std::endl;
   pose_ = new_states.pose[index];
}

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_ar_model");
  TeleopARModel teleop_ar_model;

  signal(SIGINT,quit);
  
  teleop_ar_model.keyLoop();
  quit(0);
  
  return(0);
}

void TeleopARModel::keyLoop()
{
    
    char c;
    bool dirty=false;

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the turtle. 'T': Take off. 'G' Land. 'R' Stop. 'q' to quit.");

    while(ros::ok())
    { 
        ros::spinOnce();
        try
        {
            input.readOne(&c);
        }
        catch(const std::runtime_error &)
        {
            perror("read():");
            return;
        }

        linear_= angular_= 0;
        ROS_DEBUG("value: 0x%02X\n", c);

        switch (c)
        {
            case KEYCODE_LEFT:
                ROS_DEBUG("LEFT");
                angular_ = 1.0;
                dirty=true;
                stop_ = 0;
                break;
            case KEYCODE_RIGHT:
                ROS_DEBUG("RIGHT");
                angular_ = -1.0;
                dirty = true;
                stop_= 0;
                break;
            case KEYCODE_UP:
                ROS_DEBUG("UP");
                linear_ = 1.0;
                dirty = true;
                stop_ = 0;
                break;
            case KEYCODE_DOWN:
                ROS_DEBUG("DOWN");
                linear_ = -1.0;
                dirty = true;
                stop_ = 0;
                break;
            case KEYCODE_T:
                ROS_DEBUG("MOVE UP");
                move_up_= 1;
                dirty = 1;
                stop_ = 0;
                break;
            case KEYCODE_G:
                ROS_DEBUG("MOVE DOWN");
                move_up_ = -1;
                dirty =1;
                stop_ = 0;
                break;
            case KEYCODE_R:
                ROS_DEBUG("STOP");
                stop_=1;
                dirty = 1;
            case KEYCODE_Q:
                ROS_DEBUG("quit");
                return;     
        }


        geometry_msgs::Twist twist;
        if(stop_ == 1)
        {
            angular_= linear_ = move_up_= 0;
        }

        pose_.position.y = pose_.position.y + a_scale_* angular_;
        pose_.position.x = pose_.position.x + l_scale_*linear_;
        pose_.position.z = pose_.position.z + l_scale_*move_up_;

        if(dirty == true)
        {
            gazebo_msgs::ModelState model_state;
            model_state.model_name= "ar_model";
            model_state.pose = pose_;
            //model_state.twist=twist;
            twist_pub_.publish(model_state);
            move_up_ = 0;
        }
    }
}