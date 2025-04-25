/*
 * subposecpp.cpp
 *
 *  Created on: April 9, 2024
 *      Author: utayba
 * This program subscribes to turtle1/pose and shows its
 * messages on the screen.
 */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <termios.h>

#include <chrono> //time library in system, allows for timer creation
#include <functional>
#include <memory>
using namespace std::chrono_literals;

using std::placeholders::_1;

class subPose : public rclcpp::Node
{
public:
  subPose() : Node("subscribe_pose")
  {
    subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("HWSWfinalTopic", 10, std::bind(&subPose::topic_callback, this, _1));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("BumpDriver", 100);
    timer_ = this->create_wall_timer( 13ms, std::bind(&subPose::bumper_data, this));
    

    tty = open("/dev/ttyACM1", O_RDWR | O_NOCTTY | O_NDELAY);
    // tty_fd=open("/dev/ttyACM2", O_RDWR | O_NONBLOCK);
    tcgetattr(tty, &tio);

    memset(&tio, 0, sizeof(tio));

    tio.c_cflag &= ~CSIZE;               // clear the CSIZE flag bits
    tio.c_cflag |= CS8 | CREAD | CLOCAL; // set data size to 8 bits and enable receiving data.
    tio.c_cflag &= ~PARENB;              // No Parity
    tio.c_cflag &= ~CSTOPB;              // 1 stop bit
    // tio.c_lflag=0;

    cfsetospeed(&tio, B115200);
    cfsetispeed(&tio, B115200);

    tcsetattr(tty, TCSAFLUSH, &tio);

    if (tty < 0)
      std::cout << "error connecting to the robot!!\n";
    else
      std::cout << "connecting to the robot successfuly\n";

    //bumper_data();
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  double X, Y;
  char Drive = 'G';
  int tty, Z, R;
  //int R = 0;
  struct termios tio;
  struct termios stdio;
  struct termios old_stdio;
  int tty_fd;
  unsigned char c = 'D';
  char rxString[10];
  int ind, bUP;
  // float voltage, voltageY;
  // double atdNew, atdNewY;
  char inchar = 'b';
  geometry_msgs::msg::Twist msg1;

  void topic_callback(const geometry_msgs::msg::Twist &msg)
  {
    // RCLCPP_INFO(this->get_logger(),"position=(%.2f , %.2f) ,direction= %.2f",msg.linear.x, msg.linear.y,msg.angular.z)
  

    X = double(msg.linear.x);
    Y = double(msg.linear.y);
    Z = int(msg.linear.z);
    R = int(msg.angular.x);

    if (X > 0.2)
    {
      Drive = 'A';
    }
    else if (X < -0.2)
    {
      Drive = 'D';
    }
    else if (Y > 0.2)
    {
      Drive = 'W';
    }
    else if (Y < -0.2)
    {
      Drive = 'S';
    }
    else if (Z == 6)
    {
      Drive = 'P';
    }
    else if (Z == 5){
      Drive = 'R';
    }
    else if(R == 1)
    {
      Drive = 'C';
    }
    else
      Drive = 'H';


    RCLCPP_INFO(this->get_logger(),"direction= %d, Party: %d",R, Z);

    // printf("Output: %c\n", Drive);
    write(tty, &Drive, 1);

    
  }

  void bumper_data()
  {
    ///while (c != 'q')
    //{
      if (read(tty, &inchar, 1) > 0)
      {
        if (inchar == 'p')
        {
          rxString[0] = inchar;
          ind = 1;
          //printf("got P");
          while (inchar != '\n')
          {
            if (read(tty, &inchar, 1) > 0)
            {
              if (ind < 10)
              {
                rxString[ind] = inchar;
                ind++;
              }
              else
              {
                inchar = '\n';
              }
            }
          }
          if (ind < 10)
          {
            sscanf(rxString, "p: %d", &bUP);
            // printf("ATDX: %d\n",bUP);

            //RCLCPP_INFO(this->get_logger(), "Bumper Val =%d", bUP);
            // msg.linear.x=atdNew;
            msg1.angular.z = bUP;
            publisher_->publish(msg1);
          }
        }
      }
      //read(STDIN_FILENO, &c, 1);
    //}
    //close(tty);
    //tcsetattr(STDOUT_FILENO, TCSANOW, &old_stdio);
  }

  void timer_callback()
  {
        
    geometry_msgs::msg::Twist msg;
    msg.linear.x = double(rand())/double(RAND_MAX); //Rand_MAX is a constant that can be output from the rand() function. This division sets the speed from 0-1
    msg.angular.z = (2*double(rand())/double(RAND_MAX) - 1) * M_PI; //This gives me a range from -1 to +1
    RCLCPP_INFO(this->get_logger(), "Publishing linear speed: '%f' and angular speed: '%f'\n", msg.linear.x,msg.angular.z);
    publisher_->publish(msg);
  }

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<subPose>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
