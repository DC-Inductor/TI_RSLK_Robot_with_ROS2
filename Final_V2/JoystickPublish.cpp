/*
 * analogReadcpp.cpp
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


#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <inttypes.h>



using std::placeholders::_1;
using namespace std::chrono_literals;
 
class analogRead : public rclcpp::Node {
public:
	analogRead():Node("analog_publisher"){
		publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("HWSWfinalTopic", 100);	
	
		subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("BumpDriver", 10, std::bind(&analogRead::bumper_callback, this, _1));
		
		timer_ = this->create_wall_timer( 5ms, std::bind(&analogRead::capture_data, this));

		srand(time(0));
		tcgetattr(STDOUT_FILENO,&old_stdio);
	
		memset(&stdio,0,sizeof(stdio));

		stdio.c_lflag |= ECHO;
		stdio.c_iflag |= ICRNL;
		stdio.c_oflag |= (OPOST | ONLCR);

		tcsetattr(STDOUT_FILENO,TCSAFLUSH,&stdio);
		fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);       // make the reads non-blocking

		tty_fd=open("/dev/ttyACM0", O_RDWR | O_NONBLOCK);
		tcgetattr(tty_fd,&tio);


		tio.c_cflag &= ~CSIZE;
		tio.c_cflag|= CS8|CREAD|CLOCAL;           // 8n1, see termios.h for more information
		tio.c_cflag &= ~PARENB; //No Parity
		tio.c_cflag &= ~CSTOPB; // 1 stop bit
		tio.c_lflag=0;

		cfsetospeed(&tio,B115200);
		cfsetispeed(&tio,B115200);
		tcsetattr(tty_fd,TCSAFLUSH,&tio);
		capture_data();

	}

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
	rclcpp::TimerBase::SharedPtr timer_;
	struct termios tio;
	struct termios stdio;
	struct termios old_stdio;
	int tty_fd,  B;
	int bumperCount = 0;
	unsigned char c='D';
	char Bxstring[20];
    char rxString[40];
	int ind,bumperIndex,atd,atdY,bState,Reset, goCray;
	float voltage, voltageY;
    double atdNew, atdNewY;
	char inchar='b';
	char bumperstop = 'G';
	bool rescued = false;
	geometry_msgs::msg::Twist msg;
	


//  void capture_data(const turtlesim::msg::Pose & msg) const {
//    RCLCPP_INFO(this->get_logger(),"position=(%.2f , %.2f) ,direction= %.2f",msg.x, msg.y,msg.theta);
void capture_data(){	
	//while (c!='q')
	//{
	if (read(tty_fd,&inchar,1)>0){
		if (inchar == 'd') {
			rxString[0]=inchar;
			ind=1;
			while(inchar != '\n'){
				if(read(tty_fd,&inchar,1)>0){
					if (ind<40) {
						rxString[ind]=inchar;
						ind++;

					} else {
						inchar = '\n';
					}
				}
			}
			if (ind<40) {
				sscanf(rxString,"%*s %d\t%f\t%d\t%f\t%d\t",&atd, &voltage, &atdY, &voltageY, &bState);
				//printf("ATDX: %d\t VoltageX: %.2f\t ATDY: %d\t VoltageY: %.2f\n",atd, voltage, atdY, voltageY);
					if (bumperCount >= 8){
						if (bState == 5){
							goCray = 0;
							bumperCount = 0;
							}
						else {
							goCray = 1;
							}
						// write(tty_fd, &bumperstop, 1);
						printf("Output: %d\n", goCray);

 						}
				atdNew = double(atd - 512)/512;
				atdNewY = double(atdY - 512)/512;
				// RCLCPP_INFO(this->get_logger(),"newATDx=(%.2f) newATDy=(%.2f) Button=(%.2d)",atdNew,atdNewY,bState);
				msg.linear.x=atdNew;
				msg.linear.y=atdNewY;
				msg.linear.z=bState;
				msg.angular.x = goCray;
				msg.angular.y = Reset;
				publisher_->publish(msg);
			}
		}
	}



}

void bumper_callback(const geometry_msgs::msg::Twist &msg){

B = int(msg.angular.z);

if(B == 1){
	bumperCount++;
	printf("bumper count = %d\n", bumperCount);	
 }

//  if (bumperCount >= 8){
// 	if (bumperCount >= 8 && bState == 5){
// 		bumperstop = 'R';
// 		bumperCount = 0;
// 	}
// 	else {
// 		bumperstop = 'C';

// 	}
// write(tty_fd, &bumperstop, 1);

//  }


//  if(bumperCount == 8 && bState != 5){
// 	bumperstop = 'L';
// 	write(tty_fd, &bumperstop, 1);
//  }
//  else if(bumperCount >= 8 && bState == 5){
// 	bumperstop = 'G';
// 	bumperCount = 0;
// 	write(tty_fd, &bumperstop, 1);
//  }


// printf("Output: %c\n", bumperstop);

}

};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node=std::make_shared<analogRead>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
