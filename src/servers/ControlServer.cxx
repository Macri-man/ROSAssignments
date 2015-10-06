#include "ros/ros.h"
#include "assignment1/Messager.h"
#include <sstream>
#include <string>
#include <cstdlib>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <iostream>

int swap(std::string command){
  if(command=="start"){
    return 1;
  }else if(command=="stop"){
    return 2;
  }else if(command=="pause"){
    return 3;
  }else if(command=="quit"){
    ros::shutdown();
  }else{
    return -1;
  }
}

int state=0;
int input=0;

bool control(assignment1::Messager::Request  &req,assignment1::Messager::Response &res){
  if(input==0){
    std::cout << "Enter Command: \n";
    input=7;
  }
  std::string command;
  char c;
  while((c=getchar())!=EOF || !command.empty()){

    if(c==EOF) continue;

    if(c=='\n') {
      std::cerr << "\n";
      break;
    }
      if((c=='\b' || c=='\x7f') && !command.empty()){
        std::cerr << "\b \b";
        command.resize(command.length()-1);
      }else if((c=='\b' || c=='\x7f') && command.empty()){
        std::cerr << "\r                                          \r";
        command.clear();
      }else if(c >= 32 && c<128) {
        std::cerr << c;
        command+=c;
      }
    }
    
    if(command.empty()){
      //ROS_INFO("No Command has been received!");
      switch(state){
        case 1:
          res.state=state=4;
          break;
        case 2:
          res.state=state=5;
          break;
        case 3:
          res.state=state=6;
          break;
        case -1:
          res.state=state;
        default:
          res.state=state;
      }
    }else if(!(command == "start" || command == "stop" ||  command =="pause" || command == "quit")){
      ROS_INFO("Wrong Input: [%s]", command.c_str());
    }else{
      ROS_INFO("Command sent: [%s]", command.c_str());
      res.command=command;
      res.state=state=swap(command);
    }
  
  return true;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "control_messages");

struct termios oldt, newt;
  int ch;
  int oldf;
 
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);


  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("control_messages", control);
  ROS_INFO("Starting Control Service!");
  ros::spin();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  return 0;
}