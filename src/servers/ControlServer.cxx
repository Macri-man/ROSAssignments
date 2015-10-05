#include "ros/ros.h"
#include "assignment1/Messager.h"
#include <sstream>
#include <string>
#include <cstdlib>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>

//from http://cboard.cprogramming.com/c-programming/63166-kbhit-linux.html#post803210

int kbhit(){
struct termios oldt, newt;
  int ch;
  int oldf;
 
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
  ch = getchar();
 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
 
  if(ch != EOF){
    ungetc(ch, stdin);
    return 1;
  }
 
  return 0;
}

int swap(std::string command){
  if(command=="start"){
    return 1;
  }else if(command=="stop"){
    return 2;
  }else if(command=="pause"){
    return 3;
  }else if(command==" "){
    return 4;
  }else{
    return -1;
  }
}

int state=0;

int input =0;

bool control(assignment1::Messager::Request  &req,assignment1::Messager::Response &res){
  if(input==0){
  std::cout << "Enter Command: \n";
  input=7;
  }
  std::string command;
  char c;
  if(kbhit()){
    //std::cout << "keyboard hit";
    while((c=std::cin.get())!='\n'){
      command+=c;
      //std::cout << c;
    }
    //std::cout << command;
    ROS_INFO("Command Received: [%s]", command.c_str());
    if(command.empty()){
      ROS_INFO("No Command has been received!");
    }else if(!(command == "start" || command == "stop" ||  command =="pause")){
      ROS_INFO("Wrong Input: [%s]", command.c_str());
    }else{
      res.state=state=swap(command);
    }
  }else{
    //std::cout <<"state:" << state;
    res.state=state;
  }
  return true;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "control_server");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("control_messages", control);
  ROS_INFO("Starting Control Server!");
  ros::spin();

  return 0;
}