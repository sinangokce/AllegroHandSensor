#include "grasp_sensor.h"
#include <stdio.h>
//#include <algorithm>
#include <iterator>

#define RADIANS_TO_DEGREES(radians) ((radians) * (180.0 / M_PI))
#define DEGREES_TO_RADIANS(angle) ((angle) / 180.0 * M_PI)

int joint[16];
int stop_table[16];
int condinit;
float speed_Percentage=1;
float hand_Direction=0;
double desired_position[DOF_JOINTS] = {0.0};
double current_position[DOF_JOINTS] = {0.0};
double previous_position[DOF_JOINTS] = {0.0};
double distance[DOF_JOINTS] = {0.0};
int back = 0;
int reverse = 0;
int reverse_table[16];
int desiredisgreater[16];
int startclosing = 1;
int wentback;
int wentback_condition;
int first_run;
int a; 
float sensor_data[8];

AllegroNodeGraspController::AllegroNodeGraspController() {
         
  initControllerxx();

  grasp_type_sub = nh.subscribe("allegroHand_0/libsss_cmd", 1, &AllegroNodeGraspController::graspTypeControllerCallback, this);

  SpeedPer_sub = nh.subscribe("/lwr/speedPercentage", 10, &AllegroNodeGraspController::speedPerCallback, this);

  sensor_data_sub = nh.subscribe("/LasaDataStream", 10, &AllegroNodeGraspController::sensorDataCallback, this);

  desired_state_pub = nh.advertise<sensor_msgs::JointState>("allegroHand_0/joint_cmd", 1);

  next_state_sub = nh.subscribe(NEXT_STATE_TOPIC, 1, &AllegroNodeGraspController::nextStateCallback, this);

  current_state_pub = nh.advertise<sensor_msgs::JointState>(CURRENT_LISTENER_TOPIC, 1);

  stop_pub = nh.advertise<std_msgs::String>(STOP_TOPIC, 1); 

  wentback_pub = nh.advertise<std_msgs::String>("allegroHand_0/libsss_cmd", 1);
}

AllegroNodeGraspController::~AllegroNodeGraspController() {
  delete mutex;
}

void AllegroNodeGraspController::speedPerCallback(const handtracker::spper &msg) {
  speed_Percentage = msg.sPer;
  hand_Direction = msg.dir;
}

void AllegroNodeGraspController::sensorDataCallback(const glove_tekscan_ros_wrapper::LasaDataStreamWrapper &msg) {

  std::cout << "hey" << msg.ring2_f[10] + msg.ring2_f[11] + msg.pinky2_f[8] + msg.ring2_s[9] + msg.ring2_s[10] + msg.ring2_s[11] << std::endl;
  std::cout << msg.ring3_f[4]+msg.ring3_f[5]+msg.ring3_f[6] + msg.index1_s[9]+msg.index1_s[10]+msg.index1_s[11] + msg.middle3_s[7] + msg.ring3_s[0]+msg.ring3_s[1]+msg.ring3_s[2]+msg.ring3_s[4]+msg.ring3_s[5] << std::endl;
  std::cout << msg.middle1_f[0]+msg.middle1_f[1]+msg.middle1_f[2]+msg.middle1_f[3]+msg.middle1_f[4]+msg.middle1_f[5]+msg.middle1_f[6]+msg.index1_s[7]+msg.middle1_s[0]+msg.middle1_s[1]+msg.middle1_s[2]+msg.middle1_s[4]+msg.middle1_s[5] << std::endl;

  sensor_data[0] = average((msg.ring2_f[10] + msg.ring2_f[11] + msg.pinky2_f[8] + msg.ring2_s[9] + msg.ring2_s[10] + msg.ring2_s[11]),  6);
  sensor_data[1] = average((msg.ring3_f[4]+msg.ring3_f[5]+msg.ring3_f[6] + msg.index1_s[9]+msg.index1_s[10]+msg.index1_s[11] + msg.middle3_s[7] + msg.ring3_s[0]+msg.ring3_s[1]+msg.ring3_s[2]+msg.ring3_s[4]+msg.ring3_s[5]), 12);

  sensor_data[2] = average((msg.middle1_f[0]+msg.middle1_f[1]+msg.middle1_f[2]+msg.middle1_f[3]+msg.middle1_f[4]+msg.middle1_f[5]+msg.middle1_f[6]+msg.index1_s[7]+msg.middle1_s[0]+msg.middle1_s[1]+msg.middle1_s[2]+msg.middle1_s[4]+msg.middle1_s[5]), 13);
  sensor_data[3] = average((msg.index2_f[0]+msg.index2_f[1]+msg.index2_f[2]+msg.index2_f[3]+msg.index2_s[0]+msg.index2_s[1]+msg.index2_s[2]), 7);

  sensor_data[4] = average((msg.ring1_f[0]+msg.ring1_f[1]+msg.ring1_f[2]+msg.ring1_f[3]+msg.ring1_f[4]+msg.ring1_f[5]+msg.ring1_f[6]+msg.ring1_s[0]+msg.ring1_s[1]+msg.ring1_s[2]+msg.ring1_s[4]+msg.ring1_s[5]), 12);
  sensor_data[5] = average((msg.middle1_f[14]+msg.middle1_f[15]+msg.middle2_f[0]+msg.middle2_f[1]+msg.middle2_f[2]+msg.middle2_f[3]+msg.middle1_s[10]+msg.middle1_s[11]+msg.middle1_s[13]+msg.middle1_s[14]+msg.middle1_s[15]+msg.middle2_s[0]+msg.middle2_s[1]+msg.middle2_s[2]), 14);

  sensor_data[6] = average((msg.ring1_f[14]+msg.ring1_f[15]+msg.pinky1_f[0]+msg.pinky1_f[1]+msg.pinky1_f[2]+msg.pinky1_f[3]+msg.pinky1_f[4]+msg.pinky1_f[5]+msg.pinky1_f[6]+msg.pinky1_f[12]+msg.ring1_s[7]+msg.ring1_s[10]+msg.ring1_s[11]+msg.ring1_s[13]+msg.ring1_s[14]+msg.ring1_s[15]+msg.pinky1_s[0]+msg.pinky1_s[1]+msg.pinky1_s[2]+msg.pinky1_s[4]+msg.pinky1_s[5]+msg.pinky1_s[8]), 22);
  sensor_data[7] = average((msg.ring2_f[0]+msg.ring2_f[1]+msg.ring2_f[2]+msg.ring2_f[3]+msg.ring2_s[0]+msg.ring2_s[1]+msg.ring2_s[2]+msg.ring2_s[3]), 8);

  /*for (int i = 0; i < 8; i++)
    ROS_INFO("sensor: %f", sensor_data[i]);**/
}

void AllegroNodeGraspController::graspTypeControllerCallback(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO("CTRL: Heard: [%s]", msg->data.c_str());
  const std::string grasp_type = msg->data;

  std_msgs::String stop_msg;
  std::stringstream stop_ss;
  
  condinit = 0;
  a = 1;
  back = 0;
  wentback_condition = 0;

  if (grasp_type.compare("open") == 0) {

    for (int i = 0; i < DOF_JOINTS; i++) {
      joint[i] = 0;
      stop_table[i] = 0;
    }

    reverse = 1;
    back = 1;
    startclosing = 0;

    stop_ss << "open";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);
  }

  else if (grasp_type.compare("stop") == 0) {
    for (int i = 0; i < DOF_JOINTS; i++) {
      //joint[i] = 0;
      stop_table[i] = 1;
    }

    reverse = 0;
    back = 0;
    startclosing = 0;

    stop_ss << "stop";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);
  }

  else if (grasp_type.compare("close") == 0) {
    for (int i = 0; i < DOF_JOINTS; i++) {
      joint[i] = 0;
      stop_table[i] = 0;
    }

    reverse = 0;
    back = 0;
    startclosing = 1;

    stop_ss << "close";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);
  }

  else if (grasp_type.compare("home") == 0) {
    condinit = 1;
    //startclosing = 1;

    for (int i = 0; i < DOF_JOINTS; i++)
      desired_position[i] = home_pose[i]; 

    stop_ss << "false";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);
  }

  else if (grasp_type.compare("power") == 0) {
    condinit = 1;
    
    for (int i = 0; i < DOF_JOINTS; i++) {
      desired_position[i] = power[i];
      desiredisgreater[i] = 0;
    }
    
    stop_ss << "false";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);
  }

  else if (grasp_type.compare("thumb") == 0) {
    condinit = 1;

    for (int i = 0; i < DOF_JOINTS; i++) {
      desired_position[i] = thumb[i];
      desiredisgreater[i] = 0;
    }
    
    stop_ss << "false";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);
  }

  else if (grasp_type.compare("pinch") == 0) {
    condinit = 1;

    for (int i = 0; i < DOF_JOINTS; i++) {
      desired_position[i] = pinch[i];
      desiredisgreater[i] = 0;
    }
  
    stop_ss << "false";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);
  }

  else if (grasp_type.compare("lateral") == 0) {
    condinit = 1;

    for (int i = 0; i < DOF_JOINTS; i++) {
      desired_position[i] = lateral[i]; 
      desiredisgreater[i] = 0;
    }
   
    stop_ss << "false";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);
  }

  else if (grasp_type.compare("little_tactile") == 0) {
    stop_ss << "little_tactile";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);
    for (int i = 8; i < 12; i++) {
      stop_table[i] = 1;
    }
  }

  else if (grasp_type.compare("middle_tactile") == 0) {
    stop_ss << "middle_tactile";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);
    for (int i = 4; i < 8; i++) {
      stop_table[i] = 1;
    }
  }

  else if (grasp_type.compare("index_tactile") == 0) {
    stop_ss << "index_tactile";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);
    for (int i = 0; i < 4; i++) {
      stop_table[i] = 1;
    }
  }

  else if (grasp_type.compare("thumb_tactile") == 0) {
    stop_ss << "thumb_tactile";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);
    for (int i = 12; i < 16; i++) {
      stop_table[i] = 1;
    }
  }

  else if (grasp_type.compare("wentback") == 0) {
    condinit = 1;
    wentback_condition = 1;
    a = 0;
  }

  if (condinit == 1) {

    if (first_run != 1) {
      if(wentback_condition == 0) {

        reverse = 1;
        back = 1;
        startclosing = 0;

        std_msgs::String stop_msg;
        std::stringstream stop_ss;
        stop_ss << "open";
        stop_msg.data = stop_ss.str();
        stop_pub.publish(stop_msg);

        current_state.position.resize(DOF_JOINTS);

        for (int i = 0; i < DOF_JOINTS; i++) {
          distance[i] = current_state.position[i] - home_pose[i];
          current_state.velocity[i] = (distance[i]/6000);
          joint[i] = 0;
          stop_table[i] = 0;
          reverse_table[i] = 0;
        }
        wentback_condition = 1;
        if(a == 1)
          current_state_pub.publish(current_state);
      }

      else if (wentback_condition == 1){
        
        reverse = 0;
        back = 0;
        startclosing = 1;

        std_msgs::String stop_msg;
        std::stringstream stop_ss;
        stop_ss << "close";
        stop_msg.data = stop_ss.str();
        stop_pub.publish(stop_msg);

        current_state.position.resize(DOF_JOINTS);

        for (int i = 0; i < DOF_JOINTS; i++) {
          distance[i] = desired_position[i] - current_state.position[i];
          current_state.velocity[i] = (distance[i]/8000);
          joint[i] = 0;
          stop_table[i] = 0;
          reverse_table[i] = 0;
        }
        current_state_pub.publish(current_state);
      }
    }

    else {
      startclosing = 1;
      current_state.position.resize(DOF_JOINTS);

      for (int i = 0; i < DOF_JOINTS; i++) {
        current_state.position[i] = home_pose[i];
      }

      for (int i = 0; i < DOF_JOINTS; i++) {
        distance[i] = desired_position[i] - current_state.position[i];
        current_state.velocity[i] = (distance[i]/8000);
        joint[i] = 0;
        stop_table[i] = 0;
        reverse_table[i] = 0;
      }

      first_run = 0;
      current_state_pub.publish(current_state);
    }

  }
}

void AllegroNodeGraspController::nextStateCallback(const sensor_msgs::JointState &msg) {
  std_msgs::String wentback_msg;
  std::stringstream wentback_ss;
  
  current_state = msg;

  //The stop condition changes if the hand moves backward
  if (back == 1) {

    if (reverse == 1) {   //just to fill the table only once
      for (int i = 0; i < DOF_JOINTS; i++) {                  
        if (current_state.position[i] < home_pose[i]) 
        reverse_table[i] = 1;
      }
      reverse = 0; //just to fill the table only once
    }
  
    for (int i = 0; i < DOF_JOINTS; i++) {
      if (reverse_table[i] == 0 && current_state.position[i] <= home_pose[i]) 
        joint[i] = 1;                                                                 
      else if(reverse_table[i] == 1 && current_state.position[i] >= home_pose[i])
        joint[i] = 1;
    }


    for (int i = 0; i < DOF_JOINTS; i++) {
      if (joint[i] == 1 && stop_table[i] == 0) {
        current_state.position[i] = home_pose[i];           //when a joint arrived to home pose it should stop at his posiiton
      }
    }

    if (condinit == 1) {
      if ( checkEquality(joint) ) {
        wentback_ss << "wentback";
        wentback_msg.data = wentback_ss.str(); //this check is useful for initializion for a new pose.
        wentback_pub.publish(wentback_msg);
      }
      
    }   

    current_state_pub.publish(current_state);
  }

  //The stop condition if hand moves forward
  else if (back != 1){

    if(startclosing == 1) {  //just to fill the table only once
      for (int i = 0; i < DOF_JOINTS; i++) {
        if (current_state.position[i] <= desired_position[i]) 
          desiredisgreater[i] = 1;
        else 
          desiredisgreater[i] = 0;
      }
      startclosing = 0; //just to fill the table only once
    }  

    for (int i = 0; i < DOF_JOINTS; i++) {
      if (desiredisgreater[i] == 1 && current_state.position[i] >= desired_position[i]) 
        joint[i] = 1;
      else if (desiredisgreater[i] == 0 && current_state.position[i] <= desired_position[i]) 
        joint[i] = 1;
    }
  
    for (int i = 0; i < DOF_JOINTS; i++) {
      if (joint[i] == 1 && stop_table[i] == 0) {
        current_state.position[i] = desired_position[i];    //when a joint arrived to desired position it should stop at his posiiton
      }
    }

    current_state_pub.publish(current_state);
  }  

  desired_state_pub.publish(current_state);
}

bool AllegroNodeGraspController::checkEquality(int array[]) {
  for (int i = 0; i < DOF_JOINTS; i++) {
    if(array[i] != 1)
      return false;
  }
  return true;
}

float AllegroNodeGraspController::average(int sum, int number) {

  float avg;

  avg = sum / number;

  return avg;
}



void AllegroNodeGraspController::initControllerxx() {
  current_state.position.resize(DOF_JOINTS);
  current_state.velocity.resize(DOF_JOINTS);
  
  desired_state.position.resize(DOF_JOINTS);
  desired_state.velocity.resize(DOF_JOINTS);

  first_run = 1;

  printf("*************************************\n");
  printf("         Grasp (BHand) Method        \n");
  printf("-------------------------------------\n");
  printf("         Every command works.        \n");
  printf("*************************************\n");
}

void AllegroNodeGraspController::doIt() {
  ros::Rate rate(250.0);
  while (ros::ok()) {
    rate.sleep();
    ros::spinOnce();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "allegro_hand_core_grasp");
  AllegroNodeGraspController grasping;

  grasping.doIt();
}