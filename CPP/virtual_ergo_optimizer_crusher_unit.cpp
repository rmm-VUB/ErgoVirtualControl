// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>
#include <unistd.h>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>

#include "examples_common.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>	/* for fprintf */
#include <string.h>	/* for memcpy */


#include <stdlib.h>
#include <netdb.h>
#include <arpa/inet.h>

#define PORT 8053
#define BUFSIZE 4
#define BUFSIZE_2 24
#define REMOTE_IP_ADDRESS "134.184.20.59"
#define UDP_PORT_1 8054
#define UDP_PORT_2 "8055"
#define UDP_PORT_3 8056

/**
 * @example generate_cartesian_pose_motion.cpp
 * An example showing how to generate a Cartesian motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */


// Robot-table angle
double beta = 0.7131;

struct sockaddr_in myaddr;      /* our address */
struct sockaddr_in myaddr2;      /* our address */
struct sockaddr_in remaddr;     /* remote address */
socklen_t addrlen = sizeof(remaddr);            /* length of addresses */
int recvlen;                    /* # bytes received */
int fd;                         /* our socket */
int fd2;
uint8_t buf[BUFSIZE];     /* receive buffer */
uint8_t buf2[BUFSIZE_2];

int result = 0;
int socketSend = socket(AF_INET, SOCK_DGRAM, 0);
sockaddr_storage addrDest = {};

void go_to_joint_position(franka::Robot &robot, std::array<double, 7> pos_joint)
{
  MotionGenerator motion_generator(0.5, pos_joint);
  robot.control(motion_generator);
}

void go_to_cart_position(franka::Robot &robot, std::array<double, 7> desired_pose)
{
  // Set additional parameters always before the control loop, NEVER in the control loop!
  // Set collision behavior.
  robot.setCollisionBehavior(
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

  std::array<double, 16> initial_pose;
  double time = 0.0;
  double delta_x = 0;
  double delta_y = 0;
  double delta_z = 0;
  robot.control([&time, &initial_pose, &desired_pose, &delta_x, &delta_y, &delta_z](const franka::RobotState& robot_state,
                                       franka::Duration period) -> franka::CartesianPose {

    if (time == 0.0) {
      initial_pose = robot_state.O_T_EE_c;
      std::cout << initial_pose[0] << " " << initial_pose[4] << " " << initial_pose[8] << " " << initial_pose[12] << std::endl;
      std::cout << initial_pose[1] << " " << initial_pose[5] << " " << initial_pose[9] << " " << initial_pose[13] << std::endl;
      std::cout << initial_pose[2] << " " << initial_pose[6] << " " << initial_pose[10] << " " << initial_pose[14] << std::endl;
      std::cout << initial_pose[3] << " " << initial_pose[7] << " " << initial_pose[11] << " " << initial_pose[15] << std::endl;
    }

    time += period.toSec();

    std::array<double, 16> new_pose = initial_pose;


    double angle = M_PI / 4 * (1 - std::cos(M_PI / 3.0 * time));
    delta_x = (desired_pose[0] - initial_pose[12]) * std::sin(angle);
    delta_y = (desired_pose[1] - initial_pose[13]) * std::sin(angle);
    delta_z = -(desired_pose[2] - initial_pose[14]) * (std::cos(angle) - 1);
    new_pose[12] += delta_x;
    new_pose[13] += delta_y;
    new_pose[14] += delta_z;


    if (time >= 3.0) {
      std::cout << new_pose[0] << " " << new_pose[4] << " " << new_pose[8] << " " << new_pose[12] << std::endl;
      std::cout << new_pose[1] << " " << new_pose[5] << " " << new_pose[9] << " " << new_pose[13] << std::endl;
      std::cout << new_pose[2] << " " << new_pose[6] << " " << new_pose[10] << " " << new_pose[14] << std::endl;
      std::cout << new_pose[3] << " " << new_pose[7] << " " << new_pose[11] << " " << new_pose[15] << std::endl;

      std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
      return franka::MotionFinished(new_pose);
    }
    return new_pose;
  });
}

int gripper_grasp(franka::Gripper &gripper, double gripper_width)
{
  if (!gripper.grasp(gripper_width, 0.1, 180)) {
    std::cout << "Failed to grasp object." << std::endl;
    return -1;
  }
}

void wait_user_force_on_robot(franka::Robot &robot)
{
  size_t count = 0;
  robot.read([&count](const franka::RobotState& robot_state) {
    //std::cout << robot_state.K_F_ext_hat_K[1] << std::endl;
    return (robot_state.K_F_ext_hat_K[1] < 5);
  });
}

void wait_user_force_on_robot_2(franka::Robot &robot)
{
  size_t count = 0;
  robot.read([&count](const franka::RobotState& robot_state) {
    //std::cout << robot_state.K_F_ext_hat_K[1] << std::endl;
    return (robot_state.K_F_ext_hat_K[1] > -1.2);
  });
}

void wait_user_force_on_robot_Z(franka::Robot &robot)
{
  size_t count = 0;
  robot.read([&count](const franka::RobotState& robot_state) {
    //std::cout << robot_state.K_F_ext_hat_K[2] << std::endl;
    return (robot_state.K_F_ext_hat_K[2] > -5);
  });
}


int resolvehelper(const char* hostname, int family, const char* service, sockaddr_storage* pAddr)
{
    int result;
    addrinfo* result_list = NULL;
    addrinfo hints = {};
    hints.ai_family = family;
    hints.ai_socktype = SOCK_DGRAM; // without this flag, getaddrinfo will return 3x the number of addresses (one for each socket type).
    result = getaddrinfo(hostname, service, &hints, &result_list);
    if (result == 0)
    {
        //ASSERT(result_list->ai_addrlen <= sizeof(sockaddr_in));
        memcpy(pAddr, result_list->ai_addr, result_list->ai_addrlen);
        freeaddrinfo(result_list);
    }

    return result;
}

int initialize_UDP()
{
  /* create a UDP socket */
  if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
         perror("cannot create socket\n");
         return 0;
  }

  /* create a UDP socket */
  if ((fd2 = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
         perror("cannot create socket\n");
         return 0;
  }

  /* create a UDP socket */
  if ((socketSend = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
         perror("cannot create socket\n");
         return 0;
  }

  if (bind(socketSend, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
             perror("bind failed");
             return 0;
  }

  result = resolvehelper(REMOTE_IP_ADDRESS, AF_INET, UDP_PORT_2, &addrDest);
  if (result != 0)
  {
     int lasterror = errno;
     std::cout << "error: " << lasterror;
     exit(1);
  }

  /* bind the socket to any valid IP address and a specific port */

  memset((char *)&myaddr, 0, sizeof(myaddr));
  myaddr.sin_family = AF_INET;
  myaddr.sin_addr.s_addr = htonl(INADDR_ANY); //inet_addr("134.184.20.59");
  myaddr.sin_port = htons(PORT);

  memset((char *)&myaddr2, 0, sizeof(myaddr2));
  myaddr2.sin_family = AF_INET;
  myaddr2.sin_addr.s_addr = htonl(INADDR_ANY); //inet_addr("134.184.20.59");
  myaddr2.sin_port = htons(UDP_PORT_3);

  if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
             perror("bind failed");
             return 0;
  }

  if (bind(fd2, (struct sockaddr *)&myaddr2, sizeof(myaddr2)) < 0) {
             perror("bind failed");
             return 0;
  }

  return 1;
}

int receive_step_UDP()
{
  recvlen = recvfrom(fd, buf, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen);
  //std::cout << recvlen << std::endl;

  /*if(recvlen > 0)
    buf[recvlen] = 0;*/

  int value;
  assert(sizeof value == sizeof bytes);
  std::memcpy(&value, buf, sizeof buf);

  //if (recvlen > 0)
  //  buf[recvlen] = 0;


  return value;
}

double *receive_x_y_z_UDP()
{
  recvlen = recvfrom(fd2, buf2, BUFSIZE_2, 0, (struct sockaddr *)&remaddr, &addrlen);
  //std::cout << unsigned(buf2[0]) << " " << unsigned(buf2[1]) << " " << unsigned(buf2[2]) << " " << unsigned(buf2[3]) << std::endl;

  double static value[3] = {0,0,0};

  uint8_t f_x[8] = {buf2[0], buf2[1], buf2[2], buf2[3], buf2[4], buf2[5], buf2[6], buf2[7]};
  value[0] = *(double*)(f_x);
  uint8_t f_y[8] = {buf2[8], buf2[9], buf2[10], buf2[11], buf2[12], buf2[13], buf2[14], buf2[15]};
  value[1] = *(double*)(f_y);
  uint8_t f_z[8] = {buf2[16], buf2[17], buf2[18], buf2[19], buf2[20], buf2[21], buf2[22], buf2[23]};
  value[2] = *(double*)(f_z);

  std::cout << value[0] << " " << value[1] << " " << value[2] << std::endl;

  return value;
}

int send_UDP_completion()
{
  const char* msg = "Step completed";
  size_t msg_length = strlen(msg);

  result = sendto(socketSend, msg, msg_length, 0, (sockaddr*)&addrDest, sizeof(addrDest));

  //std::cout << result << " bytes sent" << std::endl;

  return 0;
}

int send_UDP_restart()
{
  const char* msg = "R";
  size_t msg_length = strlen(msg);

  result = sendto(socketSend, msg, msg_length, 0, (sockaddr*)&addrDest, sizeof(addrDest));

  //std::cout << result << " bytes sent" << std::endl;

  return 0;
}

void go_to_ergo_opt_cart_position(franka::Robot &robot, double delta[])
{
  // Set additional parameters always before the control loop, NEVER in the control loop!
  // Set collision behavior.
  robot.setCollisionBehavior(
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

  std::array<double, 16> initial_pose;
  std::array<double, 7> desired_pose;
  double time = 0.0;
  double delta_x = 0;
  double delta_y = 0;
  double delta_z = 0;

  robot.control([&time, &initial_pose, &desired_pose, &delta, &delta_x, &delta_y, &delta_z](const franka::RobotState& robot_state,
                                       franka::Duration period) -> franka::CartesianPose {
    if (time == 0.0) {
      initial_pose = robot_state.O_T_EE_c;

      //std::cout << delta[0] << " " << delta[1] << " " << delta[2] << std::endl;

      desired_pose[0] = initial_pose[12] + delta[0]*std::cos(beta) + delta[1]*std::sin(beta);
      desired_pose[1] = initial_pose[13] - delta[0]*std::sin(beta) + delta[1]*std::cos(beta);
      desired_pose[2] = initial_pose[14] + delta[2];

      if(desired_pose[0] > 0.563172)
        desired_pose[0] = 0.563172;
      if(desired_pose[0] < 0.182412)
        desired_pose[0] = 0.182412;
      if(desired_pose[1] > 0.267527)
        desired_pose[1] = 0.267527;
      if(desired_pose[1] < -0.380281)
        desired_pose[1] = -0.380281;
      if(desired_pose[2] < 0.189644)
        desired_pose[2] = 0.189644;
      if(desired_pose[2] > 0.65)
        desired_pose[2] = 0.65;

      std::cout << desired_pose[0] << " " << desired_pose[1] << " " << desired_pose[2] << std::endl;

      std::cout << initial_pose[0] << " " << initial_pose[4] << " " << initial_pose[8] << " " << initial_pose[12] << std::endl;
      std::cout << initial_pose[1] << " " << initial_pose[5] << " " << initial_pose[9] << " " << initial_pose[13] << std::endl;
      std::cout << initial_pose[2] << " " << initial_pose[6] << " " << initial_pose[10] << " " << initial_pose[14] << std::endl;
      std::cout << initial_pose[3] << " " << initial_pose[7] << " " << initial_pose[11] << " " << initial_pose[15] << std::endl;

    }

    time += period.toSec();

    std::array<double, 16> new_pose = initial_pose;

    double angle = M_PI / 4 * (1 - std::cos(M_PI / 7.0 * time));
    delta_x = (desired_pose[0] - initial_pose[12]) * std::sin(angle);
    delta_y = (desired_pose[1] - initial_pose[13]) * std::sin(angle);
    delta_z = -(desired_pose[2] - initial_pose[14]) * (std::cos(angle) - 1);
    new_pose[12] += delta_x;
    new_pose[13] += delta_y;
    new_pose[14] += delta_z;

    //std::cout << delta_x << " " << delta_y << " " << delta_z << std::endl;

    if (time >= 7.0) {
      std::cout << new_pose[0] << " " << new_pose[4] << " " << new_pose[8] << " " << new_pose[12] << std::endl;
      std::cout << new_pose[1] << " " << new_pose[5] << " " << new_pose[9] << " " << new_pose[13] << std::endl;
      std::cout << new_pose[2] << " " << new_pose[6] << " " << new_pose[10] << " " << new_pose[14] << std::endl;
      std::cout << new_pose[3] << " " << new_pose[7] << " " << new_pose[11] << " " << new_pose[15] << std::endl;

      std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
      return franka::MotionFinished(new_pose);
    }
    return new_pose;
  });
}

void workpiece_ergo_control(franka::Robot &robot)
{
  sleep(1.0);
  double *delta = receive_x_y_z_UDP();
  go_to_ergo_opt_cart_position(robot, delta);
  //std::array<double, 7> rod_app_high = {0.47538295119,-0.0681770180493,0.2+0.118002948086-0.1,0.677013464547,0.73591168926,0.00880899424013,0.00302589715407};
  //go_to_cart_position(robot, rod_app_high);
}

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  try {

    //Maximum velocity
    double v_x_max = 2;
    double v_y_max = 2;
    double v_z_max = 2;

    // Joint angles
    std::array<double, 7> initial_pos_joint = {-0.0763579367947308, -0.9152327316985118, -0.00135821759865534, -2.2422722254845566, -0.008388358608985196, 1.337538437048594, -1.7518735996931791};
    std::array<double, 7> initial_pos_joint_2 = {-0.0763579367947308, -0.9152327316985118, -0.00135821759865534, -2.2422722254845566, -0.008388358608985196, 1.337538437048594, -0.18107727289};
    std::array<double, 7> initial_pos_joint_3 = {-0.0763579367947308, -0.9152327316985118, -0.00135821759865534, -2.2422722254845566, -0.008388358608985196, 1.337538437048594, -1.7518735996931791 + M_PI};
    std::array<double, 7> intermediate_pos_joint = {-0.2586675837416398, -0.15784268217620448, -0.021192822508073503, -2.204761657146043, -0.016421418794327312, 2.028810453843351, -1.927692936266462};
    std::array<double, 7> handing_pos = {0.7079572590568609, -0.6153276628193096, 0.00787296861817127, -1.919187662409063, 0.011063109041648336, 1.314036370118459, 0.688364014495081};//{0.33293761014355605, -0.7425065630777301, 0.16685304511714907, -1.7323635018108878, 0.14626717961496774, 0.9844012093544006, -2.615755547716579};
    // Picking position and orientation
    std::array<double, 7> rod_app_high = {0.47538295119,-0.0681770180493,0.2+0.118002948086-0.1,0.677013464547,0.73591168926,0.00880899424013,0.00302589715407};
    std::array<double, 7> rod_app_low = {0.47538295119,-0.0681770180493,0.05+0.118002948086-0.1,0.677013464547,0.73591168926,0.00880899424013,0.00302589715407};
    std::array<double, 7> rod_pick = {0.47538295119,-0.0681770180493,0.118002948086-0.004-0.1,0.677013464547,0.73591168926,0.00880899424013,0.00302589715407};
    std::array<double, 7> left_plate_app_high = {0.462643, -0.24194, 0.0165316+0.2,0.999083701265,0.0395503691333,0.00494139374111,0.0155919465067};
    std::array<double, 7> left_plate_app_low = {0.462643, -0.24194, 0.0165316+0.05,0.999083701265,0.0395503691333,0.00494139374111,0.0155919465067};
    std::array<double, 7> left_plate_pick = {0.462643,-0.24194, 0.0165316,0.999083701265,0.0395503691333,0.00494139374111,0.0155919465067};
    std::array<double, 7> right_plate_app_high = {0.394879,-0.164866,0.0159059+0.2,0.998580387807,0.051519340051,0.0135097495209,0.000673316443777};
    std::array<double, 7> right_plate_app_low = {0.394879,-0.164866,0.0159059+0.05,0.998580387807,0.051519340051,0.0135097495209,0.000673316443777};
    std::array<double, 7> right_plate_pick = {0.394879,-0.164866,0.0159059,0.998580387807,0.051519340051,0.0135097495209,0.000673316443777};
    std::array<double, 7> hub_app_high = {0.538121,-0.190488,0.0159222+0.2,0.740363903816,-0.671988784968,0.0170841842369,0.000702461833624};
    std::array<double, 7> hub_app_low = {0.538121,-0.190488,0.0159222+0.05,0.740363903816,-0.671988784968,0.0170841842369,0.000702461833624};
    std::array<double, 7> hub_pick = {0.538121,-0.190488,0.0159222,0.740363903816,-0.671988784968,0.0170841842369,0.000702461833624};
    std::array<double, 7> motor_app_high = {0.771307,-0.00528941,0.0631558+0.2,-0.746815804064,0.664700720054,-0.020710352638,0.00319199838125};
    std::array<double, 7> motor_app_low = {0.771307,-0.00528941,0.0631558+0.1,-0.746815804064,0.664700720054,-0.020710352638,0.00319199838125};
    std::array<double, 7> motor_pick = {0.771307,-0.00528941,0.0631558,-0.746815804064,0.664700720054,-0.020710352638,0.00319199838125};

    std::array<double, 7> left_plate_app_above_assembly = {0.64042+0.025*std::cos(beta), 0.105943-0.025*std::sin(beta), 0.0540818+0.12, 0.679134479358, 0.733806798457, 0.0135432985125, 0.010978185217};
    std::array<double, 7> left_plate_app_close_assembly = {0.64042+0.025*std::cos(beta), 0.105943-0.025*std::sin(beta), 0.0540818, 0.679134479358, 0.733806798457, 0.0135432985125, 0.010978185217};
    std::array<double, 7> left_plate_final_pos = {0.64042, 0.105943, 0.0540818, 0.679134479358, 0.733806798457, 0.0135432985125, 0.010978185217};
    std::array<double, 7> left_plate_retreat = {0.64042, 0.105943, 0.0540818+0.2, 0.679134479358, 0.733806798457, 0.0135432985125, 0.010978185217};

    std::array<double, 7> right_plate_app_above_assembly = {0.540636-0.025*std::cos(beta), 0.186364+0.025*std::sin(beta), 0.0512421+0.12, 0.679134479358, 0.733806798457, 0.0135432985125, 0.010978185217};
    std::array<double, 7> right_plate_app_close_assembly = {0.540636-0.025*std::cos(beta), 0.186364+0.025*std::sin(beta), 0.0512421, 0.679134479358, 0.733806798457, 0.0135432985125, 0.010978185217};
    std::array<double, 7> right_plate_final_pos = {0.540636, 0.186364, 0.0512421, 0.679134479358, 0.733806798457, 0.0135432985125, 0.010978185217};
    std::array<double, 7> right_plate_retreat = {0.540636, 0.186364, 0.0512421+0.2, 0.679134479358, 0.733806798457, 0.0135432985125, 0.010978185217};

    std::array<double, 7> motor_final_pos = {0.771307-0.045*std::cos(beta),-0.00528941+0.045*std::sin(beta),0.0631558,-0.746815804064,0.664700720054,-0.020710352638,0.00319199838125};
    std::array<double, 7> motor_retreat = {0.771307-0.045*std::cos(beta),-0.00528941+0.045*std::sin(beta),0.0631558+0.2,-0.746815804064,0.664700720054,-0.020710352638,0.00319199838125};

    std::array<double, 2> finger_pos_plate_open = {0.03983044624328613, 0.03983044624328613};
    std::array<double, 2> finger_pos_hub_open = {0.021082939580082893, 0.021082939580082893};
    std::array<double, 2> finger_pos_motor_open = {0.039839308708906174, 0.039839308708906174};
    std::array<double, 2> finger_pos_plate_close = {0.036181021481752396, 0.036181021481752396};
    std::array<double, 2> finger_pos_hub_close = {0.014438786543905735, 0.014438786543905735};
    std::array<double, 2> finger_pos_rod_close = {0.0031073465943336487, 0.0031073465943336487};
    std::array<double, 2> finger_pos_motor_close = {0.020519191399216652, 0.020519191399216652};

    double width_gripper_rod_open = 0.05;
    double width_gripper_plate_open = 0.12;
    double width_gripper_rod_close = 0.004;
    double width_gripper_hub_close = 0.025;
    double width_gripper_plate_close = 0.07;
    double width_gripper_motor_close = 0.04;

    int step = 0;
    int nbSteps = 12;

    initialize_UDP();

    //std::cout << receive_step_UDP() << std::endl;

    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    franka::Gripper gripper(argv[1]);

    std::cout << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    go_to_joint_position(robot, initial_pos_joint);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    //gripper.homing();
    //franka::GripperState gripper_state = gripper.readOnce();

    step = 8;
    wait_user_force_on_robot_Z(robot);
    send_UDP_completion();
    while(step <= nbSteps)
    {

      std::cout << "Step: " << step << std::endl;
      if(step != 2)
        receive_step_UDP();

      if(step == 1)
      {
        gripper.move(width_gripper_rod_open, 0.05);
        go_to_joint_position(robot, initial_pos_joint_3);
        go_to_cart_position(robot, rod_pick);
        gripper_grasp(gripper, width_gripper_rod_close);
        go_to_cart_position(robot, rod_app_high);
        go_to_joint_position(robot, handing_pos);
        sleep(1.0);
        wait_user_force_on_robot_2(robot);
      }

      if(step == 2 || step == 9)
      {
        workpiece_ergo_control(robot);
        sleep(1.0);
        wait_user_force_on_robot(robot);
        gripper.move(width_gripper_rod_open, 0.05);
        go_to_joint_position(robot, initial_pos_joint_3);
        //send_UDP_restart();
      }

      if(step == 3)
      {

      }

      if(step == 4)
      {
        gripper.move(width_gripper_plate_open, 0.05);
        go_to_joint_position(robot, initial_pos_joint_2);
        go_to_cart_position(robot, left_plate_app_high);
        //go_to_cart_position(robot, left_plate_app_low);
        go_to_cart_position(robot, left_plate_pick);
        gripper_grasp(gripper, width_gripper_plate_close);//width_gripper_plate_close);
        go_to_joint_position(robot, intermediate_pos_joint);
        go_to_cart_position(robot, left_plate_app_above_assembly);
        go_to_cart_position(robot, left_plate_app_close_assembly);
        go_to_cart_position(robot, left_plate_final_pos);
        gripper.move(width_gripper_plate_open, 0.05);
        go_to_cart_position(robot, left_plate_retreat);
        go_to_joint_position(robot, initial_pos_joint);
      }

      if(step == 5)
      {

      }

      if(step == 6)
      {
        gripper.move(width_gripper_plate_open, 0.05);
        go_to_joint_position(robot, initial_pos_joint_2);
        go_to_cart_position(robot, right_plate_app_high);
        go_to_cart_position(robot, right_plate_pick);
        gripper_grasp(gripper, width_gripper_plate_close);//width_gripper_plate_close);
        go_to_joint_position(robot, intermediate_pos_joint);
        go_to_cart_position(robot, right_plate_app_above_assembly);
        go_to_cart_position(robot, right_plate_app_close_assembly);
        go_to_cart_position(robot, right_plate_final_pos);
        gripper.move(width_gripper_plate_open, 0.05);
        go_to_cart_position(robot, right_plate_retreat);
        go_to_joint_position(robot, initial_pos_joint);
      }

      if(step == 7)
      {

      }

      if(step == 8)
      {
        gripper.move(width_gripper_rod_open, 0.05);
        go_to_cart_position(robot, hub_pick);
        gripper_grasp(gripper, width_gripper_hub_close);
        go_to_cart_position(robot, hub_app_high);
        go_to_joint_position(robot, handing_pos);
        sleep(1.0);
        wait_user_force_on_robot_2(robot);
        sleep(1.0);
      }

      if(step == 10)
      {

      }

      if(step == 11)
      {
        gripper.move(width_gripper_plate_open, 0.05);
        go_to_joint_position(robot, initial_pos_joint);
        go_to_cart_position(robot, motor_app_high);
        go_to_cart_position(robot, motor_pick);
        gripper_grasp(gripper, width_gripper_motor_close);
        go_to_cart_position(robot, motor_final_pos);
        gripper.move(width_gripper_plate_open, 0.05);
        go_to_cart_position(robot, motor_retreat);
        go_to_joint_position(robot, initial_pos_joint);
      }

      if(step == 12)
      {

      }

      if(step == 3 or step == 5 or step == 7 or step == 10 or step == 12)
        wait_user_force_on_robot_Z(robot);
      sleep(1.0);
      send_UDP_completion();
      step += 1;
      std::cout << "Message sent: " << std::endl;
    }

    return 0;

  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
