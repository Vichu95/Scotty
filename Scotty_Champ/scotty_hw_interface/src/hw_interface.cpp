#include <ros/ros.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <string.h>
#include <byteswap.h>
#include <vector>


///////////////////////////////////////
//     C O N T R O L    V A L U E S 
///////////////////////////////////////
#define DEFAULT_V    0.0f
#define DEFAULT_KP   5.0f
#define DEFAULT_KD   0.1f
#define DEFAULT_TRQ  0.0f


///////////////////////////////////////
//        L E G    I N D E X I N G
///////////////////////////////////////
// Define Joint Type Indices
#define ABAD 0
#define HIP  1
#define KNEE 2

// ROS Joint Indexing Order. It is from ros_control.yaml in scotty_config/config
#define ROS_FL_INDEX 0
#define ROS_FR_INDEX 1
#define ROS_RL_INDEX 2
#define ROS_RR_INDEX 3

// SPI Structure Joint Indexing (MIT Order: FR first, FL second)
#define MIT_FR_INDEX 0
#define MIT_FL_INDEX 1
#define MIT_RR_INDEX 0
#define MIT_RL_INDEX 1

// Define SPI Device
#define SPI_DEVICE_1 "/dev/spidev2.1"
#define SPI_DEVICE_2 "/dev/spidev2.0"
#define K_WORDS_PER_MESSAGE 66

unsigned char spi_mode = SPI_MODE_0;
unsigned char spi_bits_per_word = 8;
unsigned int spi_speed = 6000000;   // MIT original speed
uint8_t lsb = 0x01;

uint8_t checksumErrCnt_dev1 = 0;
uint8_t checksumErrCnt_dev2 = 0;

// SPI Command Structure (Like MIT)
typedef struct {
  float q_des_abad[2];
  float q_des_hip[2];
  float q_des_knee[2];
  float qd_des_abad[2];
  float qd_des_hip[2];
  float qd_des_knee[2];
  float kp_abad[2];
  float kp_hip[2];
  float kp_knee[2];
  float kd_abad[2];
  float kd_hip[2];
  float kd_knee[2];
  float tau_abad_ff[2];
  float tau_hip_ff[2];
  float tau_knee_ff[2];
  int32_t flags[2];
  int32_t checksum;

} spi_command_t;

// SPI Data Structure (Like MIT)
typedef struct {
    float q_abad[2];
    float q_hip[2];
    float q_knee[2];
    float qd_abad[2];
    float qd_hip[2];
    float qd_knee[2];
    int32_t flags[2];
    int32_t checksum;
} spi_data_t;

// SPI File Descriptor
int spi_1_fd = -1;
int spi_2_fd = -1;


/*!
 * Compute SPI message checksum
 * @param data : input
 * @param len : length (in 32-bit words)
 * @return
 */
uint32_t xor_checksum(uint32_t *data, size_t len) {
  uint32_t t = 0;  

  #ifdef DEBUG_PRINT_CHECKSUM_CALC
  int idx = 0;
  printf("-------- checksum calculation -----------");
  #endif

  for (size_t i = 0; i < len; i++) 
  {
    t = t ^ data[i];

    #ifdef DEBUG_PRINT_CHECKSUM_CALC
    printf("Step %d Data[%d] = 0x%08X, Current Checksum = 0x%08X\n", idx + 1, idx, data[idx], t);
    idx = idx + 1;
    #endif
  }

  #ifdef DEBUG_PRINT_CHECKSUM_CALC
  printf("Final computed checksum = 0x%08X\n",t);
  #endif

  return t;
}

/*!
 * Open SPI device
 */
int init_spi() {
  int rv = 0;
  spi_2_fd = open(SPI_DEVICE_2, O_RDWR);
  if (spi_2_fd < 0) perror("[ERROR] Couldn't open spidev 2.0");
  spi_1_fd = open(SPI_DEVICE_1, O_RDWR);
  if (spi_1_fd < 0) perror("[ERROR] Couldn't open spidev 2.1");

  rv = ioctl(spi_1_fd, SPI_IOC_WR_MODE, &spi_mode);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_mode (1)");

  rv = ioctl(spi_2_fd, SPI_IOC_WR_MODE, &spi_mode);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_mode (2)");

  rv = ioctl(spi_1_fd, SPI_IOC_RD_MODE, &spi_mode);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_mode (1)");

  rv = ioctl(spi_2_fd, SPI_IOC_RD_MODE, &spi_mode);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_mode (2)");

  rv = ioctl(spi_1_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_bits_per_word (1)");

  rv = ioctl(spi_2_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_bits_per_word (2)");

  rv = ioctl(spi_1_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bits_per_word);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_bits_per_word (1)");

  rv = ioctl(spi_2_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bits_per_word);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_bits_per_word (2)");

  rv = ioctl(spi_1_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_max_speed_hz (1)");
  rv = ioctl(spi_2_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_max_speed_hz (2)");

  rv = ioctl(spi_1_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_max_speed_hz (1)");
  rv = ioctl(spi_2_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_max_speed_hz (2)");

  rv = ioctl(spi_1_fd, SPI_IOC_RD_LSB_FIRST, &lsb);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_lsb_first (1)");

  rv = ioctl(spi_2_fd, SPI_IOC_RD_LSB_FIRST, &lsb);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_lsb_first (2)");
  return rv;
}

// Function to Swap Bytes (Fix Endianness)
void swap_bytes(uint16_t *buffer, size_t len) {
    for (size_t i = 0; i < len; i++) {
        buffer[i] = __bswap_16(buffer[i]);
    }
}

// SPI Send/Receive Function
void spi_send_receive(int spi_fd, spi_command_t *cmd, spi_data_t *data) {
    uint16_t tx_buf[K_WORDS_PER_MESSAGE] = {0};
    uint16_t rx_buf[K_WORDS_PER_MESSAGE] = {0};

    // Copy Command into tx_buf
    memcpy(tx_buf, cmd, sizeof(spi_command_t));

    // each word is two bytes long
    size_t word_len = 2;  // 16 bit word

    // Swap Bytes (Ensure Correct Order)
    swap_bytes(tx_buf, K_WORDS_PER_MESSAGE);

    // SPI Transfer Structure
    struct spi_ioc_transfer spi_message = {};
    spi_message.tx_buf = (unsigned long)tx_buf;
    spi_message.rx_buf = (unsigned long)rx_buf;
    spi_message.len = word_len * K_WORDS_PER_MESSAGE;
    spi_message.speed_hz = spi_speed;
    spi_message.bits_per_word = spi_bits_per_word;
    spi_message.cs_change = 1;

    // Perform SPI Transfer
    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_message) < 0) {
        perror("SPI Transfer Failed");
        return;
    }

    // Swap Received Bytes Back
    swap_bytes(rx_buf, K_WORDS_PER_MESSAGE);

    // Copy Data Back
    memcpy(data, rx_buf, sizeof(spi_data_t));

    // Debug Output
    // printf("Received Data: q_abad[0] = %.3f, q_hip[0] = %.3f, q_knee[0] = %.3f\n",
    //        data->q_abad[0], data->q_hip[0], data->q_knee[0]);
}

// Function to Clear Screen and Print Data in Two Columns for Each SPI Device
void print_data_dynamic(spi_data_t *data_1, spi_data_t *data_2) {
    printf("\033[H\033[J");  // Clear screen and move cursor to top-left
    printf("=================== SPI DATA RECEIVED ===================\n");

    // Print Data for SPI 1 (Two Columns)
    printf("\n[ SPI DEVICE 1: %s ]\n", SPI_DEVICE_1);
    printf("-------------------------------------------------------------\n");
    printf("| Parameter     |    Leg 0    |    Leg 1    |\n");
    printf("-------------------------------------------------------------\n");
    printf("| q_abad       | %10.6f | %10.6f |\n", data_1->q_abad[0], data_1->q_abad[1]);
    printf("| q_hip        | %10.6f | %10.6f |\n", data_1->q_hip[0], data_1->q_hip[1]);
    printf("| q_knee       | %10.6f | %10.6f |\n", data_1->q_knee[0], data_1->q_knee[1]);
    printf("| qd_abad      | %10.6f | %10.6f |\n", data_1->qd_abad[0], data_1->qd_abad[1]);
    printf("| qd_hip       | %10.6f | %10.6f |\n", data_1->qd_hip[0], data_1->qd_hip[1]);
    printf("| qd_knee      | %10.6f | %10.6f |\n", data_1->qd_knee[0], data_1->qd_knee[1]);
    printf("| flags        | %10u | %10u |\n", (uint32_t)data_1->flags[0], (uint32_t)data_1->flags[1]);
    printf("| Checksum     | %10u | Error Count : %u \n", (uint32_t)data_1->checksum, checksumErrCnt_dev1);
    printf("-------------------------------------------------------------\n");

    // Print Data for SPI 2 (Two Columns)
    printf("\n[ SPI DEVICE 2: %s ]\n", SPI_DEVICE_2);
    printf("-------------------------------------------------------------\n");
    printf("| Parameter     |    Leg 0    |    Leg 1    |\n");
    printf("-------------------------------------------------------------\n");
    printf("| q_abad       | %10.6f | %10.6f |\n", data_2->q_abad[0], data_2->q_abad[1]);
    printf("| q_hip        | %10.6f | %10.6f |\n", data_2->q_hip[0], data_2->q_hip[1]);
    printf("| q_knee       | %10.6f | %10.6f |\n", data_2->q_knee[0], data_2->q_knee[1]);
    printf("| qd_abad      | %10.6f | %10.6f |\n", data_2->qd_abad[0], data_2->qd_abad[1]);
    printf("| qd_hip       | %10.6f | %10.6f |\n", data_2->qd_hip[0], data_2->qd_hip[1]);
    printf("| qd_knee      | %10.6f | %10.6f |\n", data_2->qd_knee[0], data_2->qd_knee[1]);
    printf("| flags        | %10u | %10u |\n", (uint32_t)data_2->flags[0], (uint32_t)data_2->flags[1]);
    printf("| Checksum     | %10u | Error Count : %u \n", (uint32_t)data_2->checksum, checksumErrCnt_dev2);
    printf("-------------------------------------------------------------\n");

    printf("=============================================================\n");
}

// Callback function for subscriber
void messageCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received: %s", msg->data.c_str());
}

// // Callback Function for Joint Commands
// void trajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg) {
//     if (!msg->points.empty()) {
//         // Extract the first set of joint positions from trajectory command
//         std::vector<double> received_positions = msg->points[0].positions;

//         // Update SPI command structure for a single leg (Assuming FL leg for now)
//         cmd1.q_des_abad[0] = received_positions[0];
//         cmd1.q_des_hip[0] = received_positions[1];
//         cmd1.q_des_knee[0] = received_positions[2];

//         cmd1.q_des_abad[1] = received_positions[3];
//         cmd1.q_des_hip[1] = received_positions[4];
//         cmd1.q_des_knee[1] = received_positions[5];

//         // Other parameters can be updated if required (like stiffness, damping)
//         cmd1.kp_abad[0] = 10.0f;
//         cmd1.kp_hip[0] = 20.0f;
//         cmd1.kp_knee[0] = 30.0f;

//         cmd1.kd_abad[0] = 1.0f;
//         cmd1.kd_hip[0] = 1.0f;
//         cmd1.kd_knee[0] = 1.0f;

//         cmd1.checksum = 0;  // Placeholder checksum, update if needed

//         ROS_INFO("Updated SPI command structure from JointTrajectory.");
//     }
// }


int main(int argc, char** argv) {
    ros::init(argc, argv, "hw_interface_handler");
    ros::NodeHandle nh;

    // ROS Subscriber: Receive joint commands
    // ros::Subscriber traj_sub = nh.subscribe("/joint_group_position_controller/command", 10, trajectoryCallback);

    // ROS Publisher: Publish real joint states
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    ros::Rate rate(100);  // 100 Hz update rate


    init_spi();

    spi_command_t cmd1 = {};
    spi_command_t cmd2 = {};
    spi_data_t data_1 = {};
    spi_data_t data_2 = {};

    // Example Data (Can be replaced with actual inputs)
    cmd1.q_des_abad[0] = 0.1f;
    cmd1.q_des_hip[0] = -0.2f;
    cmd1.q_des_knee[1] = 0.3f;
    cmd1.tau_abad_ff[0] = 1.0f;
    cmd1.tau_hip_ff[0] = 2.0f;
    cmd1.tau_knee_ff[1] = 3.0f;
    cmd1.flags[0] = 1;
    cmd1.checksum = xor_checksum((uint32_t *)&cmd1, 32);

    // Example Data (Can be replaced with actual inputs)
    cmd2.q_des_abad[0] = 0.1f;
    cmd2.q_des_hip[1] = -0.2f;
    cmd2.qd_des_knee[0] = 0.3f;
    cmd2.kd_abad[0] = 3.f;
    cmd2.kd_abad[1] = 5.f;
    cmd2.tau_abad_ff[0] = 11.0f;
    cmd2.tau_hip_ff[1] = 2.124f;
    cmd2.tau_knee_ff[0] = -3.833f;
    cmd2.flags[0] = 1;
    cmd2.flags[1] = 1;
    cmd2.checksum = xor_checksum((uint32_t *)&cmd2, 32);



    // Create a subscriber to the "/test_topic"
    ros::Subscriber sub = nh.subscribe("/test_topic", 10, messageCallback);



    // Keep the node running
    ros::spin();


    // while (1) 
    // {
    //     // Read from both SPI devices
    //     for (int spi_board = 0; spi_board < 2; spi_board++) 
    //     {
    //         if(spi_board == 0 )
    //         {
    //             spi_send_receive(spi_1_fd, &cmd1, &data_1);
    //             uint32_t calc_checksum = xor_checksum((uint32_t *)&data_1, 14);
    //             if (calc_checksum != (uint32_t)data_1.checksum)
    //                 checksumErrCnt_dev1 = checksumErrCnt_dev1 + 1;
    //         }
    //         else
    //         {
    //             spi_send_receive(spi_2_fd, &cmd2, &data_2);
    //             uint32_t calc_checksum = xor_checksum((uint32_t *)&data_2, 14);
    //             if (calc_checksum != (uint32_t)data_2.checksum)
    //                 checksumErrCnt_dev2 = checksumErrCnt_dev2 + 1;
    //         }
    //     }

    //     // Print Data Dynamically for Both Devices
    //     print_data_dynamic(&data_1, &data_2);

    //     usleep(500000);  // 500ms delay
    // }


    close(spi_1_fd);
    close(spi_2_fd);
    return 0;
}
