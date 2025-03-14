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


#define SUB_FROM_ROS 0
#define PUB_TO_ROS   1
volatile int flowcontrol = SUB_FROM_ROS;
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


spi_command_t spi_command_1 = {};
spi_command_t spi_command_2 = {};
spi_data_t spi_data_1 = {};
spi_data_t spi_data_2 = {};

// SPI File Descriptor
int spi_1_fd = -1;
int spi_2_fd = -1;



// For logging
static char log_timestamp[15] = "";
static void init_log_timestamp(void)
{
    if(log_timestamp[0] == '\0')
    {
        time_t now = time(NULL);
        struct tm *t = localtime(&now);
        strftime(log_timestamp, sizeof(log_timestamp), "%Y%m%d%H%M", t);
    }
}

#ifdef DEBUG_PRINT_SPINE_COMMAND
//Printing Spine command
void printSpineCmd(spi_command_t *print_cmd) {
    printf("====================== Debugging ROS Command ======================\n");
    for (int i = 0; i < 2; i++) {
        printf("Leg Index: %d\n", i);
        printf("  q_des_abad  : %.9g\n", print_cmd->q_des_abad[i]);
        printf("  q_des_hip   : %.9g\n", print_cmd->q_des_hip[i]);
        printf("  q_des_knee  : %.9g\n", print_cmd->q_des_knee[i]);
        printf("  qd_des_abad : %.9g\n", print_cmd->qd_des_abad[i]);
        printf("  qd_des_hip  : %.9g\n", print_cmd->qd_des_hip[i]);
        printf("  qd_des_knee : %.9g\n", print_cmd->qd_des_knee[i]);
        printf("  kp_abad     : %.9g\n", print_cmd->kp_abad[i]);
        printf("  kp_hip      : %.9g\n", print_cmd->kp_hip[i]);
        printf("  kp_knee     : %.9g\n", print_cmd->kp_knee[i]);
        printf("  kd_abad     : %.9g\n", print_cmd->kd_abad[i]);
        printf("  kd_hip      : %.9g\n", print_cmd->kd_hip[i]);
        printf("  kd_knee     : %.9g\n", print_cmd->kd_knee[i]);
        printf("  tau_abad_ff : %.9g\n", print_cmd->tau_abad_ff[i]);
        printf("  tau_hip_ff  : %.9g\n", print_cmd->tau_hip_ff[i]);
        printf("  tau_knee_ff : %.9g\n", print_cmd->tau_knee_ff[i]);
        printf("  flags       : %d\n", print_cmd->flags[i]);
        printf("--------------------------------------------------------------\n");
    }
    printf("  checksum    : %d\n", print_cmd->checksum);
    printf("===============================================================\n");
}
#endif

                                                  ///////////////////////////////
                                                  //           S P I 
                                                  ///////////////////////////////
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


/*!
 * Prepare the command for transmission like checkum calculation
 */
void prepare_command_for_tx(spi_command_t *spi_tx_cmd, int leg_0) 
{

  // Init timestamp
  init_log_timestamp();
  char commandlog_file_name[128];
  sprintf(commandlog_file_name,"spi_command_log_%s.csv",log_timestamp);
  // Open the CSV file in append mode
  FILE *file = fopen(commandlog_file_name, "a");
  if (!file) {
      perror("Failed to open file");
      return;

  }

  static int header_written = 0; // Ensure header is written only once
  if (!header_written) {
      fprintf(file, "Leg_Index, q_des_abad, q_des_hip, q_des_knee, qd_des_abad, qd_des_hip, qd_des_knee, "
                    "kp_abad, kp_hip, kp_knee, kd_abad, kd_hip, kd_knee, tau_abad_ff, tau_hip_ff, tau_knee_ff, Checksum, Flags\n");
      header_written = 1;
  }

  spi_tx_cmd->qd_des_abad[0]  = DEFAULT_V;
  spi_tx_cmd->qd_des_hip[0]   = DEFAULT_V;
  spi_tx_cmd->qd_des_knee[0]  = DEFAULT_V;

  spi_tx_cmd->kp_abad[0]      = DEFAULT_KP;
  spi_tx_cmd->kp_hip[0]       = DEFAULT_KP;
  spi_tx_cmd->kp_knee[0]      = DEFAULT_KP;

  spi_tx_cmd->kd_abad[0]      = DEFAULT_KD;
  spi_tx_cmd->kd_hip[0]       = DEFAULT_KD;
  spi_tx_cmd->kd_knee[0]      = DEFAULT_KD;

  spi_tx_cmd->tau_abad_ff[0]  = DEFAULT_TRQ;
  spi_tx_cmd->tau_hip_ff[0]   = DEFAULT_TRQ;
  spi_tx_cmd->tau_knee_ff[0]  = DEFAULT_TRQ;

  spi_tx_cmd->qd_des_abad[1]  = DEFAULT_V;
  spi_tx_cmd->qd_des_hip[1]   = DEFAULT_V;
  spi_tx_cmd->qd_des_knee[1]  = DEFAULT_V;

  spi_tx_cmd->kp_abad[1]      = DEFAULT_KP;
  spi_tx_cmd->kp_hip[1]       = DEFAULT_KP;
  spi_tx_cmd->kp_knee[1]      = DEFAULT_KP;

  spi_tx_cmd->kd_abad[1]      = DEFAULT_KD;
  spi_tx_cmd->kd_hip[1]       = DEFAULT_KD;
  spi_tx_cmd->kd_knee[1]      = DEFAULT_KD;

  spi_tx_cmd->tau_abad_ff[1]  = DEFAULT_TRQ;
  spi_tx_cmd->tau_hip_ff[1]   = DEFAULT_TRQ;
  spi_tx_cmd->tau_knee_ff[1]  = DEFAULT_TRQ;


  //Calculate checksum
  spi_tx_cmd->checksum = xor_checksum((uint32_t *)spi_tx_cmd, 32);
  
  #ifdef DEBUG_PRINT_SPINE_COMMAND
  printSpineCmd(spi_tx_cmd);
  #endif

  // Adding log
  for (int i = 0; i < 2; i++) {  
    fprintf(file, "%d, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %d, %d\n",
            leg_0 + i,
            spi_tx_cmd->q_des_abad[i], spi_tx_cmd->q_des_hip[i], spi_tx_cmd->q_des_knee[i],
            spi_tx_cmd->qd_des_abad[i], spi_tx_cmd->qd_des_hip[i], spi_tx_cmd->qd_des_knee[i],
            spi_tx_cmd->kp_abad[i], spi_tx_cmd->kp_hip[i], spi_tx_cmd->kp_knee[i],
            spi_tx_cmd->kd_abad[i], spi_tx_cmd->kd_hip[i], spi_tx_cmd->kd_knee[i],
            spi_tx_cmd->tau_abad_ff[i], spi_tx_cmd->tau_hip_ff[i], spi_tx_cmd->tau_knee_ff[i],
            spi_tx_cmd->checksum, spi_tx_cmd->flags[i]);    
   }

    // Close the file
    fclose(file);
}

/*!
 * Process the received data like for checksum check, logging
 */
void process_data_after_rx(spi_data_t *spi_rx_data, int leg_0) {

  // Init timestamp
  init_log_timestamp();
  char datalog_file_name[128];
  sprintf(datalog_file_name,"spi_data_log_%s.csv",log_timestamp);
  // Open the CSV file in append mode
  FILE *file = fopen(datalog_file_name, "a");
  if (!file) {
      perror("Failed to open file");
      return;

  }
  // Write the header row if the file is empty (optional, depending on your use case)

  static int header_data_written = 0; // Ensure header is written only once
  if (!header_data_written) {
      fprintf(file, "Leg_Index, q_abad, q_hip, q_knee, qd_abad, qd_hip, qd_knee, tau_m_abad, tau_m_hip, tau_m_knee, CalcChecksum, RcvdChecksum, Flag_dataraw, Flag_data\n");
      header_data_written = 1;
  }

  uint32_t calc_checksum = xor_checksum((uint32_t *)spi_rx_data, 14);
  if (calc_checksum != (uint32_t)spi_rx_data->checksum)
  {
    printf("SPI ERROR BAD CHECKSUM in Leg %d : Calculated 0x%08X  Received 0x%08X \n", leg_0, calc_checksum,(uint32_t)spi_rx_data->checksum);
    checksumErrCnt_dev[leg_0/2] = checksumErrCnt_dev[leg_0/2] + 1; // leg_0/2 = 0 when leg_0 is 0 ie for dev1, and = 1 when leg_0 is 2 for dev2
  }

  // Log the data
  for (int i = 0; i < 2; i++) {

    // Extract each 10-bit segment
    uint32_t flag = (spi_rx_data->flags[i]) & 0x3; // First 2 bits
    uint16_t a_enc = (spi_rx_data->flags[i] >> 2) & 0x3FF; // First 10 bits (bits 2–11) 
    uint16_t b_enc = (spi_rx_data->flags[i] >> 12) & 0x3FF; // Next 10 bits (bits 12–21) 
    uint16_t c_enc = (spi_rx_data->flags[i] >> 22) & 0x3FF; // Last 10 bits (bits 22–31) 
    // Decode each value back to the original 
    float tau_m_abad = (a_enc * (140.0 / 1023.0)) - 70; 
    float tau_m_hip = (b_enc * (140.0 / 1023.0)) - 70; 
    float tau_m_knee = (c_enc * (140.0 / 1023.0)) - 70; 


    fprintf(file, "%d, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %d, %d, %d, %d\n",
            leg_0 + i,
            spi_rx_data->q_abad[i], spi_rx_data->q_hip[i], spi_rx_data->q_knee[i],
            spi_rx_data->qd_abad[i], spi_rx_data->qd_hip[i], spi_rx_data->qd_knee[i],
            tau_m_abad, tau_m_hip, tau_m_knee,
            calc_checksum, (uint32_t)spi_rx_data->checksum,
            spi_rx_data->flags[i], flag);

  }

  // Close the file
  fclose(file);
}
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

    ///////////////////////////////////////////////////
    //   Define ROS nodes, subscribers, publishers
    ///////////////////////////////////////////////////
    ros::init(argc, argv, "hw_interface_handler");
    ros::NodeHandle nh;

    // ROS Subscriber: Receive joint commands
    ros::Subscriber traj_sub = nh.subscribe("/joint_group_position_controller/command", 10, trajectoryCallback);

    // ROS Publisher: Publish real joint states
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    ros::Rate rate(100);  // 100 Hz update rate


    //////////////////////////////////////////
    //     INIT SPI and DEFINE STRUCTURES
    //////////////////////////////////////////
    init_spi();
    if (spi_1_fd < 0 || spi_2_fd < 0) 
    {
        perror("[ERROR] Couldn't open SPI device");
        return -1;
    }


    while (ros::ok()) 
    { 
        if(flowcontrol == PUB_TO_ROS)
        {
          // Read from both SPI devices
          for (int spi_board = 0; spi_board < 2; spi_board++) 
          {
              if(spi_board == 0 )
              {
                  prepare_command_for_tx(&spi_command_1, spi_board * 2);
                  spi_send_receive(spi_1_fd, &spi_command_1, &spi_data_1);
                  process_data_after_rx(&spi_data_1, spi_board * 2);
              }
              else
              {
                  prepare_command_for_tx(&spi_command_2, spi_board * 2);
                  spi_send_receive(spi_2_fd, &spi_command_2, &spi_data_2);
                  process_data_after_rx(&spi_data_2, spi_board * 2);
              }
          }

          // // Print Data Dynamically for Both Devices
          // print_data_dynamic(&spi_data_1, &spi_data_2);

    //     usleep(500000);  // 500ms delay
    // }

          flowcontrol = SUB_FROM_ROS;
        }

        ros::spinOnce();
        rate.sleep();
    }


    close(spi_1_fd);
    close(spi_2_fd);
    return 0;
}
