/*!
 * @file rt_spi.h
 * @brief SPI communication to spine board
 */
#ifdef linux


// #define DEBUG_PRINT_SPI_DATA_RCVD
#define DEBUG_PRINT_SPI_DATA_TRANSMIT
#define DEBUG_PRINT_CHECKSUM_CALC
#define DEBUG_PRINT_SPINE_COMMAND
// #define DEBUG_STUB_COMMAND_HARDCODE
// #define DEBUG_PAUSE_AFTER_EACHLEG_SPI

#include <byteswap.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <linux/spi/spidev.h>
#include "rt/rt_spi.h"
#include <lcm/lcm-cpp.hpp>

unsigned char spi_mode = SPI_MODE_0;
unsigned char spi_bits_per_word = 8;
unsigned int spi_speed = 6000000;   // MIT original speed
//unsigned int spi_speed = 600000;  // Trail
//unsigned int spi_speed = 20000000;  // Dave Arduino speed with the initialisation code = 20MHz
//unsigned int spi_speed = 8000000;  // Dave Arduino speed with clockdivider = 8MHz

uint8_t lsb = 0x01;
//uint8_t lsb = 0x00;

int spi_1_fd = -1;
int spi_2_fd = -1;

int spi_open();

static spine_cmd_t g_spine_cmd;
static spine_data_t g_spine_data;

spi_command_t spi_command_drv;
spi_data_t spi_data_drv;
spi_torque_t spi_torque;

pthread_mutex_t spi_mutex;

const float max_torque[3] = {17.f, 17.f, 26.f};  // TODO CHECK WITH BEN
const float wimp_torque[3] = {6.f, 6.f, 6.f};    // TODO CHECK WITH BEN
const float disabled_torque[3] = {0.f, 0.f, 0.f};

// only used for actual robot
// const float abad_side_sign[4] = {-1.f, -1.f, 1.f, 1.f};
// const float hip_side_sign[4] = {-1.f, 1.f, -1.f, 1.f};

// // const float knee_side_sign[4] = {-.6429f, .6429f, -.6429f, .6429f};
// const float knee_side_sign[4] = {-1.f, 1.f, -1.f, 1.f};


// only used for actual robot
// const float abad_offset[4] = {0.f, 0.f, 0.f, 0.f};
// const float hip_offset[4] = {M_PI / 2.f, -M_PI / 2.f, -M_PI / 2.f, M_PI / 2.f};
// const float knee_offset[4] = {K_KNEE_OFFSET_POS, -K_KNEE_OFFSET_POS,
//                               -K_KNEE_OFFSET_POS, K_KNEE_OFFSET_POS};




// SCOTTY Offset and Sign

// const float abad_offset[4] = {0.f, 0.f, 0.f, 0.f};
const float abad_offset[4] = {0.42f, 0.f, 0.f, 0.f};
const float hip_offset[4] = {0.f, 0.f, 0.f, 0.f};
const float knee_offset[4] = {0.f, 0.f, 0.f, 0.f};

const float abad_side_sign[4] = {-1.f, 1.f, 1.f, -1.f};
const float hip_side_sign[4] = {1.f, 1.f, -1.f, 1.f}; 
const float knee_side_sign[4] = {1.f, 1.f, -1.f, 1.f};


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
void printSpineCmd(spine_cmd_t *spine_cmd) {
    printf("====================== Debugging spine_cmd ======================\n");
    for (int i = 0; i < 2; i++) {
        printf("Leg Index: %d\n", i);
        printf("  q_des_abad  : %.9g\n", spine_cmd->q_des_abad[i]);
        printf("  q_des_hip   : %.9g\n", spine_cmd->q_des_hip[i]);
        printf("  q_des_knee  : %.9g\n", spine_cmd->q_des_knee[i]);
        printf("  qd_des_abad : %.9g\n", spine_cmd->qd_des_abad[i]);
        printf("  qd_des_hip  : %.9g\n", spine_cmd->qd_des_hip[i]);
        printf("  qd_des_knee : %.9g\n", spine_cmd->qd_des_knee[i]);
        printf("  kp_abad     : %.9g\n", spine_cmd->kp_abad[i]);
        printf("  kp_hip      : %.9g\n", spine_cmd->kp_hip[i]);
        printf("  kp_knee     : %.9g\n", spine_cmd->kp_knee[i]);
        printf("  kd_abad     : %.9g\n", spine_cmd->kd_abad[i]);
        printf("  kd_hip      : %.9g\n", spine_cmd->kd_hip[i]);
        printf("  kd_knee     : %.9g\n", spine_cmd->kd_knee[i]);
        printf("  tau_abad_ff : %.9g\n", spine_cmd->tau_abad_ff[i]);
        printf("  tau_hip_ff  : %.9g\n", spine_cmd->tau_hip_ff[i]);
        printf("  tau_knee_ff : %.9g\n", spine_cmd->tau_knee_ff[i]);
        printf("  flags       : %d\n", spine_cmd->flags[i]);
        printf("--------------------------------------------------------------\n");
    }
    printf("  checksum    : %d\n", spine_cmd->checksum);
    printf("===============================================================\n");
}
#endif

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
 * Emulate the spi board to estimate the torque.
 */
void fake_spine_control(spi_command_t *cmd, spi_data_t *data,
                        spi_torque_t *torque_out, int board_num) {
  torque_out->tau_abad[board_num] =
      cmd->kp_abad[board_num] *
          (cmd->q_des_abad[board_num] - data->q_abad[board_num]) +
      cmd->kd_abad[board_num] *
          (cmd->qd_des_abad[board_num] - data->qd_abad[board_num]) +
      cmd->tau_abad_ff[board_num];

  torque_out->tau_hip[board_num] =
      cmd->kp_hip[board_num] *
          (cmd->q_des_hip[board_num] - data->q_hip[board_num]) +
      cmd->kd_hip[board_num] *
          (cmd->qd_des_hip[board_num] - data->qd_hip[board_num]) +
      cmd->tau_hip_ff[board_num];

  torque_out->tau_knee[board_num] =
      cmd->kp_knee[board_num] *
          (cmd->q_des_knee[board_num] - data->q_knee[board_num]) +
      cmd->kd_knee[board_num] *
          (cmd->qd_des_knee[board_num] - data->qd_knee[board_num]) +
      cmd->tau_knee_ff[board_num];

  const float *torque_limits = disabled_torque;

  if (cmd->flags[board_num] & 0b1) {
    if (cmd->flags[board_num] & 0b10)
      torque_limits = wimp_torque;
    else
      torque_limits = max_torque;
  }

  if (torque_out->tau_abad[board_num] > torque_limits[0])
    torque_out->tau_abad[board_num] = torque_limits[0];
  if (torque_out->tau_abad[board_num] < -torque_limits[0])
    torque_out->tau_abad[board_num] = -torque_limits[0];

  if (torque_out->tau_hip[board_num] > torque_limits[1])
    torque_out->tau_hip[board_num] = torque_limits[1];
  if (torque_out->tau_hip[board_num] < -torque_limits[1])
    torque_out->tau_hip[board_num] = -torque_limits[1];

  if (torque_out->tau_knee[board_num] > torque_limits[2])
    torque_out->tau_knee[board_num] = torque_limits[2];
  if (torque_out->tau_knee[board_num] < -torque_limits[2])
    torque_out->tau_knee[board_num] = -torque_limits[2];
}

/*!
 * Initialize SPI
 */
void init_spi() {
  // check sizes:
  size_t command_size = sizeof(spi_command_t);
  size_t data_size = sizeof(spi_data_t);

  memset(&spi_command_drv, 0, sizeof(spi_command_drv));
  memset(&spi_data_drv, 0, sizeof(spi_data_drv));

  if (pthread_mutex_init(&spi_mutex, NULL) != 0)
    printf("[ERROR: RT SPI] Failed to create spi data mutex\n");

  if (command_size != K_EXPECTED_COMMAND_SIZE) {
    printf("[RT SPI] Error command size is %ld, expected %d\n", command_size,
           K_EXPECTED_COMMAND_SIZE);
  } else
    printf("[RT SPI] command size good\n");

  if (data_size != K_EXPECTED_DATA_SIZE) {
    printf("[RT SPI] Error data size is %ld, expected %d\n", data_size,
           K_EXPECTED_DATA_SIZE);
  } else
    printf("[RT SPI] data size good\n");

  printf("[RT SPI] Open\n");
  spi_open();
}

/*!
 * Open SPI device
 */
int spi_open() {
  int rv = 0;
  spi_2_fd = open("/dev/spidev2.0", O_RDWR);
  if (spi_2_fd < 0) perror("[ERROR] Couldn't open spidev 2.0");
  spi_1_fd = open("/dev/spidev2.1", O_RDWR);
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

int spi_driver_iterations = 0;

/*!
 * convert spi command to spine_cmd_t
 */
void spi_to_spine(spi_command_t *cmd, spine_cmd_t *spine_cmd, int leg_0, int32_t currentControlMode) {

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
  // Write the header row if the file is empty (optional, depending on your use case)

  static int header_written = 0; // Ensure header is written only once
  if (!header_written) {
      fprintf(file, "Leg_Index, q_des_abad, q_des_hip, q_des_knee, qd_des_abad, qd_des_hip, qd_des_knee, "
                    "kp_abad, kp_hip, kp_knee, kd_abad, kd_hip, kd_knee, tau_abad_ff, tau_hip_ff, tau_knee_ff, Control Mode, Checksum, Flags_raw, Flags\n");
      header_written = 1;
  }


  for (int i = 0; i < 2; i++) {
    // spine_cmd->q_des_abad[i] = (cmd->q_des_abad[i+leg_0] +
    // abad_offset[i+leg_0]) * abad_side_sign[i+leg_0]; spine_cmd->q_des_hip[i]
    // = (cmd->q_des_hip[i+leg_0] + hip_offset[i+leg_0]) *
    // hip_side_sign[i+leg_0]; spine_cmd->q_des_knee[i] =
    // (cmd->q_des_knee[i+leg_0] + knee_offset[i+leg_0]) /
    // knee_side_sign[i+leg_0];
    spine_cmd->q_des_abad[i] =
        (cmd->q_des_abad[i + leg_0] * abad_side_sign[i + leg_0]) +
        abad_offset[i + leg_0];
    spine_cmd->q_des_hip[i] =
        (cmd->q_des_hip[i + leg_0] * hip_side_sign[i + leg_0]) +
        hip_offset[i + leg_0];
    spine_cmd->q_des_knee[i] =
        (cmd->q_des_knee[i + leg_0] / knee_side_sign[i + leg_0]) +
        knee_offset[i + leg_0];

    spine_cmd->qd_des_abad[i] =
        cmd->qd_des_abad[i + leg_0] * abad_side_sign[i + leg_0];
    spine_cmd->qd_des_hip[i] =
        cmd->qd_des_hip[i + leg_0] * hip_side_sign[i + leg_0];
    spine_cmd->qd_des_knee[i] =
        cmd->qd_des_knee[i + leg_0] / knee_side_sign[i + leg_0];

    spine_cmd->kp_abad[i] = cmd->kp_abad[i + leg_0];
    spine_cmd->kp_hip[i] = cmd->kp_hip[i + leg_0];
    spine_cmd->kp_knee[i] = cmd->kp_knee[i + leg_0];

    spine_cmd->kd_abad[i] = cmd->kd_abad[i + leg_0];
    spine_cmd->kd_hip[i] = cmd->kd_hip[i + leg_0];
    spine_cmd->kd_knee[i] = cmd->kd_knee[i + leg_0];

    spine_cmd->tau_abad_ff[i] =
        cmd->tau_abad_ff[i + leg_0] * abad_side_sign[i + leg_0];
    spine_cmd->tau_hip_ff[i] =
        cmd->tau_hip_ff[i + leg_0] * hip_side_sign[i + leg_0];
    spine_cmd->tau_knee_ff[i] =
        cmd->tau_knee_ff[i + leg_0] * knee_side_sign[i + leg_0];

    spine_cmd->flags[i] = cmd->flags[i + leg_0];
    //Adding control_mode to the higher 16 bits of the flag
    spine_cmd->flags[i] = (spine_cmd->flags[i] & 0x0000FFFF) | (currentControlMode & 0xFFFF)<<16;
  
   }


  #ifdef DEBUG_STUB_COMMAND_HARDCODE
  //testing value
  spine_cmd->q_des_abad[0] = -0.110023;
  spine_cmd->q_des_hip[0] = 0.005643;
  spine_cmd->q_des_knee[0] = -0.699701;

  spine_cmd->qd_des_abad[0] = -0.000000;
  spine_cmd->qd_des_hip[0] = 0.000000;
  spine_cmd->qd_des_knee[0] = 0.000000;

  spine_cmd->kp_abad[0] = 5.000000 ;
  spine_cmd->kp_hip[0] = 5.000000 ;
  spine_cmd->kp_knee[0] =  5.000000;

  spine_cmd->kd_abad[0] =  0.100000;
  spine_cmd->kd_hip[0] =  0.100000;
  spine_cmd->kd_knee[0] =  0.100000;

  spine_cmd->tau_abad_ff[0] =-0.000000;
  spine_cmd->tau_hip_ff[0] =0.000000;
  spine_cmd->tau_knee_ff[0] = 0.000000;

  spine_cmd->flags[0] = 1 ;


  spine_cmd->q_des_abad[1] = 0.293105;
  spine_cmd->q_des_hip[1] = -0.045398;
  spine_cmd->q_des_knee[1] = -1.019152;

  spine_cmd->qd_des_abad[1] = 0.000000;
  spine_cmd->qd_des_hip[1] = 0.000000;
  spine_cmd->qd_des_knee[1] = 0.000000;

  spine_cmd->kp_abad[1] = 5.000000 ;
  spine_cmd->kp_hip[1] =  5.000000;
  spine_cmd->kp_knee[1] = 5.000000 ;

  spine_cmd->kd_abad[1] = 0.100000 ;
  spine_cmd->kd_hip[1] = 0.100000 ;
  spine_cmd->kd_knee[1] = 0.100000 ;

  spine_cmd->tau_abad_ff[1] = 0.000000;
  spine_cmd->tau_hip_ff[1] = 0.000000;
  spine_cmd->tau_knee_ff[1] = 0.000000;

  spine_cmd->flags[1] = 1;
  #endif


  spine_cmd->checksum = xor_checksum((uint32_t *)spine_cmd, 32);
  
  #ifdef DEBUG_PRINT_SPINE_COMMAND
  printSpineCmd(spine_cmd);
  #endif

  // Adding log
  for (int i = 0; i < 2; i++) {  
    fprintf(file, "%d, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %d, %d, %d, %d\n",
            leg_0 + i,
            spine_cmd->q_des_abad[i], spine_cmd->q_des_hip[i], spine_cmd->q_des_knee[i],
            spine_cmd->qd_des_abad[i], spine_cmd->qd_des_hip[i], spine_cmd->qd_des_knee[i],
            spine_cmd->kp_abad[i], spine_cmd->kp_hip[i], spine_cmd->kp_knee[i],
            spine_cmd->kd_abad[i], spine_cmd->kd_hip[i], spine_cmd->kd_knee[i],
            spine_cmd->tau_abad_ff[i], spine_cmd->tau_hip_ff[i], spine_cmd->tau_knee_ff[i],
            currentControlMode, spine_cmd->checksum, spine_cmd->flags[i], 
            cmd->flags[i + leg_0]); // control mode, flag(not using spine_cmd as it is already embedded with control mode)
    
   }

    // Close the file
    fclose(file);
}

/*!
 * convert spine_data_t to spi data
 */
void spine_to_spi(spi_data_t *data, spine_data_t *spine_data, int leg_0) {

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


  for (int i = 0; i < 2; i++) {
    data->q_abad[i + leg_0] = (spine_data->q_abad[i] - abad_offset[i + leg_0]) *
                              abad_side_sign[i + leg_0];
    data->q_hip[i + leg_0] = (spine_data->q_hip[i] - hip_offset[i + leg_0]) *
                             hip_side_sign[i + leg_0];
    data->q_knee[i + leg_0] = (spine_data->q_knee[i] - knee_offset[i + leg_0]) *
                              knee_side_sign[i + leg_0];

    data->qd_abad[i + leg_0] =
        spine_data->qd_abad[i] * abad_side_sign[i + leg_0];
    data->qd_hip[i + leg_0] = spine_data->qd_hip[i] * hip_side_sign[i + leg_0];
    data->qd_knee[i + leg_0] =
        spine_data->qd_knee[i] * knee_side_sign[i + leg_0];

    data->flags[i + leg_0] = spine_data->flags[i] & 0x03;
  }

  uint32_t calc_checksum = xor_checksum((uint32_t *)spine_data, 14);
  if (calc_checksum != (uint32_t)spine_data->checksum)
    printf("SPI ERROR BAD CHECKSUM in Leg %d : Calculated 0x%08X  Received 0x%08X \n", leg_0, calc_checksum,(uint32_t)spine_data->checksum);

  // Log the data
  for (int i = 0; i < 2; i++) {

    // Extract each 10-bit segment
    uint16_t a_enc = (spine_data->flags[i] >> 2) & 0x3FF; // First 10 bits (bits 2–11) 
    uint16_t b_enc = (spine_data->flags[i] >> 12) & 0x3FF; // Next 10 bits (bits 12–21) 
    uint16_t c_enc = (spine_data->flags[i] >> 22) & 0x3FF; // Last 10 bits (bits 22–31) 
    // Decode each value back to the original 
    float tau_m_abad = (a_enc * (140.0 / 1023.0)) - 70; 
    float tau_m_hip = (b_enc * (140.0 / 1023.0)) - 70; 
    float tau_m_knee = (c_enc * (140.0 / 1023.0)) - 70; 


    fprintf(file, "%d, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %d, %d, %d, %d\n",
            leg_0 + i,
            spine_data->q_abad[i], spine_data->q_hip[i], spine_data->q_knee[i],
            spine_data->qd_abad[i], spine_data->qd_hip[i], spine_data->qd_knee[i],
            tau_m_abad, tau_m_hip, tau_m_knee,
            calc_checksum, (uint32_t)spine_data->checksum,
            spine_data->flags[i], data->flags[i + leg_0]);

  }

  // Close the file
  fclose(file);
}

/*!
 * send receive data and command from spine
 */
void spi_send_receive(spi_command_t *command, spi_data_t *data, int32_t currentControlMode) {
  // update driver status flag
  spi_driver_iterations++;
  data->spi_driver_status = spi_driver_iterations << 16;

  // transmit and receive buffers
  uint16_t tx_buf[K_WORDS_PER_MESSAGE];
  uint16_t rx_buf[K_WORDS_PER_MESSAGE];

  for (int spi_board = 0; spi_board < 2; spi_board++) {


    #ifdef DEBUG_PAUSE_AFTER_EACHLEG_SPI
    printf("Press a key");
    getchar();   
    #endif

    // copy command into spine type:
    spi_to_spine(command, &g_spine_cmd, spi_board * 2, currentControlMode);

    // pointers to command/data spine array
    uint16_t *cmd_d = (uint16_t *)&g_spine_cmd;
    uint16_t *data_d = (uint16_t *)&g_spine_data;

    // zero rx buffer
    memset(rx_buf, 0, K_WORDS_PER_MESSAGE * sizeof(uint16_t));

    // copy into tx buffer flipping bytes
    for (int i = 0; i < K_WORDS_PER_MESSAGE; i++)
      tx_buf[i] = (cmd_d[i] >> 8) + ((cmd_d[i] & 0xff) << 8);
    // tx_buf[i] = __bswap_16(cmd_d[i]);

    // each word is two bytes long
    size_t word_len = 2;  // 16 bit word

    // spi message struct
    struct spi_ioc_transfer spi_message[1];

    // zero message struct.
    memset(spi_message, 0, 1 * sizeof(struct spi_ioc_transfer));

    // set up message struct
    for (int i = 0; i < 1; i++) {
      spi_message[i].bits_per_word = spi_bits_per_word;
      //spi_message[i].cs_change = 0;
      spi_message[i].cs_change = 1;    //original
      spi_message[i].delay_usecs = 0;
      spi_message[i].len = word_len * 66;
      spi_message[i].rx_buf = (uint64_t)rx_buf; //original
      spi_message[i].tx_buf = (uint64_t)tx_buf; //original
    // spi_message[i].rx_buf = (uint32_t)rx_buf; //Dave matched
    // spi_message[i].tx_buf = (uint32_t)tx_buf; //Dave matched
    }

    // do spi communication
    int rv = ioctl(spi_board == 0 ? spi_1_fd : spi_2_fd, SPI_IOC_MESSAGE(1),
                  &spi_message);



    #ifdef DEBUG_PRINT_SPI_DATA_TRANSMIT
        // Print the raw data 
        printf("\n=== SPI BOARD %d RAW TX DATA ===\n", spi_board); 
        for(int i = 0; i < K_WORDS_PER_MESSAGE; i++) 
        { 
          printf("tx_buf[%02d] = 0x%08X\n", i, tx_buf[i]);
        }
        printf("===============================\n\n");       
    #endif

    #ifdef DEBUG_PRINT_SPI_DATA_RCVD
      if (rv < 0) { perror("[ERROR] ioctl (SPI) failed"); } 
      else { 
        // Print the raw data 
        printf("\n=== SPI BOARD %d RAW RX DATA ===\n", spi_board); 
        for(int i = 0; i < K_WORDS_PER_MESSAGE; i++) 
        { 
          printf("rx_buf[%02d] = 0x%04X\n", i, rx_buf[i]);
        }
        printf("===============================\n\n"); 
      }
    #else         
      (void)rv;
    #endif

    //usleep(10);

    // flip bytes the other way
    for (int i = 0; i < 30; i++)
      data_d[i] = (rx_buf[i] >> 8) + ((rx_buf[i] & 0xff) << 8);
    // data_d[i] = __bswap_16(rx_buf[i]);

    // copy back to data
    spine_to_spi(data, &g_spine_data, spi_board * 2);
    //spine_to_spi(data, &g_spine_data, spi_board * 1);
  }
}

/*!
 * Run SPI
 */
void spi_driver_run(int32_t currentControlMode) {
  // do spi board calculations
  for (int i = 0; i < 4; i++) {
    fake_spine_control(&spi_command_drv, &spi_data_drv, &spi_torque, i);
  }

  // in here, the driver is good
  pthread_mutex_lock(&spi_mutex);
  spi_send_receive(&spi_command_drv, &spi_data_drv, currentControlMode);
  pthread_mutex_unlock(&spi_mutex);
}

/*!
 * Get the spi command
 */
spi_command_t *get_spi_command() {
  return &spi_command_drv;
}

/*!
 * Get the spi data
 */
spi_data_t *get_spi_data() { return &spi_data_drv; }

#endif
