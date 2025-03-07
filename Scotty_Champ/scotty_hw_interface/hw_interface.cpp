#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <string.h>
#include <byteswap.h>

// Define SPI Device
#define SPI_DEVICE "/dev/spidev2.0"
#define K_WORDS_PER_MESSAGE 66
#define SPI_MODE SPI_MODE_0
#define SPI_BITS 8
#define SPI_SPEED 6000000  // 6 MHz



unsigned char spi_mode = SPI_MODE_0;
unsigned char spi_bits_per_word = 8;
unsigned int spi_speed = 6000000;   // MIT original speed
uint8_t lsb = 0x01;


// SPI Command Structure (Like MIT)
typedef struct {
    float q_des_abad[2];
    float q_des_hip[2];
    float q_des_knee[2];
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
 * Open SPI device
 */
int init_spi() {
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

// Function to Swap Bytes (Fix Endianness)
void swap_bytes(uint16_t *buffer, size_t len) {
    for (size_t i = 0; i < len; i++) {
        buffer[i] = __bswap_16(buffer[i]);
    }
}

// SPI Send/Receive Function
void spi_send_receive(spi_command_t *cmd, spi_data_t *data) {
    uint16_t tx_buf[K_WORDS_PER_MESSAGE] = {0};
    uint16_t rx_buf[K_WORDS_PER_MESSAGE] = {0};

    // Copy Command into tx_buf
    memcpy(tx_buf, cmd, sizeof(spi_command_t));

    // Swap Bytes (Ensure Correct Order)
    swap_bytes(tx_buf, K_WORDS_PER_MESSAGE);

    // SPI Transfer Structure
    struct spi_ioc_transfer spi_message = {};
    spi_message.tx_buf = (unsigned long)tx_buf;
    spi_message.rx_buf = (unsigned long)rx_buf;
    spi_message.len = sizeof(tx_buf);
    spi_message.speed_hz = SPI_SPEED;
    spi_message.bits_per_word = SPI_BITS;

    // Perform SPI Transfer
    if (ioctl(spi_1_fd, SPI_IOC_MESSAGE(1), &spi_message) < 0) {
        perror("SPI Transfer Failed");
        return;
    }

    // Swap Received Bytes Back
    swap_bytes(rx_buf, K_WORDS_PER_MESSAGE);

    // Copy Data Back
    memcpy(data, rx_buf, sizeof(spi_data_t));

    // Debug Output
    printf("Received Data: q_abad[0] = %.3f, q_hip[0] = %.3f, q_knee[0] = %.3f\n",
           data->q_abad[0], data->q_hip[0], data->q_knee[0]);
}

int main() {
    init_spi();

    spi_command_t cmd = {};
    spi_data_t data = {};

    // Example Data (Can be replaced with actual inputs)
    cmd.q_des_abad[0] = 0.1f;
    cmd.q_des_hip[0] = -0.2f;
    cmd.q_des_knee[0] = 0.3f;
    cmd.tau_abad_ff[0] = 1.0f;
    cmd.tau_hip_ff[0] = 2.0f;
    cmd.tau_knee_ff[0] = 3.0f;
    cmd.flags[0] = 1;

    while (1) {
        spi_send_receive(&cmd, &data);
        usleep(500000);  // 500ms delay
    }

    close(spi_1_fd);
    close(spi_2_fd);
    return 0;
}