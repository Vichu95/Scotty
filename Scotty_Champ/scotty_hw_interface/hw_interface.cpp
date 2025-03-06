#include <iostream>
#include <fstream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <unistd.h>
#include <stdint.h>
#include <cstring>

#define SPI_DEVICE "/dev/spidev2.0"  

// using namespace std;

#define K_WORDS_PER_MESSAGE 66
unsigned char spi_mode = SPI_MODE_0;
unsigned char spi_bits_per_word = 8;
unsigned int spi_speed = 6000000;   // MIT original speed

class SPI {
private:
    int fd;
    uint8_t mode;
    uint8_t bits;
    uint32_t speed;

public:
    SPI(const char* device, uint8_t spi_mode, uint8_t spi_bits, uint32_t spi_speed) {
        fd = open(device, O_RDWR);
        if (fd < 0) {
            perror("Failed to open SPI device");
            return;
        }

        mode = spi_mode;
        bits = spi_bits;
        speed = spi_speed;

        // Set SPI mode
        if (ioctl(fd, SPI_IOC_WR_MODE, &mode) == -1) {
            perror("Failed to set SPI mode");
            return;
        }

        // Set bits per word
        if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1) {
            perror("Failed to set bits per word");
            return;
        }

        // Set speed
        if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1) {
            perror("Failed to set SPI speed");
            return;
        }
    }

    void transfer(uint16_t* tx_data, uint16_t* rx_data, size_t length) {


        size_t word_len = 2;  // 16 bit word


        struct spi_ioc_transfer spi_trans[1];;
        memset(spi_trans, 0, 1 * sizeof(spi_trans));

        spi_trans[0].tx_buf = (uint64_t)tx_data;
        spi_trans[0].rx_buf = (uint64_t)rx_data;
        spi_trans[0].cs_change = 1;    //original
        spi_trans[0].len = word_len * 66;
        spi_trans[0].speed_hz = speed;
        spi_trans[0].bits_per_word = bits;

        if (ioctl(fd, SPI_IOC_MESSAGE(1), &spi_trans) < 1) {
            perror("SPI transfer failed");
            return;
        }


        printf("Receiving: ");
        // for (int i = 0; i < 3; i++) {
            printf("rx_data[%02d] = 0x%04X\n", 23, rx_data[23]);
            printf("rx_data[%02d] = 0x%04X\n", 24, rx_data[24]);
            printf("rx_data[%02d] = 0x%04X\n", 25, rx_data[25]);
    }

    ~SPI() {
        close(fd);
    }
};

int main() {
    // Initialize SPI
    SPI spi(SPI_DEVICE, spi_mode, spi_bits_per_word, spi_speed);  // Mode 0, 8-bit, 500kHz

    uint16_t tx_data1[K_WORDS_PER_MESSAGE];  // Example data
    uint16_t tx_data[K_WORDS_PER_MESSAGE];  // Example data
    uint16_t rx_data[K_WORDS_PER_MESSAGE];  // Buffer for received data

    // zero rx buffer
    memset(rx_data, 0, K_WORDS_PER_MESSAGE * sizeof(uint16_t));
    memset(tx_data, 0, K_WORDS_PER_MESSAGE * sizeof(uint16_t));

    tx_data1[0] = 1;
    tx_data1[1] = 2;
    tx_data1[2] = 3;


    // copy into tx buffer flipping bytes
    for (int i = 0; i < K_WORDS_PER_MESSAGE; i++)
        tx_data[i] = (tx_data1[i] >> 8) + ((tx_data1[i] & 0xff) << 8);

    while(1)
    {
        // printf("Sending: ");
        // for (int i = 0; i < 3; i++) {
        //     printf("tx_data[%02d] = 0x%04X\n", i, tx_data[i]);
        // }

        // Perform SPI transfer
        spi.transfer(tx_data, rx_data, sizeof(tx_data));

        // }
    }

    return 0;
}
