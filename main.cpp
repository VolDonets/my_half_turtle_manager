#include <iostream>
#include <string.h>
#include <fcntl.h>      // contains file controls like O_RDWR
#include <errno.h>      // error integer and strerror() function
#include <termios.h>    // contains POSIX terminal control definitions
#include <unistd.h>     // write(), read(), close()


#include "bipropellant-api/HoverboardAPI.h"

#define DRIVE_FILE_PATH     "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
static int serialPort;

int serial_wrapper(unsigned char *data, int len) {
    return write(serialPort, data, len);
}

int main() {
    serialPort = open(DRIVE_FILE_PATH, O_RDWR);
    if (!serialPort) {
        if (errno == 2) {
            std::cout << "DEVICE NOT CONNECTED\n";
        } else if (errno == 13) {
            std::cout << "PERMISSION TO DRIVE FILE DENIED\n";
        } else {
            std::cout << "OTHER TYPE OF ERROR\n";
        }
        return 1;
    }

    struct termios tty;                     // create new termios struct, we call it 'tty' for convention
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(serialPort, &tty) != 0) {
        std::cout << "CONFIGURE ERROR 1\n";
        return 2;
    }

    // there is a configuration of CONTROL MODES (c_cflag)
    tty.c_cflag &= ~PARENB;     // clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB;     // clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= CS8;         // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS;    // disable RST/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines (CLOCAL = 1)

    // there is a configuration of LOCAL MODES (c_lflag)
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;       // disable echo
    tty.c_lflag &= ~ECHOE;      // disable erasure
    tty.c_lflag &= ~ECHONL;     // disable new-line echo
    tty.c_lflag &= ~ISIG;       // disable interpretation of INTR, QUIN and SUSP

    // there is a configuration of OUTPUT MODES (c_oflag)
    tty.c_oflag &= ~OPOST; // prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // prevent conversion of newline to carriage return/line feed

    // there is a configuration of VMIN and VTIME (c_cc)
    tty.c_cc[VTIME] = 0;   // there is VMIN = 0, VTIME = 0: no blocking, return immediately with what is available;
    tty.c_cc[VMIN] = 0;

    // there is a configuration of BAUD RATE (input and output speed)
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    // saving termios
    if (tcsetattr(serialPort, TCSANOW, &tty) != 0){
        std::cout << "CONFIGURE ERROR 2\n";
        return 2;
    }

    HoverboardAPI hoverboardApi = HoverboardAPI(serial_wrapper);

    int inx = 0;

    while (inx < 100) {
        hoverboardApi.sendPWM(100, 30, PROTOCOL_SOM_NOACK);
        usleep(1000 * 50);
        inx++;
    }

    close(serialPort);
    return 0;
}
