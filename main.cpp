#include <iostream>
#include <string.h>
#include <fcntl.h>      // contains file controls like O_RDWR
#include <errno.h>      // error integer and strerror() function
#include <termios.h>    // contains POSIX terminal control definitions
#include <unistd.h>     // write(), read(), close()
#include <memory>


#include "bipropellant-api/HoverboardAPI.h"

//#define DRIVE_FILE_PATH     "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
#define DRIVE_FILE_PATH     "/dev/ttyUSB0"

static int serialPort;

int serial_wrapper(unsigned char *data, int len) {
//    for (int inx = 0; inx < len; inx++) {
//        std::cout << (int) data[inx];
//        std::cout << " ";
//    }
//    std::cout << "\n\n" << len << " <- L\n";
    return write(serialPort, data, len);
}

int main() {
    serialPort = open(DRIVE_FILE_PATH, O_RDWR);
    //serialPort = open(DRIVE_FILE_PATH, O_RDWR | O_NOCTTY | O_NDELAY);
    //fcntl(serialPort, F_SETFL, 0);

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


//    tty.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
//    tty.c_iflag = IGNPAR;
//    tty.c_oflag = 0;
//    tty.c_lflag = 0;
//    tcflush(serialPort, TCIFLUSH);
//    tcsetattr(serialPort, TCSANOW, &tty);



//     there is a configuration of CONTROL MODES (c_cflag)
    tty.c_cflag &= ~PARENB;     //+ clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB;     //+ clear stop field, only one stop bit used in communication (most common)

    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         //+- 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS;    // disable RST/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines (CLOCAL = 1)

//    there is a configuration of LOCAL MODES (c_lflag)
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;       // disable echo
    tty.c_lflag &= ~ECHOE;      // disable erasure
    tty.c_lflag &= ~ECHONL;     // disable new-line echo
    tty.c_lflag &= ~ISIG;       // disable interpretation of INTR, QUIN and SUSP

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                     ICRNL); // Disable any special handling of received bytes


//     there is a configuration of OUTPUT MODES (c_oflag)
    tty.c_oflag &= ~OPOST; // prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // prevent conversion of newline to carriage return/line feed

//     there is a configuration of VMIN and VTIME (c_cc)
    tty.c_cc[VTIME] = 0;   // there is VMIN = 0, VTIME = 0: no blocking, return immediately with what is available;
    tty.c_cc[VMIN] = 0;

//     there is a configuration of BAUD RATE (input and output speed)
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
//     saving termios
    if (tcsetattr(serialPort, TCSANOW, &tty) != 0) {
        std::cout << "CONFIGURE ERROR 2\n";
        return 2;
    }


    std::shared_ptr<HoverboardAPI> hoverboardApi = std::make_shared<HoverboardAPI>(serial_wrapper);


    int inx = 0;
    int power = 0;

    std::cout << "Set 0 state for the iron turtle controller\n";
   // hoverboardApi->sendPIDControl(50, 20, 10, 30, PROTOCOL_SOM_NOACK);
    usleep(1000 * 3000);
    std::cout << "DONE: Set 0 state for the iron turtle controller\n";
    inx = 0;

    double current_speed = 0;

    while (inx < 50) {
        hoverboardApi->sendSpeedData(0.0, current_speed, power, 5, PROTOCOL_SOM_NOACK);
        current_speed = (current_speed < 0.05) ? (current_speed + 0.001) : 0.05;
        usleep(1000 * 30);
        inx++;
    }
    inx = 0;
    while (inx < 200) {
        hoverboardApi->sendSpeedData(0.0, 0.05, power, 5, PROTOCOL_SOM_NOACK);
        power = (power < 200) ? (power + 50) : 200;
        usleep(1000 * 30);
        inx++;
    }

    std::cout << "step 1 passed\n";

    inx = 0;
    current_speed = 0.05;
    while (inx < 50) {
        hoverboardApi->sendSpeedData(0.0, current_speed, power, 5, PROTOCOL_SOM_NOACK);
        current_speed = (current_speed > 0) ? (current_speed - 0.001) : 0;
        usleep(1000 * 30);
        inx++;
    }

// // // // // // // // // // //

    inx = 0;
    power = 0;

    std::cout << "Set 0 state for the iron turtle controller\n";

    usleep(1000 * 3000);
    inx = 0;
    while (inx < 50) {
        hoverboardApi->sendSpeedData(0.0, current_speed, power, 5, PROTOCOL_SOM_NOACK);
        current_speed = (current_speed > -0.05) ? (current_speed - 0.001) : -0.05;
        usleep(1000 * 30);
        inx++;
    }
    std::cout << "DONE: Set 0 state for the iron turtle controller\n";
    inx = 0;
    while (inx < 200) {
        hoverboardApi->sendSpeedData(0.0, -0.05, power, 5, PROTOCOL_SOM_NOACK);
        power = (power < 200) ? (power + 25) : 200;
        usleep(1000 * 30);
        inx++;
    }

    std::cout << "step 2 passed\n";
    inx = 0;
    while (inx < 50) {
        hoverboardApi->sendSpeedData(0.0, current_speed, power, 5, PROTOCOL_SOM_NOACK);
        current_speed = (current_speed < 0) ? (current_speed + 0.001) : 0;
        usleep(1000 * 30);
        inx++;
    }


    close(serialPort);
    return 0;
}
