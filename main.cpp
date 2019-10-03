#include "mainwindow.h"
#include <QApplication>

#include <iostream>
#include <thread>

#include <unistd.h>
#include <termios.h>
#include <sys/stat.h>
#include <fcntl.h>

int set_blocking_mode(int fd, int blocking) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (blocking) { flags &= ~O_NONBLOCK; }
    else { flags |= O_NONBLOCK; }
    fcntl(fd, F_SETFL, flags);
    return 0;
}

int get_blocking_mode(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    return (flags & O_NONBLOCK) ? 1 : 0;
}

// const char voltage_string[] = ":meas:volt?\x0a";
const char current_string[] = ":meas:curr?\x0a";

/* parity = 0 (no parity), = 1 odd parity, = 2 even parity */
int initialize_serial_port(const char *dev, int canonical, int parity, int min_chars) {
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
    // int fd = open(dev, O_RDWR | O_NOCTTY);
    // int fd = open(dev, O_WRONLY | O_NOCTTY);
    if(fd < 0) { return fd; }
    fcntl(fd, F_SETFL, 0);
    struct termios *settings, current_settings;

    memset(&current_settings, 0, sizeof(current_settings));
    tcgetattr(fd, &current_settings);

    /* effect new settings */
    settings = &current_settings;
    cfmakeraw(settings);
    if (parity == 0) {
        settings->c_cflag &= ~(CSIZE | CRTSCTS | CSTOPB | PARENB); /* no parity, one stop bit, no cts/rts, clear size */
        settings->c_cflag |= CS8; /* eight bits */
    } else if (parity == 1) {
        settings->c_cflag &= ~(CSIZE | CRTSCTS | CSTOPB); /* no parity, one stop bit, no cts/rts, clear size */
        settings->c_cflag |= (CS8 | PARENB | PARODD); /* eight bits, odd parity */
    } else if (parity == 2) {
        settings->c_cflag &= ~(CSIZE | CRTSCTS | CSTOPB | PARODD); /* no parity, one stop bit, no cts/rts, clear size */
        settings->c_cflag |= (CS8 | PARENB); /* eight bits, odd parity is clear for even parity */
    }
    settings->c_cflag |= (CLOCAL | CREAD); /* ignore carrier detect. enable receiver */
    settings->c_iflag &= ~(IXON | IXOFF | IXANY | IGNPAR | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    settings->c_iflag |= ( IGNPAR | IGNBRK);
    settings->c_lflag &= ~(ECHOK | ECHOCTL | ECHOKE);
    if (canonical) { settings->c_lflag |= ICANON; } /* set canonical */
    else { settings->c_lflag &= ~ICANON; } /* or clear it */
    settings->c_oflag &= ~(OPOST | ONLCR);
    settings->c_cc[VMIN] = min_chars;
    settings->c_cc[VTIME] = 1; /* 200ms timeout */

    cfsetispeed(settings, B9600);
    cfsetospeed(settings, B9600);

    tcsetattr(fd, TCSANOW, settings); /* apply settings */
    tcflush(fd, TCIOFLUSH);

    return fd;
}

class Idaho {
public:
    int fd;
    int run;
    Idaho() : run(1) { }
    char result_buffer[128];
    void operator()() {

        const char *device_name = "/dev/ttyUSB0";
        int canonical_mode = 0;
        int parity = 0;
        int min_chars = 0;

        fd = initialize_serial_port(device_name, canonical_mode, parity, min_chars);
        int flag = get_blocking_mode(fd);
        ssize_t n_bytes;
        // set_blocking_mode(fd, 0);
        flag = get_blocking_mode(fd);

        time_t previous_time = 0;
        while (run) {
            time_t now = time(0);
            if (previous_time == 0) {
                previous_time = now;
            } else if (previous_time == now) {
                usleep(100000);
                continue;
            }
            n_bytes = write(fd, current_string, sizeof(current_string) - 1);
            snprintf(result_buffer, sizeof(result_buffer), "%s", "the quick brown fox\n");

            usleep(500000);

            previous_time = now;
            n_bytes = read(fd, result_buffer, sizeof(result_buffer));
            if (n_bytes > 0) {
                result_buffer[n_bytes] = 0;
                printf("current = [%s]\n", result_buffer);
            } else {
                printf("unresponsive device %d\n", now);
                perror("skippy");
            }
        }
    }
};

class Skipper {
public:
    Skipper() : run(1) { thread_obj = new std::thread((Idaho())); }
    // Skipper(const std::string &device);
    int fd;
    int run;
    std::thread *thread_obj;
};

//Skipper::Skipper(const std::string &device) {
//    run = 1;
//    fd = initialize_serial_port(device.c_str(), 0, 0, 0);
//    int flag = get_blocking_mode(fd);
//    set_blocking_mode(fd, 0);
//    flag = get_blocking_mode(fd);
    // thread_obj = (fd > 0) ? new std::thread((Idaho(fd))) : nullptr;
//}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    Skipper *skipper = new Skipper; // (std::string("/dev/ttyUSB0"));

    return a.exec();
}
