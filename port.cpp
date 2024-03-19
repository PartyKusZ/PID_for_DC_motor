#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <vector>
#include <fstream>
#include <thread>
#include <atomic>

int main() {
    const char *portname = "/dev/ttyACM0";
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Error opening " << portname << std::endl;
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
        return -1;
    }

    cfsetospeed(&tty, B230400);
    cfsetispeed(&tty, B230400);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
        return -1;
    }

    std::vector<float> data;
    std::atomic<bool> done(false);

    // Wątek do odbierania danych z portu szeregowego
    std::thread readThread([&]() {
        unsigned char buf[4];
        while (!done.load()) {
            int n = read(fd, buf, sizeof( buf)); // Próba odczytu 4 bajtów
            if (n == 4) {
                float value;
                memcpy(&value, buf, sizeof(float)); // Bezpieczne rzutowanie
                data.push_back(value);
                // std::cout << "Odczytano wartość float: " << (int)value << std::endl;
            }
        }
    });

    // Oczekiwanie na 'q' z stdin
    std::cout << "Wpisz 'q', aby zakończyć i zapisać dane do pliku." << std::endl;
    char q;
    do {
        std::cin >> q;
    } while (q != 'q');
    done.store(true);

    readThread.join();
    close(fd);

    // Zapisywanie danych do pliku
    std::ofstream outFile("rpm.txt");
    for (const auto &value : data) {
        outFile << value << std::endl;
    }
    outFile.close();

    std::cout << "Dane zostały zapisane do pliku rpm.txt." << std::endl;

    return 0;
}
