/*
 * aurora_reset_fixed.cpp
 * Fixed version - handles device state properly before reset
 * 
 * Compile: g++ -o aurora_reset.exe aurora_reset_fixed.cpp
 * Usage: aurora_reset.exe COM3 soft
 */

#include <iostream>
#include <string>
#include <cstring>
#include <chrono>
#include <thread>
#include <algorithm>

#ifdef _WIN32
    #include <windows.h>
#else
    #include <fcntl.h>
    #include <unistd.h>
    #include <termios.h>
#endif

class AuroraReset {
private:
    std::string port;
    int baudrate;
    
#ifdef _WIN32
    HANDLE hSerial;
#else
    int fd;
#endif

    const std::string RESET_SOFT = "0";
    const std::string RESET_HARD = "1";

public:
    AuroraReset(const std::string& portName, int baud = 9600) 
        : port(portName), baudrate(baud) {
#ifdef _WIN32
        hSerial = INVALID_HANDLE_VALUE;
#else
        fd = -1;
#endif
    }

    ~AuroraReset() {
        disconnect();
    }

    bool connect() {
        std::cout << "Connecting to Aurora device on " << port << "..." << std::endl;

#ifdef _WIN32
        // Windows implementation
        hSerial = CreateFileA(port.c_str(),
                             GENERIC_READ | GENERIC_WRITE,
                             0,
                             NULL,
                             OPEN_EXISTING,
                             FILE_ATTRIBUTE_NORMAL,
                             NULL);
        
        if (hSerial == INVALID_HANDLE_VALUE) {
            std::cerr << "Error: Unable to open serial port" << std::endl;
            return false;
        }

        DCB dcbSerialParams = {0};
        dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
        
        if (!GetCommState(hSerial, &dcbSerialParams)) {
            std::cerr << "Error: Unable to get serial port state" << std::endl;
            return false;
        }

        dcbSerialParams.BaudRate = baudrate;
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity = NOPARITY;

        if (!SetCommState(hSerial, &dcbSerialParams)) {
            std::cerr << "Error: Unable to set serial port state" << std::endl;
            return false;
        }

        COMMTIMEOUTS timeouts = {0};
        timeouts.ReadIntervalTimeout = 50;
        timeouts.ReadTotalTimeoutConstant = 5000;
        timeouts.ReadTotalTimeoutMultiplier = 10;
        timeouts.WriteTotalTimeoutConstant = 5000;
        timeouts.WriteTotalTimeoutMultiplier = 10;

        if (!SetCommTimeouts(hSerial, &timeouts)) {
            std::cerr << "Error: Unable to set timeouts" << std::endl;
            return false;
        }
#else
        // Linux implementation
        fd = open(port.c_str(), O_RDWR | O_NOCTTY);
        
        if (fd == -1) {
            std::cerr << "Error: Unable to open serial port" << std::endl;
            return false;
        }

        struct termios options;
        tcgetattr(fd, &options);
        
        cfsetispeed(&options, B9600);
        cfsetospeed(&options, B9600);
        
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag &= ~CRTSCTS;
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag &= ~OPOST;
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 50;
        
        tcsetattr(fd, TCSANOW, &options);
        tcflush(fd, TCIOFLUSH);
#endif

        std::cout << "Connected successfully" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        return true;
    }

    std::string sendCommand(const std::string& command) {
        std::string cmd = command + "\r";
        std::string response;

#ifdef _WIN32
        DWORD bytesWritten;
        if (!WriteFile(hSerial, cmd.c_str(), cmd.length(), &bytesWritten, NULL)) {
            std::cerr << "Error writing to serial port" << std::endl;
            return "";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        char buffer[256] = {0};
        DWORD bytesRead;
        if (ReadFile(hSerial, buffer, sizeof(buffer) - 1, &bytesRead, NULL)) {
            response = std::string(buffer, bytesRead);
        }
#else
        ssize_t bytesWritten = write(fd, cmd.c_str(), cmd.length());
        if (bytesWritten < 0) {
            std::cerr << "Error writing to serial port" << std::endl;
            return "";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        char buffer[256] = {0};
        ssize_t bytesRead = read(fd, buffer, sizeof(buffer) - 1);
        if (bytesRead > 0) {
            response = std::string(buffer, bytesRead);
        }
#endif

        // Remove carriage return and line feed
        response.erase(std::remove(response.begin(), response.end(), '\r'), response.end());
        response.erase(std::remove(response.begin(), response.end(), '\n'), response.end());
        
        return response;
    }

    bool initDevice() {
        std::cout << "Initializing Aurora device..." << std::endl;
        std::string response = sendCommand("INIT ");
        std::cout << "Initialization response: " << response << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        return response.find("OKAY") != std::string::npos;
    }

    bool stopTracking() {
        std::cout << "Stopping tracking mode..." << std::endl;
        std::string response = sendCommand("TSTOP ");
        std::cout << "Stop tracking response: " << response << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // It's okay if this fails - device might not be in tracking mode
        return true;
    }

    bool reset(const std::string& resetType = "soft") {
        std::string resetCode;
        std::string resetName;

        if (resetType == "soft") {
            resetCode = RESET_SOFT;
            resetName = "Soft";
        } else if (resetType == "hard") {
            resetCode = RESET_HARD;
            resetName = "Hard";
        } else {
            std::cerr << "Invalid reset type. Use 'soft' or 'hard'" << std::endl;
            return false;
        }

        // CRITICAL: Stop tracking before reset
        stopTracking();

        std::cout << "\nPerforming " << resetName << " Reset..." << std::endl;
        std::string response = sendCommand("RESET " + resetCode);
        std::cout << "Reset response: " << response << std::endl;
        
        // Check if reset was successful
        if (response.find("OKAY") != std::string::npos) {
            std::cout << "✓ Reset successful!" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(2));
            return true;
        } else if (response.find("ERROR") != std::string::npos) {
            std::cerr << "✗ Reset failed with error: " << response << std::endl;
            return false;
        } else {
            std::cout << "Reset command sent (unknown response)" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(2));
            return true;
        }
    }

    void disconnect() {
#ifdef _WIN32
        if (hSerial != INVALID_HANDLE_VALUE) {
            CloseHandle(hSerial);
            hSerial = INVALID_HANDLE_VALUE;
            std::cout << "Disconnected from Aurora device" << std::endl;
        }
#else
        if (fd != -1) {
            close(fd);
            fd = -1;
            std::cout << "Disconnected from Aurora device" << std::endl;
        }
#endif
    }
};

int main(int argc, char* argv[]) {
    // Default values
    std::string serialPort = "COM3";  // Change for your system
    std::string resetType = "soft";

    // Parse command line arguments
    if (argc >= 2) {
        serialPort = argv[1];
    }
    if (argc >= 3) {
        resetType = argv[2];
    }

    std::cout << "Aurora Device Reset Utility" << std::endl;
    std::cout << "===========================" << std::endl;
    std::cout << "Port: " << serialPort << std::endl;
    std::cout << "Reset Type: " << resetType << std::endl << std::endl;

    AuroraReset aurora(serialPort);

    if (!aurora.connect()) {
        std::cerr << "Failed to connect to Aurora device" << std::endl;
        std::cout << "\nPress Enter to exit...";
        std::cin.get();
        return 1;
    }

    if (!aurora.initDevice()) {
        std::cerr << "Failed to initialize device" << std::endl;
        std::cout << "\nPress Enter to exit...";
        std::cin.get();
        return 1;
    }

    if (!aurora.reset(resetType)) {
        std::cerr << "Reset failed" << std::endl;
        std::cout << "\nPress Enter to exit...";
        std::cin.get();
        return 1;
    }

    std::cout << "\n✓ Reset completed successfully!" << std::endl;
    
    std::cout << "\nPress Enter to exit...";
    std::cin.get();
    
    return 0;
}
