#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <sys/ioctl.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

// Define the maximum number of serial ports
#define MAX_PORTS 10

#define CAN_ID_MASTER (0X00)        // Control host address - SPIE
#define CAN_ID_MOTOR_DEFAULT (0X7F) // Default motor address - Unconfigured ID
#define CAN_ID_BROADCAST (0XFE) // Broadcast address - Default receive address
#define CAN_ID_DEBUG_UI (0XFD)  // Debug address - Upper computer address

// Ubuntu
#define BAUDRATE 921600
#define TTY_PORT "/dev/ttyCH341USB0"

// Mac
// Note that this baudrate will let you send something to the device on Mac, but
// it won't work because the device baudrate is supposed to be 921600.
// #define BAUDRATE 115200
// #define TTY_PORT "/dev/tty.usbserial-110"

// Replace quint8 and quint32 with standard types
typedef uint8_t uint8_t;
typedef uint32_t uint32_t;

// Add these enums
enum canComMode {
  CANCOM_ANNOUNCE_DEVID = 0,
  CANCOM_MOTOR_CTRL,
  CANCOM_MOTOR_FEEDBACK,
  CANCOM_MOTOR_IN,
  CANCOM_MOTOR_RESET,
  CANCOM_MOTOR_CALI,
  CANCOM_MOTOR_ZERO,
  CANCOM_MOTOR_ID,
  CANCOM_PARA_WRITE,
  CANCOM_PARA_READ,
  CANCOM_PARA_UPDATE,
  CANCOM_OTA_START,
  CANCOM_OTA_INFO,
  CANCOM_OTA_ING,
  CANCOM_OTA_END,
  CANCOM_CALI_ING,
  CANCOM_CALI_RST,
  CANCOM_SDO_READ,
  CANCOM_SDO_WRITE,
  CANCOM_PARA_STR_INFO,
  CANCOM_MOTOR_BRAKE,
  CANCOM_FAULT_WARN,
  CANCOM_MODE_TOTAL,
};

enum motorMode {
  MT_MODE_RESET = 0,
  MT_MODE_CALI,
  MT_MODE_MOTOR,
  MT_MODE_BRAKE,
};

struct motorStatus {
  bool underVoltFault;
  bool overCurFault;
  bool overTempFault;
  bool encoderFault;
  bool hallFault;
  bool noCaliFault;
  enum motorMode mode;
};

// E: 41 54 18 07 e8 0c 08 00 00 00 00 00 00 00 0d 0a
// T: 41 54 18 07 e8 0c 08 00 00 00 00 00 00 00 00 0d 0a

struct ExId {
  uint32_t id : 8;
  uint32_t data : 16;
  canComMode mode : 5;
  uint32_t res : 3;
};

struct CanPack {
  ExId exId;
  uint8_t len;
  uint8_t data[8];
};

// Serial port handles
int my_serialport[MAX_PORTS];

// Receive buffer for each serial port
std::vector<uint8_t> rxBuffer[MAX_PORTS];

// Receive frame for each serial port
CanPack rxFrame[MAX_PORTS];

// Send data function
void txdPack(uint8_t index, CanPack *pack) {
  std::cout << "Sending data on port " << (int)index << std::endl;
  std::vector<uint8_t> Pack;

  if (pack->len < 1) {
    pack->len = 1;
    pack->data[0] = 0;
  } else if (pack->len > 8) {
    pack->len = 8;
  }
  pack->exId.res = 0;

  Pack.resize(9 + pack->len);

  Pack[0] = 'A';
  Pack[1] = 'T';

  uint32_t addr;
  memcpy(&addr, &(pack->exId), 4);
  addr = (addr << 3) | (0x00000004);

  Pack[2] = (uint8_t)((addr & 0xFF000000) >> 24);
  Pack[3] = (uint8_t)((addr & 0x00FF0000) >> 16);
  Pack[4] = (uint8_t)((addr & 0x0000FF00) >> 8);
  Pack[5] = (uint8_t)((addr & 0x000000FF));
  Pack[6] = pack->len;

  for (uint8_t i = 0; i < pack->len; i++)
    Pack[7 + i] = pack->data[i];

  Pack[7 + pack->len] = '\r';
  Pack[8 + pack->len] = '\n';

  // Print the full string of bits being sent
  std::cout << "Full packet (hex): ";
  for (size_t i = 0; i < Pack.size(); ++i) {
    std::cout << std::setfill('0') << std::setw(2) << std::hex << (int)Pack[i]
              << " ";
  }
  std::cout << std::dec << std::endl;

  // Write the data
  ssize_t bytes_written =
      write(my_serialport[index], Pack.data(), (uint16_t)Pack.size());
  if (bytes_written < 0) {
    std::cerr << "Error writing to serial port: " << strerror(errno)
              << std::endl;
  } else if (bytes_written != Pack.size()) {
    std::cerr << "Warning: Not all bytes written. Expected " << Pack.size()
              << ", wrote " << bytes_written << std::endl;
  }

  // Clear the buffer
  tcflush(my_serialport[index], TCIFLUSH);

  std::cout << "Data sent successfully" << std::endl;
}

int initSerialPort(const char *device) {
  std::cout << "Initializing serial port: " << device << std::endl;

  int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);

  if (fd == -1) {
    std::cerr << "Error opening serial port: " << strerror(errno) << std::endl;
    return -1;
  }

  // Clear the O_NONBLOCK flag to make read/write blocking again
  int flags = fcntl(fd, F_GETFL, 0);
  flags &= ~O_NONBLOCK;
  fcntl(fd, F_SETFL, flags);

  struct termios options;
  tcgetattr(fd, &options);

  // Configure serial port settings (baud rate, data bits, stop bits, parity,
  // etc.)
  cfsetispeed(&options, BAUDRATE);
  cfsetospeed(&options, BAUDRATE);

  // 8 data bits, no parity, 1 stop bit
  options.c_cflag &= ~PARENB; // No parity
  options.c_cflag &= ~CSTOPB; // One stop bit
  options.c_cflag &= ~CSIZE;  // Mask the character size bits
  options.c_cflag |= CS8;     // 8 data bits

  // Enable the receiver and set local mode
  options.c_cflag |=
      CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  // Set raw input/output mode
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
  options.c_oflag &= ~OPOST;                          // Raw output

  // Set VMIN and VTIME
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 10; // Timeout in deciseconds (1 second)

  // Apply the settings
  if (tcsetattr(fd, TCSANOW, &options) != 0) {
    std::cerr << "Error setting terminal attributes: " << strerror(errno)
              << std::endl;
    close(fd);
    return -1;
  }

  // Verify the settings
  struct termios verifyOptions;
  if (tcgetattr(fd, &verifyOptions) != 0) {
    std::cerr << "Error getting terminal attributes: " << strerror(errno)
              << std::endl;
    close(fd);
    return -1;
  }

  if (verifyOptions.c_cflag != options.c_cflag ||
      verifyOptions.c_iflag != options.c_iflag ||
      verifyOptions.c_oflag != options.c_oflag ||
      verifyOptions.c_lflag != options.c_lflag) {
    std::cerr << "Serial port settings do not match the requested configuration"
              << std::endl;
    close(fd);
    return -1;
  }

  // Flush the serial port
  tcflush(fd, TCIOFLUSH);

  std::cout << "Serial port initialized successfully" << std::endl;
  return fd;
}

void readAndPrintResponse(uint8_t index) {
  std::vector<uint8_t> response;
  char buffer[1];
  ssize_t bytes_read;
  bool startFound = false;

  std::cout << "Waiting for response..." << std::endl;

  // Increase timeout to 5 seconds
  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(1)) {
    bytes_read = read(my_serialport[index], buffer, 1);
    if (bytes_read > 0) {
      std::cout << "Read byte: 0x" << std::hex << std::setw(2)
                << std::setfill('0') << (int)(unsigned char)buffer[0]
                << std::dec << std::endl;
      if (buffer[0] == 'A' && !startFound) {
        startFound = true;
        response.push_back(buffer[0]);
      } else if (startFound) {
        response.push_back(buffer[0]);
        if (buffer[0] == '\n' && response.size() >= 2 &&
            response[response.size() - 2] == '\r') {
          std::cout << "Complete packet received" << std::endl;
          break;
        }
      }
    } else if (bytes_read == 0) {
      std::cout << "No bytes available, waiting..." << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } else {
      std::cerr << "Error reading from serial port: " << strerror(errno)
                << std::endl;
      break;
    }
  }

  if (response.size() > 0) {
    std::cout << "Response packet (hex): ";
    for (size_t i = 0; i < response.size(); ++i) {
      std::cout << std::setfill('0') << std::setw(2) << std::hex
                << (int)response[i] << " ";
    }
    std::cout << std::dec << std::endl;

    // Parse the response if it's a valid packet
    if (response.size() >= 9 && response[0] == 'A' && response[1] == 'T') {
      uint32_t addr = (response[2] << 24) | (response[3] << 16) |
                      (response[4] << 8) | response[5];
      uint8_t len = response[6];
      std::cout << "Address: 0x" << std::hex << addr << std::dec << std::endl;
      std::cout << "Length: " << (int)len << std::endl;
      std::cout << "Data: ";
      for (int i = 0; i < len && i < 8; ++i) {
        std::cout << std::setfill('0') << std::setw(2) << std::hex
                  << (int)response[7 + i] << " ";
      }
      std::cout << std::dec << std::endl;
    }
  } else {
    std::cout << "No response received" << std::endl;
  }
}

void send_reset(uint8_t index, uint8_t id) {
  CanPack pack;
  memset(&pack, 0, sizeof(CanPack));
  pack.len = 8;
  pack.exId.id = id;
  pack.exId.data = CAN_ID_DEBUG_UI;
  pack.exId.mode = CANCOM_MOTOR_RESET;
  pack.exId.res = 0;

  txdPack(index, &pack);
  // readAndPrintResponse(index);
}

void send_start(uint8_t index, uint8_t id) {
  CanPack pack;
  memset(&pack, 0, sizeof(CanPack));
  pack.len = 8;
  pack.exId.id = id;
  pack.exId.data = CAN_ID_DEBUG_UI;
  pack.exId.mode = CANCOM_MOTOR_IN;
  pack.exId.res = 0;

  txdPack(index, &pack);
  // readAndPrintResponse(index);
}

int main() {
  std::cout << "Starting program" << std::endl;

  // Initialize serial ports
  my_serialport[0] = initSerialPort(TTY_PORT);
  if (my_serialport[0] == -1) {
    std::cerr << "Failed to initialize serial port" << std::endl;
    return -1;
  }

  // Add a delay after initializing the port
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Initialize the device
  uint8_t deviceId = 1; // Use the default motor address
  send_start(0, deviceId);
  // send_reset(0, deviceId);

  // Close the serial port when done
  std::cout << "Closing serial port" << std::endl;
  close(my_serialport[0]);

  std::cout << "Program finished" << std::endl;
  return 0;
}
