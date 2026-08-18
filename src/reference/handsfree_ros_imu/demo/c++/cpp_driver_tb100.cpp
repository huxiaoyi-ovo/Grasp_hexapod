/*
cpp_driver_tb100.cpp
--------------------------------------------
TB100 IMU serial reader (Linux & Windows, single-file)

Packet: 73 bytes
[0]=0xAA [1]=0x55 [2]=0x44 ... [7..70]=payload, [71]=CRC_lo, [72]=CRC_hi
CRC16(Modbus) over bytes [2..70].

Usage
-----
Linux:
  g++ -std=c++17 -O2 cpp_driver_tb100.cpp -o cpp_driver_tb100
  ./cpp_driver_tb100                 # uses defaults: /dev/ttyACM0 @ 921600
  ./cpp_driver_tb100 --port /dev/ttyACM0 --baud 921600
  ./cpp_driver_tb100 --list

Windows (MinGW):
  g++ -std=c++17 -O2 cpp_driver_tb100.cpp -lsetupapi -o cpp_driver_tb100.exe
  cpp_driver_tb100.exe               # uses defaults: COM3 @ 921600
  cpp_driver_tb100.exe --port COM3 --baud 921600
  cpp_driver_tb100.exe --list
*/

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <sstream>

#ifdef _WIN32
  #define NOMINMAX
  #include <windows.h>
  #include <setupapi.h>
  #include <devguid.h>
  #pragma comment(lib, "setupapi.lib")
#else
  #include <fcntl.h>
  #include <unistd.h>
  #include <dirent.h>
  #include <sys/ioctl.h>
  #include <sys/stat.h>
  #include <sys/types.h>
  #include <termios.h>        // 只用这个，不再引 asm/linux 的 termbits
  #ifdef __linux__
    #include <linux/serial.h> // 为了 TIOCGSERIAL/TIOCSSERIAL 自定义分频
  #endif
  #include <cerrno>
#endif

// ---------- utility ----------
static inline void msleep(int ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
static inline uint16_t crc16_modbus(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i=0;i<len;i++){
    crc ^= data[i];
    for(int b=0;b<8;b++){
      if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  return crc;
}
template<typename T> static inline T rd_le(const uint8_t* p);
template<> inline uint16_t rd_le<uint16_t>(const uint8_t* p){ return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
template<> inline int16_t  rd_le<int16_t >(const uint8_t* p){ return (int16_t)rd_le<uint16_t>(p); }
template<> inline uint32_t rd_le<uint32_t>(const uint8_t* p){ return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
template<> inline int32_t  rd_le<int32_t >(const uint8_t* p){ return (int32_t)rd_le<uint32_t>(p); }

// ---------- port enumeration ----------
static std::vector<std::string> list_candidate_ports() {
  std::vector<std::string> out;
#ifdef _WIN32
  HDEVINFO hDevInfo = SetupDiGetClassDevs(&GUID_DEVCLASS_PORTS, NULL, NULL, DIGCF_PRESENT);
  if (hDevInfo != INVALID_HANDLE_VALUE) {
    SP_DEVINFO_DATA devInfoData; devInfoData.cbSize = sizeof(SP_DEVINFO_DATA);
    for (DWORD i=0; SetupDiEnumDeviceInfo(hDevInfo, i, &devInfoData); ++i) {
      HKEY hKey = SetupDiOpenDevRegKey(hDevInfo, &devInfoData, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
      if (hKey != INVALID_HANDLE_VALUE) {
        char portName[256]; DWORD len = sizeof(portName);
        if (RegQueryValueExA(hKey, "PortName", NULL, NULL, (LPBYTE)portName, &len) == ERROR_SUCCESS)
          out.emplace_back(portName); // e.g., COM3
        RegCloseKey(hKey);
      }
    }
    SetupDiDestroyDeviceInfoList(hDevInfo);
  }
  if (out.empty()){
    for (int i=1; i<=256; ++i) {
      std::ostringstream os; os << "COM" << i;
      std::string nm = os.str();
      std::string path = "\\\\.\\" + nm;
      HANDLE h = CreateFileA(path.c_str(), GENERIC_READ|GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
      if (h != INVALID_HANDLE_VALUE) { out.push_back(nm); CloseHandle(h); }
    }
  }
#else
  auto scan = [&](const char* dir, const char* prefix){
    DIR* d = opendir(dir);
    if (!d) return;
    struct dirent* ent;
    while ((ent = readdir(d)) != nullptr) {
      std::string name = ent->d_name;
      if (name.rfind(prefix,0)==0) out.emplace_back(std::string(dir)+"/"+name);
    }
    closedir(d);
  };
  scan("/dev","ttyUSB");
  scan("/dev","ttyACM");
#endif
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return out;
}

// ---------- serial wrapper ----------
class SerialPort {
public:
  SerialPort():opened_(false) {}
  ~SerialPort(){ close(); }

  bool open(const std::string& name, int baud){
#ifdef _WIN32
    std::string path = "\\\\.\\" + name;
    h_ = CreateFileA(path.c_str(), GENERIC_READ|GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (h_ == INVALID_HANDLE_VALUE) return false;

    SetupComm(h_, 1<<16, 1<<16);
    DCB dcb = {0}; dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(h_, &dcb)) { CloseHandle(h_); return false; }
    dcb.BaudRate = baud; dcb.ByteSize = 8; dcb.Parity = NOPARITY; dcb.StopBits = ONESTOPBIT;
    dcb.fOutxCtsFlow = FALSE; dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE; dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fOutX = FALSE; dcb.fInX = FALSE;
    if (!SetCommState(h_, &dcb)) { CloseHandle(h_); return false; }

    COMMTIMEOUTS to = {0};
    to.ReadIntervalTimeout         = 1;
    to.ReadTotalTimeoutMultiplier  = 0;
    to.ReadTotalTimeoutConstant    = 5;
    to.WriteTotalTimeoutMultiplier = 0;
    to.WriteTotalTimeoutConstant   = 5;
    SetCommTimeouts(h_, &to);

    opened_ = true; name_ = name; baud_ = baud;
    return true;
#else
    fd_ = ::open(name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    // 先设置标准 termios（大多数系统支持 B921600）
    struct termios tio;
    if (tcgetattr(fd_, &tio) != 0) { ::close(fd_); fd_ = -1; return false; }
    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~PARENB; // 8N1
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cc[VTIME] = 0; // non-blocking read
    tio.c_cc[VMIN]  = 0;

    // 选择最接近的标准速率
    speed_t sp = B115200;
#ifdef B230400
    if (baud == 230400) sp = B230400;
#endif
#ifdef B460800
    if (baud == 460800) sp = B460800;
#endif
#ifdef B921600
    if (baud == 921600) sp = B921600;
#endif
    cfsetispeed(&tio, sp);
    cfsetospeed(&tio, sp);
    if (tcsetattr(fd_, TCSANOW, &tio) != 0) { ::close(fd_); fd_ = -1; return false; }

    // 如果请求的波特率不是我们刚设置的标准值，并且是 Linux，则尝试自定义分频
#ifdef __linux__
    auto applied_baud = (sp==B115200?115200:
#ifdef B230400
                        sp==B230400?230400:
#endif
#ifdef B460800
                        sp==B460800?460800:
#endif
#ifdef B921600
                        sp==B921600?921600:
#endif
                        115200);
    if (applied_baud != baud) {
      struct serial_struct ser{};
      if (ioctl(fd_, TIOCGSERIAL, &ser) == 0 && ser.baud_base > 0) {
        ser.flags &= ~ASYNC_SPD_MASK;
        ser.flags |= ASYNC_SPD_CUST;
        int div = (int)(ser.baud_base / baud);
        if (div == 0) div = 1;
        ser.custom_divisor = div;
        if (ioctl(fd_, TIOCSSERIAL, &ser) == 0) {
          // 触发使能：再写一次 termios 的 38400（Linux 传统约定）
          cfsetispeed(&tio, B38400);
          cfsetospeed(&tio, B38400);
          tcsetattr(fd_, TCSANOW, &tio);
        }
      }
    }
#endif

    // 设为阻塞-短超时：靠 VMIN/VTIME 已经 non-blocking，这里保留
    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

    opened_ = true; name_ = name; baud_ = baud;
    return true;
#endif
  }

  void close(){
    if (!opened_) return;
#ifdef _WIN32
    CloseHandle(h_);
#else
    ::close(fd_);
#endif
    opened_ = false;
  }

  bool is_open() const { return opened_; }

  int read(uint8_t* dst, int maxlen){
#ifdef _WIN32
    if (!opened_) return -1;
    DWORD got = 0;
    if (!ReadFile(h_, dst, (DWORD)maxlen, &got, NULL)) return -1;
    return (int)got;
#else
    if (!opened_) return -1;
    int n = ::read(fd_, dst, maxlen);
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return 0;
    return n;
#endif
  }

  std::string port_name() const { return name_; }
  int baudrate() const { return baud_; }

private:
  bool opened_;
  std::string name_;
  int baud_{0};
#ifdef _WIN32
  HANDLE h_{INVALID_HANDLE_VALUE};
#else
  int fd_{-1};
#endif
};

// ---------- arg parsing ----------
struct Args {
  std::string port;
  int baud = 921600;
  bool list_only = false;
};
static Args parse_args(int argc, char** argv){
#ifdef _WIN32
  Args a{ "COM3", 921600, false };          // 默认值（Windows）
#else
  Args a{ "/dev/ttyACM0", 921600, false };  // 默认值（Linux）
#endif
  for (int i=1;i<argc;i++){
    std::string s = argv[i];
    if (s == "--port" && i+1<argc) a.port = argv[++i];     // 可传 "auto"
    else if (s == "--baud" && i+1<argc) a.baud = std::stoi(argv[++i]);
    else if (s == "--list") a.list_only = true;
    else if (s == "--help" || s == "-h"){
      std::cout << "Usage: " << argv[0] << " [--port PORT|auto] [--baud BAUD] [--list]\n"
#ifdef _WIN32
                << "Defaults: --port COM3  --baud 921600\n";
#else
                << "Defaults: --port /dev/ttyACM0  --baud 921600\n";
#endif
      std::exit(0);
    }
  }
  return a;
}

// ---------- packet processing ----------
static bool process_packet73(const uint8_t* pkt){
  if (pkt[0] != 0xAA || pkt[1] != 0x55) { std::cerr << "\x1b[31mInvalid header\x1b[0m\n"; return false; }
  uint8_t len = pkt[2];
  if (len != 0x44){ std::cerr << "\x1b[31mInvalid length: " << (int)len << "\x1b[0m\n"; return false; }

  uint16_t calc = crc16_modbus(pkt+2, 69);
  uint16_t recv = (uint16_t(pkt[72])<<8) | uint16_t(pkt[71]);
  if (calc != recv){
    std::cerr << "\x1b[31mCRC mismatch recv=" << std::hex << std::uppercase
              << std::setw(4) << std::setfill('0') << recv
              << " calc=" << std::setw(4) << calc << "\x1b[0m\n" << std::dec;
    return false;
  }

  const uint8_t* d = pkt + 7;
  uint32_t ts_us   = rd_le<uint32_t>(d + 0);
  double   ts_sec  = ts_us / 1e6;
  uint16_t sys     = rd_le<uint16_t>(d + 4) & 0x07;
  double roll  = rd_le<int16_t >(d + 6)  * 0.01;
  double pitch = rd_le<int16_t >(d + 8)  * 0.01;
  double yaw   = rd_le<uint16_t>(d +10)  * 0.01;

  double q4 = rd_le<int32_t>(d +12) * 1e-7;
  double q1 = rd_le<int32_t>(d +16) * 1e-7;
  double q2 = rd_le<int32_t>(d +20) * 1e-7;
  double q3 = rd_le<int32_t>(d +24) * 1e-7;

  double gx = rd_le<int32_t>(d +28) * 1e-5;
  double gy = rd_le<int32_t>(d +32) * 1e-5;
  double gz = rd_le<int32_t>(d +36) * 1e-5;

  double ax_g = rd_le<int32_t>(d +40) * 1e-5;
  double ay_g = rd_le<int32_t>(d +44) * 1e-5;
  double az_g = rd_le<int32_t>(d +48) * 1e-5;

  double mx = rd_le<int16_t>(d +52) * 1e-3;
  double my = rd_le<int16_t>(d +54) * 1e-3;
  double mz = rd_le<int16_t>(d +56) * 1e-3;

  int8_t  temperature = (int8_t) *(d +58);
  uint8_t upd_raw     = (uint8_t) *(d +59);
  double  upd_hz      = upd_raw * 10.0;

  std::ostringstream os;
  os.setf(std::ios::fixed);
  os << std::setprecision(6)
     << "\nTimestamp : " << ts_sec << " s\n"
     << "System Status: " << sys
     << " | Temperature: " << std::setprecision(2) << (double)temperature << "℃"
     << " | Update Frequency: " << upd_hz << " Hz\n"
     << "\nEuler angle(deg):\n"
     << "    x-axis : " << roll << "\n"
     << "    y-axis : " << pitch << "\n"
     << "    z-axis : " << yaw << "\n"
     << "\nQuaternion:\n"
     << "    qx : " << q1 << "\n"
     << "    qy : " << q2 << "\n"
     << "    qz : " << q3 << "\n"
     << "    qw : " << q4 << "\n"
     << "\nAngular velocity (rad/s):\n"
     << "    x-axis : " << gx << "\n"
     << "    y-axis : " << gy << "\n"
     << "    z-axis : " << gz << "\n"
     << "\nAcceleration (m/s^2):\n"
     << "    x-axis : " << (ax_g * 9.8) << "\n"
     << "    y-axis : " << (ay_g * 9.8) << "\n"
     << "    z-axis : " << (az_g * 9.8) << "\n"
     << "\nMagnetic field (uT):\n"
     << "    x-axis : " << mx << "\n"
     << "    y-axis : " << my << "\n"
     << "    z-axis : " << mz << "\n"
     << "------\n";
  std::cout << os.str() << std::flush;
  return true;
}

// ---------- main ----------
int main(int argc, char** argv){
  auto args = parse_args(argc, argv);

  if (args.list_only){
    auto ports = list_candidate_ports();
    std::cout << "Detected ports (" << ports.size() << "):\n";
    for (auto& p: ports) std::cout << "  " << p << "\n";
    return 0;
  }

  {
    auto ports = list_candidate_ports();
    std::cout << "There are currently " << ports.size() << " serial port devices connected:\n";
    for (auto& p: ports) std::cout << "  " << p << "\n";
  }

  auto prefer = args.port;   // may be "auto" or a concrete name
  auto baud   = args.baud;

  auto try_open = [&](SerialPort& sp)->bool{
    auto cand = list_candidate_ports();

    // 支持 "--port auto"：自动选第一个
    if (prefer == "auto") {
      if (cand.empty()) {
        std::cout << "\x1b[33mNo serial ports found. Retry in 1s...\x1b[0m\n";
        return false;
      }
      prefer = cand.front();
      std::cout << "\x1b[36mAuto-select port:\x1b[0m " << prefer << "\n";
    }

    bool found = std::find(cand.begin(), cand.end(), prefer) != cand.end();
    if (!found){
      std::cout << "\x1b[33mWait for the specified port to appear: " << prefer
                << " (retry after 1s)\x1b[0m\n";
      return false;
    }
    if (!sp.open(prefer, baud)){
      std::cout << "\x1b[31mFailed to open serial port: " << prefer << "\x1b[0m\n";
      return false;
    }
    std::cout << "\x1b[32mSerial port successfully opened\x1b[0m (" << sp.port_name()
              << " @ " << sp.baudrate() << ")\n";
    return true;
  };

  SerialPort sp;
  while (!try_open(sp)) msleep(1000);

  const int PACKET_SIZE = 73;
  std::vector<uint8_t> buffer; buffer.reserve(4096);
  auto last_rx = std::chrono::steady_clock::now();

  while (true){
    uint8_t tmp[1024];
    int n = sp.read(tmp, sizeof(tmp));
    if (n < 0){
      std::cerr << "\x1b[31mSerial read error. Reconnecting...\x1b[0m\n";
      sp.close();
      do { msleep(1000); } while(!try_open(sp));
      buffer.clear();
      continue;
    }
    if (n > 0){
      buffer.insert(buffer.end(), tmp, tmp+n);
      last_rx = std::chrono::steady_clock::now();
    } else {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    auto now = std::chrono::steady_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_rx).count();
    if (ms > 2000){
      std::cerr << "\x1b[33mNo data for " << ms << " ms. Reconnecting...\x1b[0m\n";
      sp.close();
      do { msleep(1000); } while(!try_open(sp));
      buffer.clear();
      last_rx = std::chrono::steady_clock::now();
      continue;
    }

    while ((int)buffer.size() >= PACKET_SIZE){
      int start = -1;
      for (size_t i=0; i+1<buffer.size(); ++i){
        if (buffer[i]==0xAA && buffer[i+1]==0x55){ start = (int)i; break; }
      }
      if (start < 0){ buffer.clear(); break; }
      if (start > 0) buffer.erase(buffer.begin(), buffer.begin()+start);
      if ((int)buffer.size() < PACKET_SIZE) break;

      uint8_t pkt[PACKET_SIZE];
      std::memcpy(pkt, buffer.data(), PACKET_SIZE);
      buffer.erase(buffer.begin(), buffer.begin()+PACKET_SIZE);

      (void)process_packet73(pkt);
    }
  }
  return 0;
}
