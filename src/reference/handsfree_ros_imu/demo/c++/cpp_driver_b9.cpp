/****************************************************
 * File: cpp_driver_b9.cpp
 * Desc: B9 IMU serial reader & parser (C++17, cross-platform)
 *
 * Build (Linux):
 *   g++ -std=c++17 cpp_driver_b9.cpp -o cpp_driver_b9 -lboost_system -lpthread
 *
 * Build (Windows / MSVC):
 *   cl /EHsc /std:c++17 cpp_driver_b9.cpp /I"path\\to\\boost" /link /LIBPATH:"path\\to\\boost\\lib" ws2_32.lib
 *
 * Run:
 *   ./cpp_driver_b9                 # Linux defaults: /dev/ttyUSB0 @ 921600
 *   ./cpp_driver_b9 --port /dev/ttyUSB0 --baud 921600
 *   cpp_driver_b9.exe               # Windows defaults: COM3 @ 921600
 *   cpp_driver_b9.exe --port COM5 --baud 921600
 *
 * Args:
 *   --port <device>   e.g., /dev/ttyUSB0 or COM5 (COM>=10 auto-handled)
 *   --baud <rate>     default: 921600
 ****************************************************/

#include <iostream>
#include <vector>
#include <string>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <memory>
#include <chrono>
#include <thread>
#include <iomanip>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>

#ifdef _WIN32
  #include <windows.h>
#endif

// Use io_service alias for compatibility with older Boost
using Io = boost::asio::io_service;
using boost::asio::serial_port;
using boost::system::error_code;
using namespace std::chrono_literals;

// ---------------- State (mirrors Python) ----------------
static std::vector<uint8_t> g_buf;
static int  g_idx = 0;                 // byte index inside a frame
static bool g_need_header = true;

static double g_temperature_c = 0.0;
static float  g_accel[3] = {0,0,0};    // m/s^2
static float  g_gyro[3]  = {0,0,0};    // rad/s
static float  g_euler[3] = {0,0,0};    // deg
static float  g_mag[3]   = {0,0,0};    // raw int16 -> float

// flags: 0x51,0x52,0x53,0x54 arrived
static bool g_have_51 = false;
static bool g_have_52 = false;
static bool g_have_53 = false;
static bool g_have_54 = false;

// ---------------- Utils ----------------
static inline int16_t le_i16(const uint8_t* p) {
    // little-endian int16
    return int16_t( (uint16_t)p[0] | (uint16_t(p[1])<<8) );
}

static inline bool checksum_ok(const uint8_t* frame11) {
    uint32_t s = 0;
    for (int i=0;i<10;++i) s += frame11[i];
    return (uint8_t(s & 0xFF) == frame11[10]);
}

static void reset_frame() {
    g_buf.clear();
    g_idx = 0;
    g_need_header = true;
}

static void print_if_ready() {
    if (!(g_have_51 && g_have_52 && g_have_53 && g_have_54)) return;
    g_have_51 = g_have_52 = g_have_53 = g_have_54 = false;

    std::cout << std::fixed;
    std::cout << "\nSystem Status: Temperature: " << std::setprecision(2) << g_temperature_c << " ℃\n\n";

    std::cout << "Acceleration (m/s^2):\n";
    std::cout << "  x: " << std::setprecision(2) << g_accel[0] << "\n";
    std::cout << "  y: " << std::setprecision(2) << g_accel[1] << "\n";
    std::cout << "  z: " << std::setprecision(2) << g_accel[2] << "\n\n";

    std::cout << "Angular velocity (rad/s):\n";
    std::cout << "  x: " << std::setprecision(2) << g_gyro[0] << "\n";
    std::cout << "  y: " << std::setprecision(2) << g_gyro[1] << "\n";
    std::cout << "  z: " << std::setprecision(2) << g_gyro[2] << "\n\n";

    std::cout << "Euler angle (deg):\n";
    std::cout << "  roll:  " << std::setprecision(2) << g_euler[0] << "\n";
    std::cout << "  pitch: " << std::setprecision(2) << g_euler[1] << "\n";
    std::cout << "  yaw:   " << std::setprecision(2) << g_euler[2] << "\n\n";

    std::cout << "Magnetic field:\n";
    std::cout << "  x: " << std::setprecision(2) << g_mag[0] << "\n";
    std::cout << "  y: " << std::setprecision(2) << g_mag[1] << "\n";
    std::cout << "  z: " << std::setprecision(2) << g_mag[2] << "\n";
    std::cout << "------\n" << std::endl;
}

// ---------------- Parser (state machine) ----------------
// Accumulate bytes; when 11 bytes frame is ready, parse
static void feed_byte(uint8_t b) {
    if (g_need_header) {
        if (b != 0x55) return;  // wait header
        g_buf.clear();
        g_buf.push_back(b);
        g_idx = 1;
        g_need_header = false;
        return;
    }

    // accumulating
    g_buf.push_back(b);
    ++g_idx;

    if (g_idx < 11) return; // wait full frame

    // got full 11 bytes
    uint8_t* f = g_buf.data(); // [0..10]
    if (!checksum_ok(f)) {
        // optional: std::cout << "Checksum failed (TYPE=0x" << std::hex << int(f[1]) << std::dec << ")\n";
        reset_frame();
        return;
    }

    uint8_t type = f[1];
    const uint8_t* p = &f[2]; // payload 8 bytes: 4 * int16 (LE)

    switch (type) {
        case 0x51: { // accel + temperature
            int16_t ax = le_i16(p+0);
            int16_t ay = le_i16(p+2);
            int16_t az = le_i16(p+4);
            int16_t tp = le_i16(p+6);
            g_accel[0] = float(ax) / 32768.0f * 16.0f * (-9.8f);
            g_accel[1] = float(ay) / 32768.0f * 16.0f * (-9.8f);
            g_accel[2] = float(az) / 32768.0f * 16.0f * (-9.8f);
            g_temperature_c = double(tp) / 100.0;
            g_have_51 = true;
            break;
        }
        case 0x52: { // gyro
            int16_t gx = le_i16(p+0);
            int16_t gy = le_i16(p+2);
            int16_t gz = le_i16(p+4);
            constexpr double k = (2000.0 * M_PI / 180.0) / 32768.0; // rad/s per LSB
            g_gyro[0] = float(gx * k);
            g_gyro[1] = float(gy * k);
            g_gyro[2] = float(gz * k);
            g_have_52 = true;
            break;
        }
        case 0x53: { // euler
            int16_t r = le_i16(p+0);
            int16_t pt= le_i16(p+2);
            int16_t yw= le_i16(p+4);
            constexpr double k = 180.0 / 32768.0; // deg per LSB
            g_euler[0] = float(r  * k);
            g_euler[1] = float(pt * k);
            g_euler[2] = float(yw * k);
            g_have_53 = true;
            break;
        }
        case 0x54: { // mag (raw)
            int16_t mx = le_i16(p+0);
            int16_t my = le_i16(p+2);
            int16_t mz = le_i16(p+4);
            g_mag[0] = float(mx);
            g_mag[1] = float(my);
            g_mag[2] = float(mz);
            g_have_54 = true;
            break;
        }
        default:
            std::cout << "Unsupported TYPE: 0x" << std::hex << int(type) << std::dec << "\n";
            break;
    }

    reset_frame();
    print_if_ready();
}

// ---------------- Port defaults & helpers ----------------
static std::string default_port() {
#ifdef _WIN32
    return "COM3";            // common default on Windows
#else
    return "/dev/ttyUSB0";    // or "/dev/ttyUSB0" per your device
#endif
}

#ifdef _WIN32
static std::string normalize_win_port(std::string p) {
    // If COM>=10 and doesn't have "\\.\", add it.
    if (p.rfind("\\\\.\\", 0) == 0) return p;
    if (p.rfind("COM", 0) == 0) {
        try {
            int n = std::stoi(p.substr(3));
            if (n >= 10) return std::string("\\\\.\\") + p;
        } catch (...) {}
    }
    return p;
}
#endif

// ---------------- Serial Open / Reconnect ----------------
static std::unique_ptr<serial_port> open_serial_with_retry(Io& ctx,
                                                           const std::string& port_in,
                                                           unsigned baud,
                                                           double retry_delay_sec=1.0)
{
    std::string port_name = port_in.empty() ? default_port() : port_in;
#ifdef _WIN32
    port_name = normalize_win_port(port_name);
#endif
    while (true) {
        try {
            auto sp = std::make_unique<serial_port>(ctx, port_name);
            sp->set_option(serial_port::baud_rate(baud));
            sp->set_option(boost::asio::serial_port_base::character_size(8));
            sp->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
            sp->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
            sp->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

            std::cout << "Serial port opened: " << port_name << " @ " << baud << "\n";
            return sp;
        } catch (const std::exception& e) {
            std::cout << "Failed to open serial port (" << port_name << "): " << e.what()
                      << ". Retrying in " << retry_delay_sec << " s...\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(int(retry_delay_sec*1000)));
        }
    }
}

static std::unique_ptr<serial_port> reconnect(Io& ctx,
                                              std::unique_ptr<serial_port>& old_sp,
                                              const std::string& port_name,
                                              unsigned baud,
                                              double retry_delay_sec=1.0)
{
    try { if (old_sp && old_sp->is_open()){ error_code ec; old_sp->close(ec);} } catch(...) {}
    std::cout << "Serial port disconnected or I/O error. Reconnecting...\n";
    return open_serial_with_retry(ctx, port_name, baud, retry_delay_sec);
}

// ---------------- Main ----------------
int main(int argc, char** argv) {
    // Defaults: platform-specific port + 921600 baud
    std::string port_arg = default_port();
    unsigned baud = 921600;

    for (int i=1;i<argc;++i) {
        std::string a = argv[i];
        if      (a=="--port" && i+1<argc) port_arg = argv[++i];
        else if (a=="--baud" && i+1<argc) baud = unsigned(std::stoul(argv[++i]));
        else if (a=="--help" || a=="-h") {
            std::cout<<"Usage:\n  "<<argv[0]<<" [--port <device>] [--baud <rate>]\n"
                     <<"Defaults:\n  --port "<<default_port()<<"   --baud 921600\n"
#ifdef _WIN32
                     <<"Note: COM>=10 auto-handled (e.g., COM12 -> \\\\.\\COM12)\n"
#endif
                     <<"Examples:\n  "<<argv[0]<<" --port /dev/ttyUSB0 --baud 921600\n  "
                     <<argv[0]<<" --port COM5 --baud 921600\n";
            return 0;
        }
    }

    Io ctx;
    auto sp = open_serial_with_retry(ctx, port_arg, baud, 1.0);

    std::vector<uint8_t> readbuf(256);
    reset_frame();

    while (true) {
        try {
            if (!sp || !sp->is_open()) {
                sp = reconnect(ctx, sp, port_arg, baud, 1.0);
                continue;
            }
            error_code ec;
            size_t n = sp->read_some(boost::asio::buffer(readbuf), ec);
            if (ec) {
                std::cout<<"exception: "<<ec.message()<<"\n";
                sp = reconnect(ctx, sp, port_arg, baud, 1.0);
                continue;
            }
            if (n>0) {
                for (size_t i=0;i<n;++i) feed_byte(readbuf[i]);
            } else {
                std::this_thread::sleep_for(100us);
            }
        } catch (const std::exception& e) {
            std::cout<<"exception: "<<e.what()<<"\n";
            sp = reconnect(ctx, sp, port_arg, baud, 1.0);
        }
    }
    return 0;
}
