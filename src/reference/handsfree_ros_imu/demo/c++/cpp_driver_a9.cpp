/****************************************************
 * File: cpp_driver_a9.cpp
 * Desc: A9 IMU serial reader & parser (cross-platform C++17)
 * Dep:  Boost.Asio (+ libboost_system)
 *
 * Linux build:
 *   g++ -std=c++17 cpp_driver_a9.cpp -o cpp_driver_a9 -lboost_system -lpthread
 *
 * Windows (MSVC) build:
 *   cl /EHsc /std:c++17 cpp_driver_a9.cpp /I"path\to\boost" /link /LIBPATH:"path\to\boost\lib" ws2_32.lib
 *
 * Run examples:
 *   ./cpp_driver_a9             # uses defaults: /dev/ttyUSB0, 921600
 *   ./cpp_driver_a9 --port /dev/ttyUSB0 --baud 921600
 *   cpp_driver_a9.exe           # uses defaults: COM3, 921600
 *   cpp_driver_a9.exe --port COM5 --baud 921600
 *
 * Args:
 *   --port <device>   (REQUIRED) e.g., /dev/ttyUSB0 or COM5
 *   --baud <rate>     default: 921600
 ****************************************************/

#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <cstdint>
#include <chrono>
#include <thread>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <memory>
#include <cmath>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>

#ifdef _WIN32
  #include <windows.h>
#endif

// Compatibility: use io_service alias to support older Boost where io_context may not exist.
using Io = boost::asio::io_service;
using boost::asio::serial_port;
using boost::system::error_code;
using namespace std::chrono_literals;

// --------------------- Global State (same as Python) ---------------------
static std::vector<uint8_t> g_buff;
static int    g_key = 0;
static double g_timestamp = 0.0;
static float  g_angularVelocity[3] = {0,0,0};
static float  g_acceleration[3]    = {0,0,0};
static float  g_magnetometer[3]    = {0,0,0};
static float  g_angle_degree[3]    = {0,0,0};
static bool   g_pub_flag[2]        = {true, true}; // [0] for 0x2c, [1] for 0x14

// --------------------- Utils ---------------------
static uint16_t crc16_modbus(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i=0;i<len;++i) {
        crc ^= data[i];
        for (int j=0;j<8;++j) {
            if (crc & 1) { crc >>= 1; crc ^= 0xA001; }
            else crc >>= 1;
        }
    }
    return crc; // [low, high]
}

static std::vector<float> parse_floats_little_endian(const std::vector<uint8_t>& bytes) {
    std::vector<float> out;
    if (bytes.size()%4) return out;
    out.reserve(bytes.size()/4);
    for (size_t i=0;i<bytes.size(); i+=4) {
        // 小端: 低字节在前
        uint32_t u = (uint32_t(bytes[i])      ) |
                     (uint32_t(bytes[i+1])<< 8) |
                     (uint32_t(bytes[i+2])<<16) |
                     (uint32_t(bytes[i+3])<<24);
        float f; std::memcpy(&f, &u, sizeof(float));
        out.push_back(f);
    }
    return out;
}

static double norm3(const float v[3]) {
    return std::sqrt(double(v[0])*v[0] + double(v[1])*v[1] + double(v[2])*v[2]);
}

// --------------------- Parser ---------------------
static void reset_buffer(){ g_buff.clear(); g_key=0; }

static void flush_and_print_if_ready() {
    if (g_pub_flag[0] || g_pub_flag[1]) return;
    g_pub_flag[0] = g_pub_flag[1] = true;

    double acc_k = norm3(g_acceleration); if (acc_k==0) acc_k=1.0;

    std::cout<<std::fixed<<std::setprecision(6);
    std::cout<<"\nTimestamp: "<<g_timestamp<<" s\n\n";

    std::cout<<"Acceleration (m/s^2):\n";
    std::cout<<"  x: "<<std::setprecision(2)<< (g_acceleration[0]*9.8/acc_k) <<"\n";
    std::cout<<"  y: "<<std::setprecision(2)<< (g_acceleration[1]*9.8/acc_k) <<"\n";
    std::cout<<"  z: "<<std::setprecision(2)<< (g_acceleration[2]*9.8/acc_k) <<"\n\n";

    std::cout<<"Angular velocity (rad/s):\n";
    std::cout<<"  x: "<<std::setprecision(2)<< g_angularVelocity[0] <<"\n";
    std::cout<<"  y: "<<std::setprecision(2)<< g_angularVelocity[1] <<"\n";
    std::cout<<"  z: "<<std::setprecision(2)<< g_angularVelocity[2] <<"\n\n";

    std::cout<<"Euler angle (deg):\n";
    std::cout<<"  roll:  "<<std::setprecision(2)<< g_angle_degree[0] <<"\n";
    std::cout<<"  pitch: "<<std::setprecision(2)<< g_angle_degree[1] <<"\n";
    std::cout<<"  yaw:   "<<std::setprecision(2)<< g_angle_degree[2] <<"\n\n";

    std::cout<<"Magnetic field:\n";
    std::cout<<"  x: "<<std::setprecision(2)<< g_magnetometer[0] <<"\n";
    std::cout<<"  y: "<<std::setprecision(2)<< g_magnetometer[1] <<"\n";
    std::cout<<"  z: "<<std::setprecision(2)<< g_magnetometer[2] <<"\n";
    std::cout<<"------\n"<<std::endl;
}

static void handle_byte(uint8_t b) {
    if (g_key==0) { if (b!=0xAA){ reset_buffer(); return; } g_buff.push_back(b); g_key=1; return; }
    if (g_key==1) { if (b!=0x55){ reset_buffer(); return; } g_buff.push_back(b); g_key=2; return; }

    g_buff.push_back(b); g_key++;
    if (g_key<3) return;

    uint8_t len = g_buff[2];
    if (g_key < int(len)+5) return;   // same as Python: key < buff[2] + 5

    const auto& data_buff = g_buff;
    if (len==0x2c && g_pub_flag[0]) {
        if (data_buff.size()>=49) {
            uint16_t crc_calc  = crc16_modbus(&data_buff[2], 45);
            uint16_t crc_frame = uint16_t(data_buff[47]) | (uint16_t(data_buff[48])<<8);
            if (crc_calc==crc_frame) {
                uint32_t ts_us = (uint32_t(data_buff[10])<<24)|(uint32_t(data_buff[9])<<16)|(uint32_t(data_buff[8])<<8)|uint32_t(data_buff[7]);
                g_timestamp = double(ts_us)/1e6;

                std::vector<uint8_t> fbytes(data_buff.begin()+7, data_buff.begin()+47);
                auto floats = parse_floats_little_endian(fbytes);
                if (floats.size()>=10) {
                    g_angularVelocity[0]=floats[1]; g_angularVelocity[1]=floats[2]; g_angularVelocity[2]=floats[3];
                    g_acceleration[0]=floats[4];    g_acceleration[1]=floats[5];    g_acceleration[2]=floats[6];
                    g_magnetometer[0]=floats[7];    g_magnetometer[1]=floats[8];    g_magnetometer[2]=floats[9];
                }
            } else std::cout<<"0x2c CRC verification failed\n";
        }
        g_pub_flag[0]=false;
    } else if (len==0x14 && g_pub_flag[1]) {
        if (data_buff.size()>=25) {
            uint16_t crc_calc  = crc16_modbus(&data_buff[2], 21);
            uint16_t crc_frame = uint16_t(data_buff[23]) | (uint16_t(data_buff[24])<<8);
            if (crc_calc==crc_frame) {
                std::vector<uint8_t> fbytes(data_buff.begin()+7, data_buff.begin()+23);
                auto floats = parse_floats_little_endian(fbytes);
                if (floats.size()>=4) {
                    g_angle_degree[0]=floats[1]; g_angle_degree[1]=floats[2]; g_angle_degree[2]=floats[3];
                }
            } else std::cout<<"0x14 CRC verification failed\n";
        }
        g_pub_flag[1]=false;
    } else {
        std::cout<<"Unsupported frame length or data error: 0x"<<std::hex<<int(len)<<std::dec<<"\n";
        reset_buffer(); return;
    }

    reset_buffer();
    flush_and_print_if_ready();
}

// --------------------- Port defaults & helpers ---------------------
static std::string default_port() {
#ifdef _WIN32
    return "COM3";            // common default on Windows
#else
    return "/dev/ttyUSB0";    // or "/dev/ttyUSB0" for your setup
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

// --------------------- Open / Reconnect (specified port only) ---------------------
static std::unique_ptr<serial_port> open_serial_with_retry(Io& ctx,
                                                           const std::string& port_name_in,
                                                           unsigned baud,
                                                           double retry_delay_sec=1.0)
{
    std::string port_name = port_name_in;
#ifdef _WIN32
    port_name = normalize_win_port(port_name);
#endif
    while (true) {
        try {
            auto sp = std::make_unique<serial_port>(ctx, port_name);
            sp->set_option(serial_port::baud_rate(baud));
            std::cout<<"Serial port opened: "<<port_name<<" @ "<<baud<<"\n";
            return sp;
        } catch (const std::exception& e) {
            std::cout<<"Failed to open serial port ("<<port_name<<"): "<<e.what()
                     <<". Retrying in "<<retry_delay_sec<<" s...\n";
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
    std::cout<<"Serial port disconnected or I/O error. Reconnecting...\n";
    return open_serial_with_retry(ctx, port_name, baud, retry_delay_sec);
}

// --------------------- Main ---------------------
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
    g_buff.reserve(256);

    std::vector<uint8_t> readbuf(1024);

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
                for (size_t i=0;i<n;++i) handle_byte(readbuf[i]);
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
