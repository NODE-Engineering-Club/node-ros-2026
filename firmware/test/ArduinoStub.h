// Minimal Arduino stub, enough to type-check the sketch on a host compiler.
// It is NOT an emulator: Serial output goes to a queue the harness inspects,
// and the SBUS UART is fed from a byte queue.
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <string>
#include <deque>
#include <vector>
#include <algorithm>

typedef uint8_t byte;
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define SERIAL_8E2 0

extern unsigned long g_millis;
// setup() contains a 900 ms busy-wait (startup_light_sequence), so the clock
// must advance on its own there. During the test body we drive it by hand.
extern bool g_auto_ms;
inline unsigned long millis() { if (g_auto_ms) g_millis++; return g_millis; }

extern std::vector<int> g_pin_state;
inline void pinMode(int, int) {}
inline void digitalWrite(int pin, int v) { if ((int)g_pin_state.size() <= pin) g_pin_state.resize(pin + 1, 0); g_pin_state[pin] = v; }
inline int  digitalRead(int pin) { return ((int)g_pin_state.size() > pin) ? g_pin_state[pin] : 0; }

template <typename T> T constrain(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }
using std::max;
using std::fabs;

#define F(x) (x)

class String {
 public:
  std::string s;
  String() {}
  String(const char* p) : s(p ? p : "") {}
  String(const String& o) : s(o.s) {}
  String& operator=(const String& o) { s = o.s; return *this; }
  void trim() {
    size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) { s.clear(); return; }
    size_t e = s.find_last_not_of(" \t\r\n");
    s = s.substr(b, e - b + 1);
  }
  void replace(char a, char b) { for (auto& c : s) if (c == a) c = b; }
  void toUpperCase() { for (auto& c : s) c = toupper((unsigned char)c); }
  size_t length() const { return s.size(); }
  const char* c_str() const { return s.c_str(); }
  bool operator==(const char* o) const { return s == o; }
  bool operator==(const String& o) const { return s == o.s; }
};

class SerialStub {
 public:
  std::string out;            // everything printed
  std::deque<char> in;        // everything the host "sent"
  void begin(unsigned long) {}
  void begin(unsigned long, int) {}
  int  available() { return (int)in.size(); }
  int  read() { if (in.empty()) return -1; char c = in.front(); in.pop_front(); return (unsigned char)c; }
  void feed(const std::string& d) { for (char c : d) in.push_back(c); }

  void print(const char* v) { out += v; }
  void print(const String& v) { out += v.s; }
  void print(char v) { out += v; }
  void print(int v) { char b[32]; snprintf(b, 32, "%d", v); out += b; }
  void print(unsigned long v) { char b[32]; snprintf(b, 32, "%lu", v); out += b; }
  void println() { out += "\n"; }
  template <typename T> void println(T v) { print(v); out += "\n"; }
  void println(const char* v) { out += v; out += "\n"; }

  String readStringUntil(char term) {
    String r; 
    while (!in.empty()) { char c = in.front(); in.pop_front(); if (c == term) break; r.s += c; }
    return r;
  }
  void setRX(int) {} void setTX(int) {} void setInvertRX(bool) {}
};
extern SerialStub Serial;
extern SerialStub Serial2;

struct TinyUSBDeviceStub { bool mounted() { return true; } };
extern TinyUSBDeviceStub TinyUSBDevice;

class Servo {
 public:
  int last = 1500;
  void attach(int) {}
  void attach(int, int, int) {}
  void writeMicroseconds(int us) { last = us; }
};
