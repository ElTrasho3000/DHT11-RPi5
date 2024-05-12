/*
Code making use of gpiod-library to read 
data from DHT11-sensor on a RaspberryPi5.

This code is largely based on:
https://github.com/adafruit/DHT-sensor-library/blob/master/DHT.cpp#L36
Adafruit published their library under MIT License.

This adaption is licensed GPLv3.
*/

#include <iostream>
#include <gpiod.hpp>
#include <chrono>
#include <thread>

// Uncomment for debug messages
//#define _DEBUG

// Define _DEBUG_PRINT
#ifdef _DEBUG
#define _DEBUG_PRINT(msg) std::cout << msg << std::endl
#define _DEBUG_RELEASE() gpiod_line_release(data_line);
#define _DEBUG_CLOSE() gpiod_chip_close(chip);
#else
#define _DEBUG_PRINT(msg)
#define _DEBUG_RELEASE()
#define _DEBUG_CLOSE()
#endif

// namespace declaration
using namespace std::chrono;

//gpiod variables
// GPIO connected to DHT11 data pin
const uint8_t data_pin = 26;
// Variable to store GPIO chip (pinctrl-RP1 = gpiochip4)
const char *chipname = "gpiochip4";
// Create instance of gpiod_chip
struct gpiod_chip *chip;
// Create instance of gpiod_line
struct gpiod_line *data_line;

// Array to store DHT11 data
std::uint8_t data[5];

// Variables
uint32_t _lastreadtime;
bool _lastresult;

// Function to map 1millisecond(ms) in microseconds(us) to clock ticks
uint64_t microsecondsToClockCycles(uint64_t ms) {
    const time_point<high_resolution_clock> start = high_resolution_clock::now();
    std::this_thread::sleep_for(microseconds(ms));
    uint64_t us_hrc = duration_cast<microseconds>(high_resolution_clock::now()-start).count();
    _DEBUG_PRINT(us_hrc);
    return us_hrc;
}

// Count of cycles till timeout
uint32_t _maxcycles = microsecondsToClockCycles(1000);

// Used programmatically for timeout.                           \
// Not a timeout duration. Type: uint32_t
#define _TIMEOUT UINT32_MAX 

// Function to measure duration of high(1) and low(0) signal
uint32_t expectPulse(bool level) {
    uint32_t count = 0;
    while (gpiod_line_get_value(data_line) == level) {
        if (count++ >= _maxcycles) {
        _DEBUG_PRINT("PULSE TIMEOUT");
        return _TIMEOUT; // Exceeded timeout, fail.
        }
    }
    return count;
}

// Function to read DHT data into data-array
bool readDHT() {
    // Open GPIO chip
    chip = gpiod_chip_open_by_name(chipname);
    // Open GPIO line
    data_line = gpiod_chip_get_line(chip, data_pin);
    //Data Single-bus free status is at high voltage level
    gpiod_line_request_input_flags(data_line, "DHT-data", GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP);
    _DEBUG_PRINT("data_line:");
    _DEBUG_PRINT(gpiod_line_get_value(data_line));
    /*
     When the communication between MCU and DHT11 begins, 
     the programme of MCU will set Data Single-bus voltage 
     level from high to low and this process must take at 
     least 18ms to ensure DHT's detection of MCU's signal, 
     then MCU will pull up voltage and wait 20-40us for 
     DHT's response.
    */
    // START SIGNAL
    // Release gpiod_line to change mode
    gpiod_line_release(data_line);
    // Set data_line from high to low
    gpiod_line_request_output(data_line, "DHT-data", 0);
    _DEBUG_PRINT("data_line:");
    _DEBUG_PRINT(gpiod_line_get_value(data_line));
    // Keep low signal for at least 18ms
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    // Release gpiod_line to change mode
    gpiod_line_release(data_line);


    // Use internal pull-up to set data_line from low to high
    gpiod_line_request_input_flags(data_line, "DHT-data", GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP);
    _DEBUG_PRINT("data_line:");
    _DEBUG_PRINT(gpiod_line_get_value(data_line));
    // Release gpiod_line to change mode
    gpiod_line_release(data_line);

    // Set data_line to input
    gpiod_line_request_input(data_line, "DHT-data");
    _DEBUG_PRINT("data_line:");
    _DEBUG_PRINT(gpiod_line_get_value(data_line));    
    // Wait for DHT's response
    if (expectPulse(0) == _TIMEOUT) {
      _DEBUG_PRINT("DHT timeout waiting for start signal low pulse.");
      _lastresult = false;
      return _lastresult;
    }

    if (expectPulse(1) == _TIMEOUT) {
      _DEBUG_PRINT("DHT timeout waiting for start signal high pulse.");
      _lastresult = false;
      return _lastresult;
    }

    // READ DATA
    // Release data_line to change mode
    gpiod_line_release(data_line);
    // Set data_line to input with pull-down
    gpiod_line_request_input(data_line, "DHT-data");
    // When DHT is sending data to MCU, every bit of 
    // data begins with the 50us low-voltage-level and
    // the length of the following high-voltage-level
    // signal determines whether data bit is "0" or "1"

    // Reset 40 bits of received data to zero.
    data[0] = data[1] = data[2] = data[3] = data[4] = 0;

    // Array to store 80-signal bits from DHT
    uint32_t cycles[80];

    // Assign signal bits from DHT to array
    for (int i = 0; i < 80; i += 2) {
      cycles[i] = expectPulse(0);
      cycles[i + 1] = expectPulse(1);
    }

    // Release data_line and close chip
    gpiod_line_release(data_line);
    gpiod_chip_close(chip);
    
    // Timing critical code is now complete.

    // Inspect pulses and determine which ones are 0 (high state cycle count < low
    // state cycle count), or 1 (high state cycle count > low state cycle count).
    for (int i = 0; i < 40; ++i) {
        uint64_t lowCycles = cycles[2 * i];
        uint64_t highCycles = cycles[2 * i + 1];
        if ((lowCycles == _TIMEOUT) || (highCycles == _TIMEOUT)) {
        _DEBUG_PRINT("DHT timeout waiting for pulse.");
        _lastresult = false;
        return _lastresult;
        }
        data[i / 8] <<= 1;
        // Now compare the low and high cycle times to see if the bit is a 0 or 1.
        if (highCycles > lowCycles) {
        // High cycles are greater than 50us low cycle count, must be a 1.
        data[i / 8] |= 1;
        }
        // Else high cycles are less than (or equal to, a weird case) the 50us low
        // cycle count so this must be a zero.  Nothing needs to be changed in the
        // stored data.
    }

    /* Uncomment for additional DEBUG info
    // Prints data in binary
    for (uint8_t i = 0; i < 5; i++) {
        _DEBUG_PRINT(std::bitset<8> (data[i]));
    }
    */

    // Check we read 40 bits and that the checksum matches.
    if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
        _lastresult = true;
        return _lastresult;
    }
    else {
        _DEBUG_PRINT("DHT checksum failure!");
        _lastresult = false;
        return _lastresult;
    }
    _DEBUG_PRINT(_lastresult);
}

// Function to read temperature in degrees celcius
float readTemperature() {
    float t;
    t = data[2];
    if (data[3] & 0x80) {
        t = -1 - t;
    }
    t += (data[3] & 0x0f) * 0.1;
    return t;
}

// Function to read relative humidity
float readHumidity() {
    float rh;
    rh = data[0] + data[1] * 0.1;
    return rh;
}

int main() {
    readDHT();
    float temp = readTemperature();
    std::cout << temp << "°C" << std::endl;
    float relHumidity = readHumidity();
    std::cout << relHumidity << "%" << std::endl;
}