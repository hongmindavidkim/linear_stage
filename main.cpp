#include "mbed.h"
#include "crc.h"
#include "string.h"
#include "dynamixel_XM430.h"
#include "math.h"
#include <cmath>
#include <cstdint>
#include "actuator_transformation.h"

uint8_t state = 0; // Operating State

// Initialize stuff for dynamixels
#define WAIT_TIME_MS 1
#define LEN 100 
#define wait_ms(x) wait_us(x*1000)
#define rad2pulse_t(x) uint32_t(rad2pulse(x))
#define deg2rad(x) float((PI/180.0f)*x)
#define pulse2deg(x) (360.0f/4096.0f)*(float)(x-2048.0f)

RawSerial uart(D1, D0);
DigitalInOut RTS(D2);
volatile uint8_t waitForReceive = 0;
volatile uint8_t nextReload = 15;
uint8_t rx_buffer[LEN];

uint8_t dxl_ID[] =  {1};
uint8_t idLength = sizeof(dxl_ID) / sizeof(dxl_ID[0]);

// Initialize serial port
RawSerial pc(USBTX, USBRX, 921600);
Timer t;

int32_t dxl_time;
// Initial Positions

uint32_t HomePos = 2048;

uint32_t rtPos;
uint32_t goalPos;
float currentPos;
float currentVel;
float currentCur;

double current_limit = 1000;

int32_t dxl_position[2];
int32_t dxl_velocity[2];
int16_t dxl_current[2];

XM430_bus dxl_bus(4000000, D1, D0, D2); // baud, tx, rx, rts

int main() {

    for (int i=0; i<idLength; i++) {
        dxl_bus.SetTorqueEn(dxl_ID[i],0x00);
        dxl_bus.SetRetDelTime(dxl_ID[i],0x05); // 4us delay time?
        dxl_bus.SetControlMode(dxl_ID[i], POSITION_CONTROL);
        wait_ms(100);
        dxl_bus.TurnOnLED(dxl_ID[i], 0x01);
        //dxl_bus.TurnOnLED(dxl_ID[i], 0x00); // turn off LED
        dxl_bus.SetTorqueEn(dxl_ID[i],0x01); //to be able to move 
        wait_ms(100);
        }
    for (int i=0; i<idLength; i++) {
        dxl_bus.SetVelocityProfile(dxl_ID[i], 0); // 414(94.81RPM) @ 14.8V, 330(75.57RPM) @ 12V
        dxl_bus.SetAccelerationProfile(dxl_ID[i], 0); // 80(17166) rev/min^2
        }

    dxl_bus.SetGoalPosition(dxl_ID[0], HomePos);
  
    t.reset();
    t.start();
    while (true) {

    }
    t.stop();
    dxl_time = t.read_ms();


}

