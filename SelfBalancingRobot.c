// Target Platform: EK-TM4C123GXL with LCD Interface
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// IN1: PC4
// IN2: PC5
// IN3: PC6
// IN4: PC7
// EN1: PF2   M1PWM6  1kHz
// EN2: PF3   M1PWM7  1kHz
// I2C devices on I2C bus 0 with 2kohm pullups on SDA and SCL (PB3 and PB2)

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "tm4c123gh6pm.h"
#include "gpio.h"
#include "wait.h"
#include "clock.h"
#include "uart0.h"
#include "spi1.h"
#include "i2c0.h"


#define INT1      (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4)))
#define INT2      (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4)))
#define INT3      (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))
#define INT4      (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4)))
#define CS        (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 1*4)))  //PD1

#define PIN_IN1 4 //Motor direction IN1 PC4
#define PIN_IN2 5 //Motor direction IN2 PC5
#define PIN_IN3 6 //Motor direction IN3 PC6
#define PIN_IN4 7 //Motor direction IN4 PC7
//
//#define PWM6_MASK 4// M1PWM6   PF2
//#define PWM7_MASK 8// M1PWM7   PF3


#define PWM6_MASK 1// M1PWM6   PF2
#define PWM7_MASK 2// M1PWM7   PF3

#define ACCEL_RANGE_2G 0x14  // 2g range setting
#define GYRO_RANGE_VALUE    _gyro_2000dps //2000dpm
#define ACCEL_RANGE_VALUE   _accel_4g//4g
//#define GYRO_SENS   16.4f;
//#define ACCEL_SENS = 8192.0f;
//#define MIN_WORKING_PWM = 18000
//#define MAX_PWM = 19999

//////////REGISTER////////
//#define bank_reg 0x7h;
///BANK0
#define ICM20948_ADDR        0x69
#define WHO_AM_I             0x00
#define BANK_SELECT_REG      0x7F
#define PWR_MGMT_1           0x06
#define PWR_MGMT_2           0x07


///BANK2
#define ODR_ALIGN_EN         0x09
#define GYRO_SMPLRT_DIV      0x00  // Gyroscope sample rate divider register
#define GYRO_CONFIG_1        0x01  // Gyroscope configuration register
#define ACCEL_SMPLRT_DIV_1   0x10  // Accelerometer sample rate divider
#define ACCEL_SMPLRT_DIV_2   0x11  // Accelerometer sample rate divider
#define ACCEL_CONFIG         0x14  // Register for accelerometer configuration


////Bank 0
////Sensor
#define ACCEL_XOUT_H 0x2D
#define ACCEL_XOUT_L 0x2E
#define ACCEL_YOUT_H 0x2F
#define ACCEL_YOUT_L 0x30
#define ACCEL_ZOUT_H 0x31
#define ACCEL_ZOUT_L 0x32
#define GYRO_XOUT_H  0x33
#define GYRO_XOUT_L  0x34
#define GYRO_YOUT_H  0x35
#define GYRO_YOUT_L  0x36
#define GYRO_ZOUT_H  0x37
#define GYRO_ZOUT_L  0x38




/////////////Global/////////////////////

// Global calibration offsets
volatile int16_t offset_ax = 0.0;
volatile int16_t offset_az = 0.0;
volatile int16_t offset_gy = 0.0;

// Complementary filter
float accel_pitch = 0.0;
float pitch = 0.0;  // Pitch angle (in degrees)
float alpha = 0.8;  // Weight for gyroscope (0.98 gives more weight to gyroscope)
float dt = 0.0005; // Time delta (in seconds, this depends on your loop's timing): Assuming a 100Hz update rate (10ms)

//PID
static float integral = 0;
static float previousError = 0;
float KP = 4;
float KI = 0.0005;
float KD = 0.4;

//Buffer
USER_DATA data;
char buffer[150];



///Filter
int printCounter = 0;
const uint8_t printInterval = 70;  // print every 5 loops (~50ms if dt = 10ms)

const float GYRO_SENS = 16.4f;   // LSB per degree/second
const float ACCEL_SENS = 8192.0f; // LSB per g




typedef enum
{
    _gyro_250dps,
    _gyro_500dps,
    _gyro_1000dps,
    _gyro_2000dps
} gyro_range;

typedef enum
{
    _accel_2g,
    _accel_4g,
    _accel_8g,
    _accel_16g
} accel_range;

typedef enum
{
    _b0 = 0,
    _b1 = 1 << 4,
    _b2 = 2 << 4,
    _b3 = 3 << 4
} user_bank;




void initGPIO()
{
    // Initialize GPIO for Direction PINs
    initSystemClockTo40Mhz();
    enablePort(PORTC);              //Enable clock for Port C
    selectPinPushPullOutput(PORTC, PIN_IN1);
    selectPinPushPullOutput(PORTC, PIN_IN2);
    enablePort(PORTC);              //Enable clock for Port C
    selectPinPushPullOutput(PORTC, PIN_IN3);
    selectPinPushPullOutput(PORTC, PIN_IN4);


    // --- Set up PF4 (SW1) ---
    enablePort(PORTF);
    selectPinDigitalInput(PORTF, 4);
    enablePinPullup(PORTF, 4);
    setPinCommitControl(PORTF, 4);
    selectPinInterruptFallingEdge(PORTF, 4); // trigger on button press (falling edge)
    clearPinInterrupt(PORTF, 4);
    enablePinInterrupt(PORTF, 4);

    NVIC_EN0_R |= 1 << (INT_GPIOF - 16); // Enable interrupt in NVIC

    //Red LED for calibration  (PF1)
    selectPinPushPullOutput(PORTF, 1);
}

void initPWM()
{
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    // Configure PF2,3 as PWM output
    GPIO_PORTF_DEN_R |= PWM6_MASK | PWM7_MASK;
    GPIO_PORTF_AFSEL_R |= PWM6_MASK | PWM7_MASK;
    GPIO_PORTF_PCTL_R &= ~(GPIO_PCTL_PF2_M | GPIO_PCTL_PF3_M);
    GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF2_M1PWM6 | GPIO_PCTL_PF3_M1PWM7;

    //Configure PWM
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM1 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM1_3_CTL_R = 0;         // turn-off PWM1 generator 3 (drives outs 6 and 7)
    PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO; // output 5 on PWM1, gen 2b, cmpb
    PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
    //SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_2;
    PWM1_3_LOAD_R = 13000; // set frequency to 40 MHz sys clock / 2 / LOAD = 2 kHz
    //  PWM1_3_LOAD_R = 20000;                            // (internal counter counts down from load value to zero)

    PWM1_3_CMPA_R = 0;                      // (0=always low, 20000=always high)
    PWM1_3_CMPB_R = 0;
    PWM1_3_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 3

    PWM1_ENABLE_R = PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;   //Enable PWM output
}

// Function to select the user bank
void selectUserBank(uint8_t bank)
{
    writeI2c0Register(ICM20948_ADDR, BANK_SELECT_REG, bank);
    waitMicrosecond(1000);  // Give some time for the bank to switch
}

// Function to write data to a register
void writeICM(uint8_t bank, uint8_t reg, uint8_t value)
{
    selectUserBank(bank);
    writeI2c0Register(ICM20948_ADDR, reg, value);
    selectUserBank(_b0);
}

//Read a register
uint8_t readICM(uint8_t bank, uint8_t reg)
{
    uint8_t data;
    selectUserBank(bank);
    data = readI2c0Register(ICM20948_ADDR, reg);
    selectUserBank(_b0);
    return data;
}


// Function to read a 16-bit register (high byte + low byte)
int16_t readSensor(uint8_t regH)
{
//    int16_t data;
//    uint8_t highByte, lowByte;
//
//    // Read the high and low byte of the accelerometer data
//    highByte = readI2c0Register(ICM20948_ADDR, regH);
//    lowByte = readI2c0Register(ICM20948_ADDR, regL);
//
//    // Combine the high and low byte to form a 16-bit data value
//    data = (int16_t) ((highByte << 8) | lowByte);
//
//    return data;
    uint8_t data[2];
    readI2c0Registers(ICM20948_ADDR, regH, data, 2); // read 2 bytes starting from high byte register
    return (int16_t)((data[0] << 8) | data[1]);
}


void initICM20948()
{
  //  uint8_t temp_data;

    // IMU reset: DEVICE_RESET: reset the sensor, SLEEP mode, auto select best clock source
    writeICM(_b0, PWR_MGMT_1, 0xc1);
    waitMicrosecond(10000);  // Wait for reset to complete

    // Exit from sleep mode,
    writeICM(_b0, PWR_MGMT_1, 0x01);

    //output data rate start time alignment: sensors will be sampled synchronus at the same time
    writeICM(_b0, ODR_ALIGN_EN, 0x01);

    // Gyroscope configuration in Bank 2
    writeICM(_b2, GYRO_SMPLRT_DIV, 0x00);  // Gyroscope sample rate divider = 0 (1.1kHz)
    writeICM(_b2, GYRO_CONFIG_1, ((GYRO_RANGE_VALUE << 1) | 0x01));  // Gyroscope range set and enable digital filter

    // Accelerometer configuration in Bank 2
    writeICM(_b2, ACCEL_SMPLRT_DIV_1, 0x00);  // Accelerometer sample rate divider = 0 (1.1kHz)
    writeICM(_b2, ACCEL_SMPLRT_DIV_2, 0x00);  // Accelerometer sample rate divider
    writeICM(_b2, ACCEL_CONFIG,((ACCEL_RANGE_VALUE << 1) | 0x01));  // Accelerometer range and filter config

    selectUserBank(0);
    putsUart0("ICM-20948 Initialized.\n");
}


void calibrateSensor(int16_t* offset_ax, int16_t* offset_az, int16_t* offset_gy)
{
    int32_t sum_ax = 0, sum_az = 0, sum_gy = 0;
    const int samples = 500;
    int16_t ax, az, gy;

    setPinValue(PORTF, 1, 1);  // Turn on LED during calibration
    putsUart0("Calibrating... Keep still.\n");
    int i;
    for (i = 0; i < samples; i++)
    {
        ax = readSensor(ACCEL_XOUT_H);
        az = readSensor(ACCEL_ZOUT_H);
        gy = readSensor(GYRO_YOUT_H);

        sum_ax += ax;
        sum_az += az;
        sum_gy += gy;
    }

    *offset_ax = sum_ax / samples;
    *offset_az = (sum_az / samples) - 8192; // assuming ±4g = 8192 LSB/g → standing still = 1g
    *offset_gy = sum_gy / samples;
    setPinValue(PORTF, 1, 0);  // Turn off LED after calibration

    // Debug print
    snprintf(buffer, sizeof(buffer),
             "Offsets -> offset_ax: %d | offset_az: %d | offset_gy: %d\n",
             *offset_ax, *offset_az, *offset_gy);
    putsUart0(buffer);
    waitMicrosecond(2000);
}

//SW1 as interrupt to call to calculate offset
void GPIOF_Handler()
{
    if (getPinValue(PORTF, 4) == 0)  // Make sure it's really pressed
    {
        calibrateSensor((int16_t*)&offset_ax, (int16_t*)&offset_az, (int16_t*)&offset_gy);
        putsUart0("Calibration done!\n");
        waitMicrosecond(2000000);
    }

    clearPinInterrupt(PORTF, 4); // Clear interrupt flag
}

void readWhoami()
{
    uint8_t whoAmI = readICM(0, 0x00);
    //For ICM-20948, the WHO_AM_I register should return 0xEA if the sensor is connected correctly
    if (whoAmI == 0xEA)
    {
        putsUart0("ICM-20948 is connected.\n");
    }
    else
    {
        putsUart0("Failed to connect to ICM-20948!\n");
    }
}


void setMotorSpeed(uint32_t right_speed, uint32_t left_speed)
{
    //PWM1_3_CTL_R = 0;
    PWM1_3_CMPA_R = right_speed;
    PWM1_3_CMPB_R = left_speed;
   // PWM1_3_CTL_R = PWM_1_CTL_ENABLE;
}


void setMotorDirection(bool forward)
{
    if (forward)
    {

        INT1 = 1;
        INT2 = 0;
        INT3 = 1;
        INT4 = 0;
    }
    else
    {
        INT1 = 0;
        INT2 = 1;
        INT3 = 0;
        INT4 = 1;
    }
}

void manualSetSpeed()
{
   // Prompt for data
   putsUart0("s(Speed), d(Direction): ");
   // Get user input
   getsUart0(&data);
   // Echo back to the user of the TTY interface for testing
   putsUart0(data.buffer);
   putcUart0('\n');
   // Parse fields
   parseFields(&data);
   // Echo back the parsed field data (type and fields)
   uint8_t i;
   for (i = 0; i < data.fieldCount; i++)
   {
       putcUart0(data.fieldType[i]);
       putcUart0('\t');
       putsUart0(&data.buffer[data.fieldPosition[i]]);
       putcUart0('\n');
   }
   if (isCommand(&data, "s", 2))  //Speed
   {
       setMotorSpeed(getFieldInteger(&data,2), getFieldInteger(&data,1));
   }
   else if(isCommand(&data, "d",1)) //Direction
   {
       setMotorDirection(getFieldInteger(&data,1));
   }
}
// Function to calculate pitch using complementary filter
float compFilter(int16_t rawAccelX, int16_t rawAccelZ, int16_t gyroY_dps)
{
    accel_pitch = atan2(rawAccelX, rawAccelZ) * (180.0 / M_PI);  // Convert to degrees : Calculate pitch angle from accelerometer data (in radians)
    return  (alpha * (pitch + gyroY_dps * dt)) + (1 - alpha) * accel_pitch;
}



void initSysTick(void)
{
    NVIC_ST_CTRL_R = 0;                      // Disable SysTick during setup
    NVIC_ST_RELOAD_R = 0xFFFFFF;             // Max reload value (24-bit timer)
    NVIC_ST_CURRENT_R = 0;                   // Clear current
    NVIC_ST_CTRL_R = NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_ENABLE; // Use system clock, enable
}

uint32_t getMicroseconds(void)
{
    return (0xFFFFFF - NVIC_ST_CURRENT_R) / 40; // 40MHz clock = 25ns → divide by 40 to get µs
}

int main(void)
{
    initSystemClockTo40Mhz();
    initGPIO();
    initPWM();
    initUart0();
    initI2c0();
    initICM20948();
 //   initSysTick();




//    setMotorSpeed(19999,19999);
 //   setMotorDirection(1);






    uint32_t lastTime;
    while(1)
    {
//        uint32_t now = getMicroseconds();
//        dt = (now - lastTime) / 1000000.0f;  // convert µs to seconds
//        lastTime = now;

         //Read accelerometer data
        int16_t rawAccelX = readSensor(ACCEL_XOUT_H) - offset_ax;
        int16_t rawAccelZ = readSensor(ACCEL_ZOUT_H) - offset_az;
        int16_t rawGyroY  = readSensor(GYRO_YOUT_H)  - offset_gy;

        float accelX_g = rawAccelX / ACCEL_SENS;  // in g
        float accelZ_g = rawAccelZ / ACCEL_SENS;  // in g
        float gyroY_dps = rawGyroY / GYRO_SENS;   // in degrees/second

        // --- Pitch ---
      //  pitch = compFilter(accelX, accelZ, gyroY);
        pitch = compFilter(rawAccelX, rawAccelZ, gyroY_dps);

        // --- PID ---
          float error = 0.0 - pitch;
          //integral += error * dt;
          integral += error ;
         // float derivative = (error - previousError) / dt;
          float derivative = (error - previousError);
          float output = KP * error + KI * integral + KD * derivative;
          previousError = error;

          // Clamp output
          if (output > 50) output = 50;
          if (output < -50) output = -50;

          // Convert to PWM signal
          // Scale output
          //uint32_t pwmValue = (uint32_t)(fabs(output) / 50.0f) * 5000;
          uint32_t pwmValue = 11000 + (uint32_t)(fabs(output) * 40.0f);

          if (pwmValue > 12999)
              {
                  pwmValue = 12999;
              }

          // Add dead zone
          if (fabs(error) < 1.6)
          {
              setMotorSpeed(0, 0);
          }

          else
          {
              setMotorSpeed(pwmValue, pwmValue );
              setMotorDirection(output > 0);
          }

        //   Optional: emergency stop if tilted too far
          if (fabs(pitch) > 45)
          {
              setMotorSpeed(0, 0);
              while (1);
          }

          // Debug print
          snprintf(buffer, sizeof(buffer),
                                 "%.6f |  accX: %.2f g | accZ: %.2f g | gyroY: %.2f dps | accel_pitch: %.2f | Pitch: %.2f | Output: %.2f | PWM: %d\n\n",
                                 dt, accelX_g, accelZ_g, gyroY_dps, accel_pitch, pitch, output, pwmValue);
          if (++printCounter >= printInterval)
          {
//              snprintf(buffer, sizeof(buffer), "Accel [X]: %6d  [Z]: %6d Gyro  [Y]: %6d\n", accelX, accelZ, gyroY);
//              putsUart0(buffer);

              printCounter = 0;

              putsUart0(buffer);

          }

        //  waitMicrosecond(500); // 10ms delay


    }


}




