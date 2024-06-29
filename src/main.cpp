// main.cpp
#include "mbed.h"
#include "PID.hpp"

// Initialize variables
BufferedSerial pc(USBTX, USBRX, 115200); // Serial communication
CAN can(PA_11, PA_12, (int)1e6);         // CAN communication
uint8_t DATA[8] = {};                    // CAN data array
int x = 0, y = 0, w = 0;                 // Rotation variables

// PID control parameters
float ckp = 0, cki = 0.0, ckd = 0.00;
float sample_time = 0.02;
float target_x = 0, target_y = 0;
float measured_x = 0, measured_y = 0;

PID pid_x(ckp, cki, ckd, sample_time);
PID pid_y(ckp, cki, ckd, sample_time);

void updateTunings() {
    pid_x.setTunings(ckp, cki, ckd);
    pid_y.setTunings(ckp, cki, ckd);
}

int main()
{
    while (1)
    {
        int16_t kaitensuu = 0, kaitensuu1 = 0, kaitensuu2 = 0, kaitensuu3 = 0; // Rotation variables
        char buf;                               // Receive buffer
        CANMessage msg, msg1, msg2, msg3, msg4; // CAN message variables

        if (pc.readable() && pc.read(&buf, sizeof(buf)))
        {
            if (buf == 's')
            {
                target_x = 1000; // Update target value for X-axis
                target_y = 1500; // Update target value for Y-axis
            }
            else if (buf == 't')
            {
                ckp += 0.05;
                updateTunings();
                printf("kp = %f , ki = %f, kd = %f\n", ckp, cki, ckd);
            }
            else if (buf == 'g')
            {
                ckp -= 0.05;
                updateTunings();
                printf("kp = %f , ki = %f, kd = %f\n", ckp, cki, ckd);
            }
            else if (buf == 'y')
            {
                cki += 0.05;
                updateTunings();
                printf("kp = %f , ki = %f, kd = %f\n", ckp, cki, ckd);
            }
            else if (buf == 'h')
            {
                cki -= 0.05;
                updateTunings();
                printf("kp = %f , ki = %f, kd = %f\n", ckp, cki, ckd);
            }
            else if (buf == 'u')
            {
                ckd += 0.0005;
                updateTunings();
                printf("kp = %f , ki = %f, kd = %f\n", ckp, cki, ckd);
            }
            else if (buf == 'j')
            {
                ckd -= 0.0005;
                updateTunings();
                printf("kp = %f , ki = %f, kd = %f\n", ckp, cki, ckd);
            }

            // Calculate PID control outputs for X and Y axes
            float output_x = pid_x.calculate(target_x, measured_x);
            float output_y = pid_y.calculate(target_y, measured_y);

            // Calculate outputs
            float output = output_x + output_y;
            float output1 = output_x - output_y;
            float output2 = -output_x + output_y;
            float output3 = -output_x - output_y;

            // Store outputs in DATA array
            int16_t output_int16 = static_cast<int16_t>(output);
            DATA[0] = output_int16 >> 8;   // MSB
            DATA[1] = output_int16 & 0xFF; // LSB
            int16_t output1_int16 = static_cast<int16_t>(output1);
            DATA[2] = output1_int16 >> 8;   // MSB
            DATA[3] = output1_int16 & 0xFF; // LSB
            int16_t output2_int16 = static_cast<int16_t>(output2);
            DATA[4] = output2_int16 >> 8;   // MSB
            DATA[5] = output2_int16 & 0xFF; // LSB
            int16_t output3_int16 = static_cast<int16_t>(output3);
            DATA[6] = output3_int16 >> 8;   // MSB
            DATA[7] = output3_int16 & 0xFF; // LSB

            // Send CAN message
            CANMessage msg0(0x200, DATA, 8);
            can.write(msg0);

            // Receive CAN messages and update rotation variables
            if (can.read(msg1) && msg1.id == 0x201)
            {
                kaitensuu = (msg1.data[0] << 8) | msg1.data[1]; // Update rotation
            }
            if (can.read(msg2) && msg2.id == 0x202)
            {
                kaitensuu1 = (msg2.data[0] << 8) | msg2.data[1]; // Update rotation
            }
            if (can.read(msg3) && msg3.id == 0x203)
            {
                kaitensuu2 = (msg3.data[0] << 8) | msg3.data[1]; // Update rotation
            }
            if (can.read(msg4) && msg4.id == 0x204)
            {
                kaitensuu3 = (msg4.data[0] << 8) | msg4.data[1]; // Update rotation
            }

            // Calculate x, y, and w from rotation variables
            x = (kaitensuu + kaitensuu1 - kaitensuu2 - kaitensuu3) / 4;
            y = (kaitensuu - kaitensuu2 + kaitensuu1 - kaitensuu3) / 4;
            w = (kaitensuu + kaitensuu2 + kaitensuu1 + kaitensuu3) / 4;

            // Update measured values
            measured_x = x;
            measured_y = y;

            // Print rotation values
            printf("0 = %d , 1 = %d, 2 = %d, 3 = %d\n", kaitensuu, kaitensuu1, kaitensuu2, kaitensuu3);

            ThisThread::sleep_for(20ms); // Wait for 20 milliseconds
        }
    }
}
