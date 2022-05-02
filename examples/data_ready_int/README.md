## Introduction

This folder contains two example sketches that turn on the data ready interrupt line from the MPU-6050. In both cases the IMU has been configured to generate an active low data ready interrupt signal.

## mpu_6050_polling

mpu_6050_polling.ino simply polls the GPIO line that the MPU-6050's INT line is wired to. When it see it go low, it reads the MPU-6050's data registers.

## mpu_6050_interrupt

mpu_6050_interrupt.ino configures an interrupt with an associated interrupt handler/service routine (ISR). 

## Timing Info

Both examples contain a timing_info folder with text and oscilloscope traces showing the relationship between the interrupt signal, interrupt handler, and processing of the new data.

The interrupt folder show images of the interrupt configured for latching or pulsed operation.