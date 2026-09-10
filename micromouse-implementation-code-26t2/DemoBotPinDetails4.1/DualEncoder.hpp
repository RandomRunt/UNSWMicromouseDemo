#pragma once

#include <digitalWriteFast.h>
#include <Arduino.h>

namespace mtrn3100 {
  class DualEncoder {
    public:
      DualEncoder(uint8_t enc1, uint8_t enc2, uint8_t enc3, uint8_t enc4)
        : mot1_int(enc1), mot1_dir(enc2), mot2_int(enc3), mot2_dir(enc4) {
        instance = this;  // Store the instance pointer
        pinModeFast(mot1_int, INPUT_PULLUP);
        pinModeFast(mot1_dir, INPUT_PULLUP);
        pinModeFast(mot2_int, INPUT_PULLUP);
        pinModeFast(mot2_dir, INPUT_PULLUP);

        attachInterrupt(digitalPinToInterrupt(mot1_int), DualEncoder::readLeftEncoderISR, RISING);
        attachInterrupt(digitalPinToInterrupt(mot2_int), DualEncoder::readRightEncoderISR, RISING);
      }


      // Encoder function used to update the encoder
      void readLeftEncoder() {
        noInterrupts();
        direction = digitalReadFast(mot1_dir) ? 1 : -1;
        l_count += direction;
        interrupts();
      }

      void readRightEncoder() {
        noInterrupts();
        direction = digitalReadFast(mot2_dir) ? 1 : -1;
        r_count += direction;
        interrupts();
      }

      // Helper function which to convert encoder count to radians
      float getLeftRotation() {
        return (static_cast<float>(l_count) / counts_per_revolution) * 2 * PI;
      }

      float getRightRotation() {
        return (static_cast<float>(r_count) / counts_per_revolution) * 2 * PI;
      }

      // Function to reset encoders
      void resetEncoders() {
        noInterrupts();  // Disable interrupts while resetting
        l_count = 0;
        r_count = 0;
        interrupts();    // Re-enable interrupts
      }

    private:
      static void readLeftEncoderISR() {
        if (instance != nullptr) {
          instance->readLeftEncoder();
        }
      }

      static void readRightEncoderISR() {
        if (instance != nullptr) {
          instance->readRightEncoder();
        }
      }

    public:
      const uint8_t mot1_int, mot1_dir, mot2_int, mot2_dir;
      volatile int8_t direction;
      float position = 0;
      uint16_t counts_per_revolution = 700;
      volatile long l_count = 0;
      volatile long r_count = 0;
      uint32_t prev_time;
      bool read = false;

    private:
      static DualEncoder* instance;
  };

  DualEncoder* DualEncoder::instance = nullptr;
}