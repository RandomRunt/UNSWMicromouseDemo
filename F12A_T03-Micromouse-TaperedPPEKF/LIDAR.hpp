#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <VL6180X.h>

class Lidar {
public:
  static constexpr int SEARCH_START = 48;    // where address scanning begins
  static constexpr int SEARCH_END   = 119;   // last valid 7-bit addr
  static constexpr int ADDRESS_NONE = 0;     // sentinel: none found
  static constexpr int OUT_OF_RANGE_MM = -1; // sentinel: no valid reading yet
  static constexpr size_t MAX_SENSORS = 8;   // registry capacity

  // Per-sensor sanity filter (median window + trust frames).
  static constexpr uint8_t kFilterWindow = 5;
  static constexpr int kMinValidMm = 20;
  static constexpr int kMaxValidMm = 200;
  static constexpr uint8_t kTrustFrames = 3;

  explicit Lidar(uint8_t enablePin, int edgeOffsetMm = 0)
    : _enablePin(enablePin),
      _address(ADDRESS_NONE),
      _edgeOffsetMm(edgeOffsetMm),
      _lastRangeMm(OUT_OF_RANGE_MM) {
    if (registryCount() < MAX_SENSORS) {
      registry()[registryCount()++] = this;
    }
  }

  bool isTrusted() const { return _trusted; }

  // Body-edge clearance only when filter has built confidence.
  int trustedBodyRange() const {
    if (!_trusted) {
      return OUT_OF_RANGE_MM;
    }
    if (_lastRangeMm == OUT_OF_RANGE_MM) {
      return OUT_OF_RANGE_MM;
    }
    const int bodyMm = _lastRangeMm - _edgeOffsetMm;
    if (bodyMm < kMinValidMm || bodyMm > kMaxValidMm) {
      return OUT_OF_RANGE_MM;
    }
    return bodyMm;
  }

  bool pollIfReady() {
    if (_address == ADDRESS_NONE) {
      return false;
    }

    if ((_sensor.readReg(VL6180X::RESULT__INTERRUPT_STATUS_GPIO) & 0x07) != 0x04) {
      return false;
    }

    uint8_t raw = _sensor.readReg(VL6180X::RESULT__RANGE_VAL);
    _sensor.writeReg(VL6180X::SYSTEM__INTERRUPT_CLEAR, 0x01);

    int measured = OUT_OF_RANGE_MM;
    if (_sensor.readRangeStatus() == VL6180X_ERROR_NONE) {
      measured = (int)((uint16_t)_sensor.getScaling() * raw);
    }
    updateFiltered(measured);
    return true;
  }

  // Brings up every Lidar instance constructed so far, in the correct
  // order: disable all first (so none collide while still at the
  // shared default address), then enable+address them one at a time.
  // Call this once in setup(), after Wire.begin().
  static bool initAll(uint8_t rangeIntervalMs = 50) {
    for (size_t i = 0; i < registryCount(); i++) {
      registry()[i]->beginDisabled();
    }
    bool sensorsWorking = true;
    for (size_t i = 0; i < registryCount(); i++) {
      if (!registry()[i]->enable(rangeIntervalMs)) {
        sensorsWorking = false;
      }
    }
    return sensorsWorking;
  }

private:
  void beginDisabled() {
    pinMode(_enablePin, OUTPUT);
    digitalWrite(_enablePin, LOW);
  }

  bool enable(uint8_t rangeIntervalMs = 100, uint16_t powerUpDelayMs = 50) {
    digitalWrite(_enablePin, HIGH);
    delay(powerUpDelayMs);

    _address = findAvailableAddress(searchCursor());
    if (_address == ADDRESS_NONE) {
      return false;
    }

    _sensor.init();
    _sensor.configureDefault();
    _sensor.setTimeout(5);
    _sensor.setAddress((uint8_t)_address);
    _sensor.startRangeContinuous(rangeIntervalMs);

    // Next sensor's search resumes after the address we just claimed.
    searchCursor() = _address + 1;

    resetFilter();
    delay(powerUpDelayMs);
    return true;
  }

  static int findAvailableAddress(int startAddress = SEARCH_START) {
    for (int addr = startAddress; addr <= SEARCH_END; addr++) {
      Wire.beginTransmission(addr);
      uint8_t error = Wire.endTransmission();
      if (error == 2) {
        return addr;
      }
    }
    return ADDRESS_NONE;
  }

  void resetFilter() {
    _historyLen = 0;
    _lastFilteredMm = OUT_OF_RANGE_MM;
    _lastRangeMm = OUT_OF_RANGE_MM;
    resetTrust();
  }

  static int medianOf(const int* values, uint8_t count) {
    if (count == 0) {
      return OUT_OF_RANGE_MM;
    }
    int sorted[kFilterWindow];
    for (uint8_t i = 0; i < count; ++i) {
      sorted[i] = values[i];
    }
    for (uint8_t i = 1; i < count; ++i) {
      int v = sorted[i];
      int8_t j = (int8_t)i;
      while (j > 0 && sorted[j - 1] > v) {
        sorted[j] = sorted[j - 1];
        --j;
      }
      sorted[j] = v;
    }
    return sorted[count / 2];
  }

  void pushHistory(int valueMm) {
    if (_historyLen < kFilterWindow) {
      _history[_historyLen++] = valueMm;
      return;
    }
    for (uint8_t i = 1; i < kFilterWindow; ++i) {
      _history[i - 1] = _history[i];
    }
    _history[kFilterWindow - 1] = valueMm;
  }

  bool acceptSample(int rawMm) const {
    if (rawMm == OUT_OF_RANGE_MM) {
      return false;
    }
    return rawMm >= kMinValidMm && rawMm <= kMaxValidMm;
  }

  bool bodyRangeValidFromFiltered() const {
    if (_lastFilteredMm == OUT_OF_RANGE_MM) {
      return false;
    }
    const int bodyMm = _lastFilteredMm - _edgeOffsetMm;
    return bodyMm >= kMinValidMm && bodyMm <= kMaxValidMm;
  }

  void onGoodFrame() {
    if (_goodFrameCount < kTrustFrames) {
      ++_goodFrameCount;
    }
    _trusted = (_goodFrameCount >= kTrustFrames);
  }

  void resetTrust() {
    _goodFrameCount = 0;
    _trusted = false;
  }

  void clearFilteredState() {
    _historyLen = 0;
    _lastFilteredMm = OUT_OF_RANGE_MM;
    _lastRangeMm = OUT_OF_RANGE_MM;
    resetTrust();
  }

  void updateFiltered(int rawMm) {
    if (!acceptSample(rawMm)) {
      clearFilteredState();
      return;
    }

    pushHistory(rawMm);
    _lastFilteredMm = medianOf(_history, _historyLen);
    _lastRangeMm = _lastFilteredMm;

    if (bodyRangeValidFromFiltered()) {
      onGoodFrame();
    } else {
      resetTrust();
    }
  }

  static int& searchCursor() {
    static int cursor = SEARCH_START;
    return cursor;
  }

  static Lidar** registry() {
    static Lidar* sensors[MAX_SENSORS] = { nullptr };
    return sensors;
  }
  static size_t& registryCount() {
    static size_t count = 0;
    return count;
  }

  VL6180X _sensor;
  uint8_t _enablePin;
  int _address;
  int _edgeOffsetMm;
  int _lastRangeMm;
  int _lastFilteredMm = OUT_OF_RANGE_MM;
  int _history[kFilterWindow]{};
  uint8_t _historyLen = 0;
  uint8_t _goodFrameCount = 0;
  bool _trusted = false;
};
