#pragma once

#include <math.h>

#include "Motor.hpp"
#include "Encoder.hpp"
#include "PID.hpp"

class Drive {
public:
    enum class MotionMode : uint8_t {
        Straight,
        Turn
    };

    Drive(Motor& motL, Motor& motR, Encoder& encL, Encoder& encR,
          PID& pidL, PID& pidR, float wheelRadius, float wheelBase)
        : motL_(motL), motR_(motR), encL_(encL), encR_(encR),
          pidL_(pidL), pidR_(pidR), wheelRadius_(wheelRadius), wheelBase_(wheelBase) {}

    void goMeters(float meters) {
        mode_ = MotionMode::Straight;
        setWheelTargets(meters, meters);
    }

    void turnDegrees(float degrees) {
        float thetaRad = degrees * (PI / 180.0f);
        float arc = (wheelBase_ * 0.5f) * thetaRad;
        mode_ = MotionMode::Turn;
        setWheelTargets(arc, -arc);
    }

    void update() {
        float distL = encL_.getRadians() - startL_;
        float distR = encR_.getRadians() - startR_;
        float outL = pidL_.compute(distL);
        float outR = pidR_.compute(distR);

        if (isSettled()) {
            motL_.setPWM(0);
            motR_.setPWM(0);
            return;
        }

        motL_.setPWM(outL);
        motR_.setPWM(outR);
    }

    bool isSettled() const {
        float posEps = (mode_ == MotionMode::Turn) ? kTurnPosEps : kStraightPosEps;
        float velEps = (mode_ == MotionMode::Turn) ? kTurnVelEps : kStraightVelEps;
        uint8_t settleCyclesRequired =
            (mode_ == MotionMode::Turn) ? kTurnSettleCyclesRequired : kStraightSettleCyclesRequired;

        bool nearZero = pidNearZero(pidL_, posEps, velEps) && pidNearZero(pidR_, posEps, velEps);
        if (nearZero) {
            if (settledCycles_ < settleCyclesRequired) {
                ++settledCycles_;
            }
        } else {
            settledCycles_ = 0;
        }
        return settledCycles_ >= settleCyclesRequired;
    }

private:
    void setWheelTargets(float leftMeters, float rightMeters) {
        float leftRad = leftMeters / wheelRadius_;
        float rightRad = rightMeters / wheelRadius_;
        pidL_.setTarget(leftRad);
        pidR_.setTarget(rightRad);
        startL_ = encL_.getRadians();
        startR_ = encR_.getRadians();
        pidL_.reset();
        pidR_.reset();
        settledCycles_ = 0;
    }

    static bool pidNearZero(const PID& pid, float posEps, float velEps) {
        return fabs(pid.getError()) <= posEps &&
               fabs(pid.getErrorVelocity()) <= velEps;
    }

    static constexpr float kStraightPosEps = 0.08f;
    static constexpr float kStraightVelEps = 0.10f;
    static constexpr uint8_t kStraightSettleCyclesRequired = 8;

    static constexpr float kTurnPosEps = 0.12f;
    static constexpr float kTurnVelEps = 0.10f;
    static constexpr uint8_t kTurnSettleCyclesRequired = 8;

    Motor& motL_;
    Motor& motR_;
    Encoder& encL_;
    Encoder& encR_;
    PID& pidL_;
    PID& pidR_;
    float wheelRadius_;
    float wheelBase_;
    MotionMode mode_ = MotionMode::Straight;
    float startL_ = 0;
    float startR_ = 0;
    mutable uint8_t settledCycles_ = 0;
};
