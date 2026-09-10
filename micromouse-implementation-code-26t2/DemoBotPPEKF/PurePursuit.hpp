#pragma once

#include <Arduino.h>
#include <math.h>

#include "Route.hpp"

namespace mtrn3100 {

struct Pose2D {
    float x;
    float y;
    float heading;
};

class PurePursuit {
public:
    PurePursuit(
        float wheelBaseM,
        float lookaheadM,
        float cruiseSpeedMps,
        float maximumOmegaRadS,
        float turnSlow,
        float speedLookaheadM)
        : wheelBaseM_(wheelBaseM),
          lookaheadM_(lookaheadM),
          cruiseSpeedMps_(cruiseSpeedMps),
          maximumOmegaRadS_(maximumOmegaRadS),
          turnSlow_(turnSlow),
          speedLookaheadM_(speedLookaheadM) {}

    void setPath(const Waypoint* waypoints, size_t count) {
        waypoints_ = waypoints;
        count_ = count;
        segment_ = 0;
        finished_ = count < 2;
        lastCurvature_ = 0.0f;
    }

    void setCruiseSpeed(float speedMps) { cruiseSpeedMps_ = speedMps; }
    size_t segment() const { return segment_; }
    float lastCurvature() const { return lastCurvature_; }
    bool isFinished() const { return finished_; }

    bool compute(const Pose2D& pose, float& leftMps, float& rightMps) {
        if (finished_ || waypoints_ == nullptr || count_ < 2) {
            leftMps = 0.0f;
            rightMps = 0.0f;
            return false;
        }

        advanceSegment(pose);
        float goalX = 0.0f;
        float goalY = 0.0f;
        if (!lookaheadPoint(pose, lookaheadM_, goalX, goalY)) {
            finished_ = true;
            leftMps = 0.0f;
            rightMps = 0.0f;
            return false;
        }

        const float curvature = curvatureToPoint(pose, goalX, goalY);
        lastCurvature_ = curvature;
        float previewCurvature = curvature;
        if (speedLookaheadM_ > lookaheadM_) {
            float previewX = 0.0f;
            float previewY = 0.0f;
            if (lookaheadPoint(pose, speedLookaheadM_, previewX, previewY)) {
                previewCurvature = curvatureToPoint(pose, previewX, previewY);
            }
        }

        const float slowingCurvature =
            fmaxf(fabsf(curvature), fabsf(previewCurvature));
        float speed = cruiseSpeedMps_ /
            (1.0f + turnSlow_ * slowingCurvature * cruiseSpeedMps_);
        float omega = speed * curvature;
        if (fabsf(omega) > maximumOmegaRadS_) {
            omega = omega > 0.0f ? maximumOmegaRadS_ : -maximumOmegaRadS_;
            speed = fminf(
                speed,
                fabsf(omega) / fmaxf(fabsf(curvature), 1e-3f));
        }

        leftMps = speed - 0.5f * omega * wheelBaseM_;
        rightMps = speed + 0.5f * omega * wheelBaseM_;
        return true;
    }

private:
    void advanceSegment(const Pose2D& pose) {
        while (segment_ < count_ - 1) {
            const Waypoint& start = waypoints_[segment_];
            const Waypoint& end = waypoints_[segment_ + 1];
            const float dx = end.x - start.x;
            const float dy = end.y - start.y;
            const float lengthSquared = dx * dx + dy * dy;
            float progress = 1.0f;
            if (lengthSquared > 1e-9f) {
                progress = ((pose.x - start.x) * dx
                          + (pose.y - start.y) * dy) / lengthSquared;
            }
            if (progress < 1.0f) break;
            ++segment_;
        }

        if (segment_ >= count_ - 1) {
            const float dx = waypoints_[count_ - 1].x - pose.x;
            const float dy = waypoints_[count_ - 1].y - pose.y;
            if (hypotf(dx, dy) <= waypointToleranceM_) finished_ = true;
        }
    }

    static float curvatureToPoint(
        const Pose2D& pose, float goalX, float goalY) {
        const float dx = goalX - pose.x;
        const float dy = goalY - pose.y;
        const float cosine = cosf(pose.heading);
        const float sine = sinf(pose.heading);
        const float localX = cosine * dx + sine * dy;
        const float localY = -sine * dx + cosine * dy;
        const float distanceSquared = localX * localX + localY * localY;
        return distanceSquared < 1e-6f ? 0.0f
                                      : 2.0f * localY / distanceSquared;
    }

    bool lookaheadPoint(
        const Pose2D& pose,
        float lookahead,
        float& goalX,
        float& goalY) const {
        float remaining = lookahead;
        for (size_t index = segment_; index < count_ - 1; ++index) {
            const Waypoint& start = waypoints_[index];
            const Waypoint& end = waypoints_[index + 1];
            float startX = start.x;
            float startY = start.y;
            if (index == segment_) {
                projectOntoSegment(
                    pose.x, pose.y,
                    start.x, start.y, end.x, end.y,
                    startX, startY);
            }

            const float dx = end.x - startX;
            const float dy = end.y - startY;
            const float length = hypotf(dx, dy);
            if (length < 1e-6f) continue;
            if (length >= remaining) {
                const float fraction = remaining / length;
                goalX = startX + fraction * dx;
                goalY = startY + fraction * dy;
                return true;
            }
            remaining -= length;
        }

        goalX = waypoints_[count_ - 1].x;
        goalY = waypoints_[count_ - 1].y;
        return true;
    }

    static void projectOntoSegment(
        float pointX, float pointY,
        float startX, float startY,
        float endX, float endY,
        float& projectionX, float& projectionY) {
        const float dx = endX - startX;
        const float dy = endY - startY;
        const float lengthSquared = dx * dx + dy * dy;
        if (lengthSquared < 1e-9f) {
            projectionX = startX;
            projectionY = startY;
            return;
        }
        const float fraction = constrain(
            ((pointX - startX) * dx + (pointY - startY) * dy)
                / lengthSquared,
            0.0f,
            1.0f);
        projectionX = startX + fraction * dx;
        projectionY = startY + fraction * dy;
    }

    const float wheelBaseM_;
    const float lookaheadM_;
    float cruiseSpeedMps_;
    const float maximumOmegaRadS_;
    const float turnSlow_;
    const float speedLookaheadM_;
    const Waypoint* waypoints_ = nullptr;
    size_t count_ = 0;
    size_t segment_ = 0;
    float waypointToleranceM_ = 0.020f;
    float lastCurvature_ = 0.0f;
    bool finished_ = true;
};

}  // namespace mtrn3100
