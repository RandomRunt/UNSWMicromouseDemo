#pragma once

#include <Arduino.h>
#include <math.h>

struct Pose2D {
    float x;
    float y;
    float theta;
};

struct Waypoint2D {
    float x;
    float y;
};

class PurePursuit {
public:
    PurePursuit(
        float wheelBase,
        float lookahead,
        float cruiseSpeed,
        float maxOmega = 3.5f,
        float turnSlow = 0.08f,
        float speedLookahead = -1.0f)
        : wheelBase_(wheelBase),
          lookahead_(lookahead),
          cruiseSpeed_(cruiseSpeed),
          maxOmega_(maxOmega),
          turnSlow_(turnSlow),
          speedLookahead_(speedLookahead > 0.0f ? speedLookahead : lookahead) {}

    void setPath(const Waypoint2D* waypoints, size_t count) {
        waypoints_ = waypoints;
        count_ = count;
        segment_ = 0;
        finished_ = (count_ < 2);
        lastCurvature_ = 0.0f;
    }

    void setLookahead(float lookahead) { lookahead_ = lookahead; }
    void setSpeedLookahead(float speedLookahead) { speedLookahead_ = speedLookahead; }
    void setCruiseSpeed(float speed) { cruiseSpeed_ = speed; }
    void setMaxOmega(float maxOmega) { maxOmega_ = maxOmega; }
    void setTurnSlow(float turnSlow) { turnSlow_ = turnSlow; }
    void setWaypointTolerance(float tolerance) { waypointTol_ = tolerance; }

    void reset() {
        segment_ = 0;
        finished_ = (count_ < 2);
        lastCurvature_ = 0.0f;
    }

    bool isFinished() const { return finished_; }
    size_t getSegment() const { return segment_; }
    float lastCurvature() const { return lastCurvature_; }

    // True if current→next bends and pose is within distM of the corner.
    bool approachingCorner(const Pose2D& pose, float distM = 0.12f) const {
        if (!waypoints_ || segment_ + 2 >= count_) {
            return false;
        }
        const size_t s = segment_;
        const float ax = waypoints_[s + 1].x - waypoints_[s].x;
        const float ay = waypoints_[s + 1].y - waypoints_[s].y;
        const float bx = waypoints_[s + 2].x - waypoints_[s + 1].x;
        const float by = waypoints_[s + 2].y - waypoints_[s + 1].y;
        const float la = hypotf(ax, ay);
        const float lb = hypotf(bx, by);
        if (la < 1e-6f || lb < 1e-6f) {
            return false;
        }
        const float dot = (ax * bx + ay * by) / (la * lb);
        if (dot > 0.5f) {
            return false;
        }
        const float dx = waypoints_[s + 1].x - pose.x;
        const float dy = waypoints_[s + 1].y - pose.y;
        return hypotf(dx, dy) <= distM;
    }

    bool compute(const Pose2D& pose, float& vLeft, float& vRight) {
        if (finished_ || !waypoints_ || count_ < 2) {
            vLeft = 0;
            vRight = 0;
            lastCurvature_ = 0.0f;
            return false;
        }

        advanceSegment(pose);

        float gx, gy;
        if (!lookaheadPoint(pose, lookahead_, gx, gy)) {
            finished_ = true;
            vLeft = 0;
            vRight = 0;
            lastCurvature_ = 0.0f;
            return false;
        }

        float curvature = curvatureToPoint(pose, gx, gy);
        lastCurvature_ = curvature;

        // Longer preview for speed only — brake before the turn without
        // pulling steering early (steer still uses short lookahead_).
        float speedCurvature = curvature;
        if (speedLookahead_ > lookahead_ + 1e-6f) {
            float sx, sy;
            if (lookaheadPoint(pose, speedLookahead_, sx, sy)) {
                speedCurvature = curvatureToPoint(pose, sx, sy);
            }
        }
        const float slowCurvature =
            fmaxf(fabsf(curvature), fabsf(speedCurvature));

        // Fast on straights; slow when preview/steer curvature rises.
        float speed = cruiseSpeed_ / (1.0f + turnSlow_ * slowCurvature * cruiseSpeed_);
        float omega = speed * curvature;

        if (fabsf(omega) > maxOmega_) {
            omega = (omega > 0.0f) ? maxOmega_ : -maxOmega_;
            speed = fminf(speed, fabsf(omega) / fmaxf(fabsf(curvature), 1e-3f));
        }

        vLeft = speed - omega * wheelBase_ * 0.5f;
        vRight = speed + omega * wheelBase_ * 0.5f;
        return true;
    }

private:
    void advanceSegment(const Pose2D& pose) {
        // Only leave a segment once past its end (t >= 1). Euclidean
        // "near waypoint" alone turns early and clips the outside wall.
        while (segment_ < count_ - 1) {
            float ax = waypoints_[segment_].x;
            float ay = waypoints_[segment_].y;
            float bx = waypoints_[segment_ + 1].x;
            float by = waypoints_[segment_ + 1].y;
            float dx = bx - ax;
            float dy = by - ay;
            float len2 = dx * dx + dy * dy;
            float t = 1.0f;
            if (len2 > 1e-9f) {
                t = ((pose.x - ax) * dx + (pose.y - ay) * dy) / len2;
            }
            if (t < 1.0f) {
                break;
            }
            segment_++;
        }

        if (segment_ >= count_ - 1) {
            float dx = waypoints_[count_ - 1].x - pose.x;
            float dy = waypoints_[count_ - 1].y - pose.y;
            if (hypotf(dx, dy) <= waypointTol_) {
                finished_ = true;
            }
        }
    }

    static float curvatureToPoint(const Pose2D& pose, float gx, float gy) {
        float dx = gx - pose.x;
        float dy = gy - pose.y;
        float cosT = cosf(pose.theta);
        float sinT = sinf(pose.theta);
        float localX = cosT * dx + sinT * dy;
        float localY = -sinT * dx + cosT * dy;
        float ld2 = localX * localX + localY * localY;
        if (ld2 < 1e-6f) {
            return 0.0f;
        }
        return 2.0f * localY / ld2;
    }

    bool lookaheadPoint(const Pose2D& pose, float distance, float& gx, float& gy) const {
        float remaining = distance;

        for (size_t i = segment_; i < count_ - 1; ++i) {
            float ax = waypoints_[i].x;
            float ay = waypoints_[i].y;
            float bx = waypoints_[i + 1].x;
            float by = waypoints_[i + 1].y;

            float segDx = bx - ax;
            float segDy = by - ay;
            float segLen = hypotf(segDx, segDy);
            if (segLen < 1e-6f) {
                continue;
            }

            float startX = ax;
            float startY = ay;
            if (i == segment_) {
                float projX, projY;
                projectOntoSegment(pose.x, pose.y, ax, ay, bx, by, projX, projY);
                startX = projX;
                startY = projY;
            }

            float alongDx = bx - startX;
            float alongDy = by - startY;
            float alongLen = hypotf(alongDx, alongDy);
            if (alongLen < 1e-6f) {
                continue;
            }

            if (alongLen >= remaining) {
                float t = remaining / alongLen;
                gx = startX + alongDx * t;
                gy = startY + alongDy * t;
                return true;
            }

            remaining -= alongLen;
        }

        gx = waypoints_[count_ - 1].x;
        gy = waypoints_[count_ - 1].y;
        return true;
    }

    static void projectOntoSegment(
        float px, float py,
        float ax, float ay, float bx, float by,
        float& projX, float& projY) {
        float dx = bx - ax;
        float dy = by - ay;
        float len2 = dx * dx + dy * dy;
        if (len2 < 1e-9f) {
            projX = ax;
            projY = ay;
            return;
        }

        float t = constrain(((px - ax) * dx + (py - ay) * dy) / len2, 0.0f, 1.0f);
        projX = ax + dx * t;
        projY = ay + dy * t;
    }

    float wheelBase_;
    float lookahead_;
    float cruiseSpeed_;
    float maxOmega_;
    float turnSlow_;
    float speedLookahead_;
    float waypointTol_ = 0.020f;

    const Waypoint2D* waypoints_ = nullptr;
    size_t count_ = 0;
    size_t segment_ = 0;
    bool finished_ = true;
    float lastCurvature_ = 0.0f;
};
