#pragma once

#include <stdint.h>

// Generative-AI-assisted implementation (OpenAI Codex, 2026-08-11).

namespace mtrn3100 {

enum class ObservedEdge : uint8_t {
    Unknown = 0,
    Open = 1,
    Wall = 2,
};

struct WallObservation {
    ObservedEdge front = ObservedEdge::Unknown;
    ObservedEdge left = ObservedEdge::Unknown;
    ObservedEdge right = ObservedEdge::Unknown;
};

enum class CellMoveResult : uint8_t {
    Arrived = 0,
    InvalidRequest,
    ImuFault,
    LeftWheelStall,
    RightWheelStall,
    WheelMismatch,
    HeadingLost,
    ArrivalUncertain,
    Timeout,
};

enum class MappingPhase : uint8_t {
    Explore = 0,
    ReturnToStart,
    ShortestRun,
    Done,
    Fault,
};

}  // namespace mtrn3100
