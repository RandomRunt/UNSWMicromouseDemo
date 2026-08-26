#pragma once

#include "Movement.hpp"

class Controller {
public:
    void initialise();
    void run();

private:
    mtrn3100::Movement move;
    bool ready = false;
};
