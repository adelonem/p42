#pragma once

#include "game.h"

struct application_configuration {
    const char* title {};
    int width         {};
    int height        {};
};

[[nodiscard]] int run_application(const application_configuration& configuration, game& game);
