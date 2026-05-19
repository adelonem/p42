#include "application.h"
#include "space_game.h"

int main(int, char**)
{
    space_game game {};

    return run_application(
        {
            .title  = "Vaisseau SDL3",
            .width  = 900,
            .height = 700,
        },
        game
    );
}
