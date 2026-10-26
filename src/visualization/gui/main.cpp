#include "VisualizationApp.h"

#include <cstdlib>

int main()
{
    aeroforge::visualization::gui::VisualizationApp app;

    if (!app.initialize())
    {
        return EXIT_FAILURE;
    }

    return app.run();
}
