#pragma once

namespace aeroforge::hil { class HilTelemetryStore; }

namespace aeroforge::visualization::gui::panels
{
    void drawHilDashboardPanel(bool* open,
        const hil::HilTelemetryStore* telemetry);
}
