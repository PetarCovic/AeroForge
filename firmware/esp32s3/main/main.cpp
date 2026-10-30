#include "HilWireProtocol.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include "driver/uart.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace
{
    using namespace aeroforge::hil::firmware;

    constexpr uart_port_t HIL_UART = UART_NUM_0;
    constexpr int HIL_BAUD_RATE = 460800;
    constexpr int HIL_TX_PIN = 1;
    constexpr int HIL_RX_PIN = 3;
    constexpr int RX_BUFFER_SIZE = 2048;
    constexpr int TX_BUFFER_SIZE = 2048;
    constexpr std::int64_t SENSOR_TIMEOUT_US = 100'000;
    constexpr float RATE_GAIN = 5.0f;
    constexpr float MAX_ABS_MOMENT = 20.0f;

    struct ControllerState
    {
        bool haveSensor = false;
        std::uint32_t lastSensorSequence = 0;
        std::uint32_t nextCommandSequence = 1;
        std::int64_t lastSensorArrivalUs = 0;
        float commandedMomentB[3]{};

        void failSafe() noexcept
        {
            for (auto& value : commandedMomentB) value = 0.0f;
            haveSensor = false;
        }

        bool acceptSensor(const SensorSample& sensor,
            std::int64_t nowUs) noexcept
        {
            if (haveSensor && nowUs - lastSensorArrivalUs > SENSOR_TIMEOUT_US)
                failSafe();
            if (haveSensor)
            {
                const std::uint32_t distance =
                    sensor.sequence - lastSensorSequence;
                if (distance == 0 || distance >= 0x80000000u)
                    return false;
            }
            lastSensorSequence = sensor.sequence;
            lastSensorArrivalUs = nowUs;
            haveSensor = true;
            for (int axis = 0; axis < 3; ++axis)
            {
                commandedMomentB[axis] = std::clamp(
                    -RATE_GAIN * sensor.angularVelocityB[axis],
                    -MAX_ABS_MOMENT, MAX_ABS_MOMENT);
            }
            return true;
        }

        void checkWatchdog(std::int64_t nowUs) noexcept
        {
            if (haveSensor && nowUs - lastSensorArrivalUs > SENSOR_TIMEOUT_US)
                failSafe();
        }
    };

    bool writeAll(const std::uint8_t* data, std::size_t length) noexcept
    {
        if (!data || length == 0) return false;
        std::size_t offset = 0;
        while (offset < length)
        {
            const int written = uart_write_bytes(
                HIL_UART, data + offset, length - offset);
            if (written <= 0) return false;
            offset += static_cast<std::size_t>(written);
        }
        return uart_wait_tx_done(HIL_UART, pdMS_TO_TICKS(20)) == ESP_OK;
    }

    void sendControlResponse(const SensorSample& sensor,
        ControllerState& controller)
    {
        ControlCommand command{};
        command.sequence = controller.nextCommandSequence++;
        command.referencedSensorSequence = sensor.sequence;
        command.controllerStatus = 0;
        for (int axis = 0; axis < 3; ++axis)
            command.commandedMomentB[axis] =
            controller.commandedMomentB[axis];

        std::uint8_t outgoing[MAX_ENCODED_FRAME_SIZE]{};
        const std::size_t length = encodeControlFrame(
            command, outgoing, sizeof(outgoing));
        if (length == 0 || !writeAll(outgoing, length))
            controller.failSafe();
    }

    void processFrame(const std::uint8_t* frame,
        std::size_t frameLength,
        ControllerState& controller)
    {
        SensorSample sensor{};
        if (!decodeSensorFrame(frame, frameLength, sensor)) return;
        if (!controller.acceptSensor(sensor, esp_timer_get_time())) return;
        sendControlResponse(sensor, controller);
    }

    void runController()
    {
        ControllerState controller{};
        std::uint8_t incoming[64]{};
        std::uint8_t frame[MAX_ENCODED_FRAME_SIZE]{};
        std::size_t frameLength = 0;
        bool discardUntilDelimiter = false;

        while (true)
        {
            controller.checkWatchdog(esp_timer_get_time());
            const int length = uart_read_bytes(
                HIL_UART, incoming, sizeof(incoming), pdMS_TO_TICKS(10));
            if (length <= 0) continue;

            for (int i = 0; i < length; ++i)
            {
                const std::uint8_t byte = incoming[i];
                if (byte == 0)
                {
                    if (!discardUntilDelimiter && frameLength != 0)
                        processFrame(frame, frameLength, controller);
                    frameLength = 0;
                    discardUntilDelimiter = false;
                    continue;
                }
                if (discardUntilDelimiter) continue;
                if (frameLength >= sizeof(frame) - 1)
                {
                    frameLength = 0;
                    discardUntilDelimiter = true;
                    controller.failSafe();
                    continue;
                }
                frame[frameLength++] = byte;
            }
        }
    }

    void initializeUart()
    {
        uart_config_t config{};
        config.baud_rate = HIL_BAUD_RATE;
        config.data_bits = UART_DATA_8_BITS;
        config.parity = UART_PARITY_DISABLE;
        config.stop_bits = UART_STOP_BITS_1;
        config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        config.source_clk = UART_SCLK_DEFAULT;

        ESP_ERROR_CHECK(uart_driver_install(
            HIL_UART, RX_BUFFER_SIZE, TX_BUFFER_SIZE, 0, nullptr, 0));
        ESP_ERROR_CHECK(uart_param_config(HIL_UART, &config));
        ESP_ERROR_CHECK(uart_set_pin(
            HIL_UART, HIL_TX_PIN, HIL_RX_PIN,
            UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        ESP_ERROR_CHECK(uart_flush_input(HIL_UART));
        const std::uint8_t delimiter = 0;
        (void)writeAll(&delimiter, 1);
    }
}

extern "C" void app_main(void)
{
    initializeUart();
    runController();
}