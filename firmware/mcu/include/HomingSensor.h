#pragma once
#include "gpio.h"
#include "Encoder.h"
#include "CanBus.h"
#include "CanFrame.h"

enum class HomingState : uint8_t {
    IDLE = 0,
    SEARCHING_ENTER = 1, // Waits for entry into the sensor
    SEARCHING_EXIT  = 2, // Waits for exit from the sensor
    SUCCESS         = 3, // Homing successful, offset set
    FAILED          = 4
};

class HomingSensor {
public:
    HomingSensor(GPIO_TypeDef* port, uint16_t pin, 
                 Encoder& encoder, CanBus& bus, uint8_t nodeId, uint8_t sensorId,
                 bool activeHigh = true)
        : gpioPort(port), gpioPin(pin),
          encoder_(encoder), canBus(bus), ownNodeId(nodeId), sensorId(sensorId),
          activeHigh(activeHigh) {}

    /**
     * Starts the homing process (triggered, for example, by a CAN command)
     */
    void startHoming() {
        p1_motor_pos = 0;
        p2_motor_pos = 0;
        state = HomingState::SEARCHING_ENTER;
    }

    void cancelHoming() {
        state = HomingState::IDLE;
    }

    HomingState update() {
        if (state == HomingState::IDLE || state == HomingState::SUCCESS) {
            return state;
        }

        // Read the raw position of the motor (without offset subtraction)
        int32_t current_motor_pos = encoder_.getMotorPosition();

        // Read sensor pin (proximity sensor)
        GPIO_PinState pinState = HAL_GPIO_ReadPin(gpioPort, gpioPin);
        bool isActive = activeHigh ? (pinState == GPIO_PIN_SET) : (pinState == GPIO_PIN_RESET);

        // 1st edge: Entry into the sensor area
        if (state == HomingState::SEARCHING_ENTER && isActive) {
            p1_motor_pos = current_motor_pos;
            state = HomingState::SEARCHING_EXIT;
        }
        // 2nd edge: Exit from the sensor area
        else if (state == HomingState::SEARCHING_EXIT && !isActive) {
            p2_motor_pos = current_motor_pos;
            
            // 1. Arithmetic mean
            int32_t motor_zero_pos = (p1_motor_pos + p2_motor_pos) / 2;

            // 2. Convert the raw engine position to the offset, taking the gear ratio into account
            int64_t calculated_offset = (static_cast<int64_t>(motor_zero_pos) * 1000LL) / encoder_.getGr();

            // 3. Update Offset
            encoder_.setOffset(static_cast<int32_t>(calculated_offset));

            state = HomingState::SUCCESS;

            // 4. Feedback
            sendHomingResult();
        }

        return state;
    }

    HomingState getState() const { return state; }

private:
    GPIO_TypeDef* gpioPort;
    uint16_t gpioPin;
    Encoder& encoder_;
    CanBus& canBus;
    uint8_t ownNodeId;
    uint8_t sensorId;
    bool activeHigh;

    HomingState state{HomingState::IDLE};
    int32_t p1_motor_pos{0};
    int32_t p2_motor_pos{0};

    void sendHomingResult() {
        PayloadMessage msg{};
        msg.msg_type = 2;                        // Telemetrie / Homing Type
        msg.write = 0;
        msg.echo = 0;
        msg.p1 = static_cast<int32_t>(sensorId);
        msg.p2 = static_cast<int32_t>(state);     // Status (SUCCESS = 3)
        msg.p3 = encoder_.getOffset();            // Send new Offset

        canBus.send(ownNodeId, msg);
    }
};