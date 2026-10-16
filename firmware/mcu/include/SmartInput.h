#pragma once
#include "gpio.h"
#include "CanBus.h"
#include "CanFrame.h"

enum class InputEvent : uint8_t {
    NONE = 0,
    SINGLE_INPUT = 1,
    DOUBLE_INPUT = 2,
    LONG_INPUT   = 3,
    RELEASE      = 4
};

class SmartInput {
public:
    SmartInput(GPIO_TypeDef* port, uint16_t pin, 
                     CanBus& bus, uint8_t nodeId, uint8_t inputId,
                     bool activeLow = false,
                     uint32_t longInputTimeMs = 400, uint32_t doubleInputTimeMs = 400,
                     uint32_t debounceMs = 25)
        : gpioPort_(port), gpioPin_(pin),
          canBus_(bus), ownNodeId_(nodeId), inputId_(inputId),
          activeLow_(activeLow),
          longInputTimeMs_(longInputTimeMs), doubleInputTimeMs_(doubleInputTimeMs),
          debounceMs_(debounceMs) {}

    InputEvent update() {
        uint32_t now = HAL_GetTick();

        // 1. Read raw GPIO state
        GPIO_PinState pinState = HAL_GPIO_ReadPin(gpioPort_, gpioPin_);
        bool rawState = activeLow_ ? (pinState == GPIO_PIN_RESET) : (pinState == GPIO_PIN_SET);

        // 2. Software Debouncing
        if (rawState != lastRawState_) {
            lastDebounceTime_ = now;
            lastRawState_ = rawState;
        }

        if ((now - lastDebounceTime_) >= debounceMs_) {
            debouncedState_ = rawState;
        }

        InputEvent event = InputEvent::NONE;

        // 3. State Machine
        
        // Edge: Button newly PRESSED
        if (debouncedState_ && !lastState_) {
            inputStartTime_ = now;
            longFired_ = false;

            // Check if this new press happens within doubleClick window of last release
            if (inputPending_ && ((now - releaseTime_) <= doubleInputTimeMs_)) {
                isSecondInput_ = true;
            } else {
                isSecondInput_ = false;
            }
            inputPending_ = false; // Reset pending state on new press
        }
        // Button HELD: Check for Long Press
        else if (debouncedState_ && lastState_) {
            if (!longFired_ && ((now - inputStartTime_) >= longInputTimeMs_)) {
                longFired_ = true;
                isSecondInput_ = false;
                event = InputEvent::LONG_INPUT;
            }
        }
        // Edge: Button RELEASED
        else if (!debouncedState_ && lastState_) {
            releaseTime_ = now;
            event = InputEvent::RELEASE;

            if (!longFired_) {
                if (isSecondInput_) {
                    // This was the release of a double click
                    event = InputEvent::DOUBLE_INPUT;
                    isSecondInput_ = false;
                    inputPending_ = false;
                } else {
                    // Mark as pending single click until timeout or second press
                    inputPending_ = true;
                }
            }
        }
        // Button IS UP: Check timeout for pending Single Click
        else if (!debouncedState_ && inputPending_) {
            if ((now - releaseTime_) > doubleInputTimeMs_) {
                event = InputEvent::SINGLE_INPUT;
                inputPending_ = false;
            }
        }

        lastState_ = debouncedState_;

        // 4. Transmit CAN message on valid events
        if (event != InputEvent::NONE) {
            sendCanMessage(event);
        }

        return event;
    }

private:
    GPIO_TypeDef* gpioPort_;
    uint16_t gpioPin_;
    CanBus& canBus_;
    uint8_t ownNodeId_;
    uint8_t inputId_;
    bool activeLow_;

    uint32_t longInputTimeMs_;
    uint32_t doubleInputTimeMs_;
    uint32_t debounceMs_;

    // Debounce tracking
    bool lastRawState_{false};
    bool debouncedState_{false};
    uint32_t lastDebounceTime_{0};

    // State machine tracking
    bool lastState_{false};
    uint32_t inputStartTime_{0};
    uint32_t releaseTime_{0};
    bool inputPending_{false};
    bool isSecondInput_{false};
    bool longFired_{false};

    void sendCanMessage(InputEvent event) {
        PayloadMessage msg{};
        msg.msg_type = 2; // Type ID for IO/Button Telemetry
        msg.write = 0;
        msg.echo = 0;
        msg.p1 = static_cast<int32_t>(inputId_);
        msg.p2 = static_cast<int32_t>(event);

        canBus_.send(ownNodeId_, msg);
    }
};