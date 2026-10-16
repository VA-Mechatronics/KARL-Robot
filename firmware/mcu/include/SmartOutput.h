#pragma once
#include "gpio.h"

class SmartOutput {
public:
    SmartOutput(GPIO_TypeDef* port, uint16_t pin, 
           uint32_t shortTimeMs = 200, uint32_t longTimeMs = 500, uint32_t pauseTimeMs = 200)
        : gpioPort_(port), gpioPin_(pin),
          shortTimeMs_(shortTimeMs), longTimeMs_(longTimeMs), pauseTimeMs_(pauseTimeMs) {}

    void out(uint8_t smart_event) {
        switch (smart_event)
        {
        case 1:
            shortOut();
            break;
        case 2:
            doubleOut();
            break;
        case 3:
            longOut();
            break;
        default:
            break;
        }
    }

    void shortOut() {
        clearQueue();
        pushSegment(shortTimeMs_, true);
        pushSegment(0, false);
        startNextSegment();
    }

    void longOut() {
        clearQueue();
        pushSegment(longTimeMs_, true);
        pushSegment(0, false);
        startNextSegment();
    }

    void doubleOut() {
        clearQueue();
        pushSegment(shortTimeMs_, true);
        pushSegment(pauseTimeMs_, false);
        pushSegment(shortTimeMs_, true);
        pushSegment(0, false);
        startNextSegment();
    }

    void clearQueue() {
        queueHead_ = 0;
        queueTail_ = 0;
        queueCount_ = 0;
        currentState_ = false;
    }

    /**
     * Sets the pin according to its current state and handles timing.
     * Should be called cyclically in the main loop.
     */
    void update() {
        uint32_t now = HAL_GetTick();

        if (currentState_ && now >= stateEndTime_) {
            startNextSegment();
        } else if (!currentState_ && queueCount_ > 0 && now >= stateEndTime_) {
            startNextSegment();
        }
        HAL_GPIO_WritePin(gpioPort_, gpioPin_, currentState_ ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

private:
    struct Segment {
        uint32_t durationMs;
        bool state;
    };

    GPIO_TypeDef* gpioPort_;
    uint16_t gpioPin_;
    uint32_t shortTimeMs_;
    uint32_t longTimeMs_;
    uint32_t pauseTimeMs_;

    Segment queue_[4];
    uint8_t queueHead_{0};
    uint8_t queueTail_{0};
    uint8_t queueCount_{0};

    bool currentState_{false};
    uint32_t stateEndTime_{0};

    void pushSegment(uint32_t durationMs, bool state) {
        if (queueCount_ < 4) {
            queue_[queueTail_] = {durationMs, state};
            queueTail_ = (queueTail_ + 1) % 4;
            queueCount_++;
        }
    }

    void startNextSegment() {
        if (queueCount_ > 0) {
            Segment seg = queue_[queueHead_];
            queueHead_ = (queueHead_ + 1) % 4;
            queueCount_--;

            currentState_ = seg.state;
            stateEndTime_ = HAL_GetTick() + seg.durationMs;
        } else {
            currentState_ = false;
        }
    }
};