#include "main.h"
#include "CanBus.h"
#include "PostionController.h"
#include "Encoder.h"
#include "StepperDriver.h"
#include "UsbCdc.h"
#include "bootloader_jump.h"
#include "Io.h"
#include "SmartOutput.h"
#include "HomingSensor.h"

extern const uint8_t ownNodeId;
extern CanBus can;
extern PositionController controller;
extern Encoder encoder;
extern StepperDriver* driver;
extern UsbCdc usb;
extern SmartOutput toolBuz;
extern HomingSensor homingSensor;

uint16_t lastReceivedTimestamp = 0;
uint16_t getReceived_time = 0;
uint16_t getReceived_duration = 0;

extern "C" uint32_t micros(void);

uint16_t processIncomingCAN(void)
{
    
    static CanFrame canFrame;
    while (can.getReceived(canFrame))
    {
        getReceived_time = micros();

        // --- Gloabal HARD STOP ---
        if (canFrame.id == 0x000) {
            if (canFrame.dlc >= 2) {
                uint8_t cmd = canFrame.data[0];
                uint8_t sub_cmd = canFrame.data[1];

                __disable_irq();
                controller.clearQueue();
                __enable_irq();

                if (cmd == 0x99) {
                    if (sub_cmd == 0x00) {
                        driver->enable(false);
                    }
                    if (sub_cmd == 0x01) {
                        //driver.setVelocity(0);
                    }
                }
                usb.send("HARD STOP");
            }
        }

        // --- Gloabal Motion Broadcast ---
        if (canFrame.id == 0x100) {
            const auto* globalData = reinterpret_cast<const GlobalMotionBroadcast*>(canFrame.data.data());
            uint8_t arrayIdx = ownNodeId - 1;

            if (arrayIdx < 7) {
                int32_t pos = globalData->axes[arrayIdx].set_pos;
                int32_t vel = globalData->axes[arrayIdx].set_vel;
                uint16_t ts = globalData->timestamp;
                uint8_t cmd = globalData->command;
                __disable_irq();
                bool ok = controller.addTrajectoryPoint(pos, vel, ts);
                __enable_irq();

                lastReceivedTimestamp = ts;

                if (!ok) {
                    static char motion_buf[128];
                    snprintf(motion_buf, sizeof(motion_buf), "mot buf: pos:%ld vel:%ld cmd:%d ts:%d\n",
                    pos, vel, cmd, ts);
                    usb.send(motion_buf);
                }
            }
        }
        else if (canFrame.id == (0x200 | (ownNodeId & 0x0F))) {
            if (canFrame.dlc >= 2) {
                const auto* config = reinterpret_cast<const PayloadMessage*>(canFrame.data.data());

                PayloadMessage payload{};
                DriverStatus driver_status{};
                payload.msg_type = config->msg_type;
                payload.echo = lastReceivedTimestamp;

                switch (config->msg_type) {
                    case 0: // TELEMETRY 0
                        switch (config->write) {
                            case 1:
                                driver->enable(config->p1);
                                break;
                            case 2:
                                __disable_irq();
                                encoder.setZero();
                                controller.clearQueue();
                                __enable_irq();
                                break;
                            case 3:
                                __disable_irq();
                                controller.setMode(config->p1);
                                controller.clearQueue();
                                __enable_irq();
                                break;
                            case 4:
                                __disable_irq();
                                homingSensor.startHoming();
                                __enable_irq();
                                break;
                        }
                        uint32_t pos, vel, out, err;
                        __disable_irq();
                        pos = encoder.getPosition();
                        vel = encoder.getVelocity();
                        out = controller.getOutput();
                        err = controller.getError();
                        __enable_irq();
                        payload.p1 = pos;
                        payload.p2 = vel;
                        payload.p3 = out;
                        payload.p4 = err;
                        can.send(ownNodeId, payload);
                        break;
                    case 1: // TELEMETRY 1
                        __disable_irq();
                        driver_status = driver->getStatus();
                        __enable_irq();
                        payload.p1 = driver_status.ok;
                        payload.p2 = driver_status.currentScale;
                        payload.p3 = driver_status.sgResult;
                        payload.p4 = driver_status.overTempPrewarn;
                        can.send(ownNodeId, payload);
                        break;
                    case 2: // Smart IO
                        if (config->write == 1) {
                            // Tooling special Case
                            if (config->p1 == 1) {
                                toolBuz.out(config->p2);
                            }
                        }
                        // Event based, see SmartInput in main.cpp
                        break;
                    case 3: // PID
                        if (config->write == 1) {
                            __disable_irq();
                            controller.setKp(config->p1);
                            controller.setKi(config->p2);
                            controller.setKd(config->p3);
                            controller.setKv(config->p4);
                            __enable_irq();
                        }
                        payload.p1 = controller.getKp();
                        payload.p2 = controller.getKi();
                        payload.p3 = controller.getKd();
                        payload.p4 = controller.getKv();
                        can.send(ownNodeId, payload);
                        break;

                    case 4: // MECHANICS
                        if (config->write == 1) {
                            __disable_irq();
                            controller.setGr(config->p1);
                            encoder.setGr(config->p2);
                            controller.clearQueue();
                            __enable_irq();
                        }
                        payload.p1 = controller.getGr();
                        payload.p2 = encoder.getGr();
                        can.send(ownNodeId, payload);
                        break;

                    case 5: // ENCODER
                        if (config->write == 1) {
                            __disable_irq();
                            encoder.setOffset(config->p1);
                            controller.clearQueue();
                            __enable_irq();
                        }
                        payload.p1 = encoder.getOffset();
                        can.send(ownNodeId, payload);
                        break;

                    case 6: // DRIVER
                        if (config->write == 1) {
                            __disable_irq();
                            driver->setMicroSteps(config->p1);
                            driver->setMaxCurrent(config->p2);
                            __enable_irq();
                        }
                        payload.p1 = driver->getMicroSteps();
                        payload.p2 = driver->getMaxCurrent();
                        can.send(ownNodeId, payload);
                        break;

                    case 7: // GUIDING
                        if (config->write == 1) {
                            __disable_irq();
                            controller.setGuidingGain(config->p1);
                            controller.setGuidingDeadzone(config->p2);
                            controller.setErrorLimit(config->p3);
                            __enable_irq();
                        }
                        payload.p1 = controller.getGuidingGain();
                        payload.p2 = controller.getGuidingDeadzone();
                        payload.p3 = controller.getErrorLimit();
                        can.send(ownNodeId, payload);
                        break;

                    case 8:
                        if (config->write == 1) {
                            Bootloader_TriggerDFU();
                        }
                        break;
                    default:
                        break;
                }
            }
        }
        getReceived_duration = micros() - getReceived_time;
    }
    return getReceived_duration;
}