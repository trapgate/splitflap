/*
   Copyright 2020 Scott Bezek and the splitflap contributors

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
#pragma once

#include "config.h"
#include "logger.h"
#include "src/splitflap_module_data.h"
#include "../proto_gen/splitflap.pb.h"

#include "task.h"

enum class SplitflapMode {
    MODE_RUN,
    MODE_SENSOR_TEST,
};

struct Settings {
    bool force_full_rotation;
    uint8_t max_moving;
    uint32_t start_delay_millis;
    PB_Settings_AnimationStyle animation_style;
};

struct SplitflapModuleState {
    State state;
    uint8_t flap_index;
    bool moving;
    bool home_state;
    uint8_t count_unexpected_home;
    uint8_t count_missed_home;

    bool operator==(const SplitflapModuleState& other) {
        return state == other.state
            && flap_index == other.flap_index
            && moving == other.moving
            && home_state == other.home_state
            && count_unexpected_home == other.count_unexpected_home
            && count_missed_home == other.count_missed_home;
    }

    bool operator!=(const SplitflapModuleState& other) {
        return !(*this == other);
    }
};

struct SplitflapState {
    SplitflapMode mode;
    Settings settings;
    SplitflapModuleState modules[NUM_MODULES];

#ifdef CHAINLINK
    bool loopbacks_ok = false;
#endif

    bool operator==(const SplitflapState& other) {
        for (uint8_t i = 0; i < NUM_MODULES; i++) {
            if (modules[i] != other.modules[i]) {
                return false;
            }
        }

        return mode == other.mode
#ifdef CHAINLINK
            && loopbacks_ok == other.loopbacks_ok
#endif
            ;
    }

    bool operator!=(const SplitflapState& other) {
        return !(*this == other);
    }
};

enum class LedMode {
    AUTO,
    MANUAL,
};

enum class CommandType {
    MODULES,
    SENSOR_TEST_SET,
    SENSOR_TEST_CLEAR,
    CONFIG,
};

struct ModuleConfig {
    uint8_t target_flap_index;
    uint8_t movement_nonce;
    uint8_t reset_nonce;
};

struct ModuleConfigs {
    Settings settings;
    uint8_t module_count;
    ModuleConfig config[NUM_MODULES];
};

struct Command {
    CommandType command_type;
    union CommandData {
        uint8_t module_command[NUM_MODULES];
        ModuleConfigs module_configs;
    };
    CommandData data;
};

struct Motions {
    uint8_t target_flap_index[NUM_MODULES];
    PB_Settings_AnimationStyle anim_style;
    uint8_t pos;
};

#define QCMD_NO_OP          0
#define QCMD_RESET_AND_HOME 1
#define QCMD_LED_ON         2
#define QCMD_LED_OFF        3
#define QCMD_DISABLE        4
#define QCMD_FLAP           5

class SplitflapTask : public Task<SplitflapTask> {
    friend class Task<SplitflapTask>; // Allow base Task to invoke protected run()

    public:
        SplitflapTask(const uint8_t task_core, const LedMode led_mode);
        ~SplitflapTask();
        
        SplitflapState getState();

        void showString(const char *str, uint8_t length);
        void resetAll();
        void disableAll();
        void setLed(uint8_t id, bool on);
        void setSensorTest(bool sensor_test);
        void setLogger(Logger* logger);
        void postRawCommand(Command command);

    protected:
        void run();

    private:
        const LedMode led_mode_;
        const SemaphoreHandle_t state_semaphore_;
        QueueHandle_t queue_;
        Command queue_receive_buffer_ = {};
        Logger* logger_;

        bool all_stopped_ = true;
        
        uint8_t start_orders_[_PB_Settings_AnimationStyle_MAX+1][NUM_MODULES] = {
            {}, 
            {},
            {5,17,6,18,4,16,7,19,3,15,8,20,2,14,9,21,1,13,10,22,0,12,11,23},
            {0,11,12,23,1,10,13,22,2,9,14,21,3,8,15,20,4,7,16,19,5,6,17,18},
            {0,11,1,12,2,13,3,14,4,15,5,16,6,17,7,18,8,19,9,20,10,21,11,22},
            {23,11,22,10,21,9,20,8,19,7,18,6,17,5,16,4,15,3,14,2,13,1,12,0},
        };
        Settings settings_ = {
            .force_full_rotation = FORCE_FULL_ROTATION,
            .max_moving = 0,
            .start_delay_millis = 0,
            .animation_style = PB_Settings_AnimationStyle_LEFT_TO_RIGHT,
        };

        uint8_t moving_ = 0;
        uint32_t last_sensor_print_millis_ = 0;
        uint32_t last_module_start_millis_ = 0;
        bool sensor_test_ = SENSOR_TEST;
        Motions next_motion_ = {};
        ModuleConfigs current_configs_ = {};

#ifdef CHAINLINK
        uint8_t loopback_current_out_index_ = 0;
        uint16_t loopback_step_index_ = 0;
        bool loopback_current_ok_ = true;
        bool loopback_all_ok_ = false;
#endif

        // Cached state. Protected by state_semaphore_
        SplitflapState state_cache_;
        void updateStateCache();

        void processQueue();
        void setStartOrders();
        void startModules();
        void runUpdate();
        void sensorTestUpdate();
        void log(const char* msg);

        int8_t findFlapIndex(uint8_t character);
};
