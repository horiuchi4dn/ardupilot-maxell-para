#pragma once

#include <stdbool.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_SMBus.h"

#define AP_BATTMONITOR_MAXELL_MAX_CELLS 6

class AP_BattMonitor_SMBus_Maxell_Para : public AP_BattMonitor_SMBus {
public:
    // The BattMonitor_State structure is filled in by the backend driver
    struct Slot {
        uint32_t last_read_msec;
        uint32_t last_access_msec;
        uint16_t consumed_mah;
        uint16_t voltage; // mV
        int16_t current; // mA
        uint16_t design_capacity; // mAh
        uint16_t full_charge_capacity; // mAh
        uint16_t remaining_capacity; // mAh
        uint16_t cell_voltages[AP_BATTMONITOR_MAXELL_MAX_CELLS]; // mV
        uint16_t temperature; // 10 * K
        uint16_t serial;
        bool i2c_healthy:1;            // battery monitor is communicating correctly
        bool fault:1;    // true if the battery has permament failure
        bool powering:1; // true if battery is powering, false if battery internal fuse has blown

        void reset_readings() {
            consumed_mah = voltage = current = design_capacity = full_charge_capacity = remaining_capacity = temperature = serial = 0;
            memset(cell_voltages, 0, sizeof(cell_voltages));
            fault = false;
            powering = false;
        }
    };

    AP_BattMonitor_SMBus_Maxell_Para(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);
    virtual ~AP_BattMonitor_SMBus_Maxell_Para();
    virtual void init() override {}
    void write_log() const;
    uint8_t capacity_remaining_pct() const override;
    virtual float capacity_remaining_mah() const override;

private:
    Slot *_slots;
    uint32_t _last_access_msec = 0;
    uint8_t _current_slot = 255;
    uint32_t _consumed_mah = 0; // consumed since boot (mAh)

    void read(void) override;
    void timer();
    bool read_slot(uint8_t slot, bool powering);
    bool select(uint8_t slot);
    float synth_voltage() const;
    float synth_current_amps() const;
    float synth_current_total_mah() const;
    float synth_temperature() const;
    void submit_cell_voltages();
    float synth_pack_capacity() const;
    float synth_pack_remaining() const;
    float synth_design_capacity() const;
    uint8_t get_I_scale() const { return 2; }
    float to_voltage(uint16_t) const;
    float to_ampare(int32_t) const;
    float to_mah(uint32_t) const;
    float to_celsius(uint16_t) const;
    void slot_log(Slot &slot, uint8_t msg_no) const;
};
