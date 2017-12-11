#include <cstdio>
#include <AP_HAL/AP_HAL.h>
#include <DataFlash/DataFlash.h>
#include "AP_BattMonitor_SMBus_Maxell_Para.h"

#define NUM_PARA 2
#define I2C_SELECTOR_ADDRESS 0x70
#define I2C_ERROR_RETRY_INTERVAL_MSEC 1000

#define SBS_CMD_TEMPERATURE 0x08
#define SBS_CMD_VOLTAGE 0x09
#define SBS_CMD_CURRENT 0x0a
#define SBS_CMD_REMAINING_CAPACITY 0x0f
#define SBS_CMD_FULL_CHARGE_CAPACITY 0x10
#define SBS_CMD_DESIGN_CAPACITY 0x18
#define SBS_CMD_SPECIFICATION_INFO 0x1a
#define SBS_CMD_SERIAL_NUMBER 0x1c
#define SBS_CMD_MANUFACTURER_NAME 0x20
#define SBS_CMD_DEVICE_NAME  0x21

#define MAXELL_NUM_CELLS 6
#define MAXELL_CMD_CELL_VOLTAGE_1 0x3f

#define MAXELL_CMD_SAFETY_ALERT 0x50
#define MAXELL_CMD_SAFETY_STATUS 0x51
#define MAXELL_CMD_PF_ALERT 0x52
#define MAXELL_CMD_PF_STATUS 0x53

#if 1
#define DEBUG_PRINTF(...) do { std::printf(__VA_ARGS__); } while (0)
#else
#define DEBUG_PRINTF(...)
#endif

AP_BattMonitor_SMBus_Maxell_Para::AP_BattMonitor_SMBus_Maxell_Para(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
: AP_BattMonitor_SMBus(mon, mon_state, std::move(dev))
{
    _slots = new Slot[NUM_PARA]{};
    _dev->register_periodic_callback(500000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_SMBus_Maxell_Para::timer, void));
}

AP_BattMonitor_SMBus_Maxell_Para::~AP_BattMonitor_SMBus_Maxell_Para() {
    delete _slots;
}

static uint8_t
selector_value(uint8_t slot_no)
{
    return 1 << slot_no;
}


bool
AP_BattMonitor_SMBus_Maxell_Para::select(uint8_t slot_no)
{
    _dev->set_address(I2C_SELECTOR_ADDRESS);
    uint8_t sval = selector_value(slot_no);
    if (_dev->transfer(&sval, 1, nullptr, 0)) {
        _state.healthy = true;
    } else {
        if (AP_HAL::millis() - _last_access_msec > AP_BATTMONITOR_SMBUS_TIMEOUT_MICROS / 1000) {
            _state.healthy = false;
            for (int i = 0; i < NUM_PARA; i++) {
                _slots[i] = {};
            }
        }
        return false;
    }
    _dev->set_address(AP_BATTMONITOR_SMBUS_I2C_ADDR);
    return true;
}

bool
AP_BattMonitor_SMBus_Maxell_Para::read_slot(uint8_t slot_no, bool powering)
{
    Slot &slot = _slots[slot_no];
    if (!slot.i2c_healthy && AP_HAL::millis() - slot.last_access_msec < I2C_ERROR_RETRY_INTERVAL_MSEC) {
        return false;
    }
    if (!select(slot_no)) {
        return false;
    }
    slot.last_access_msec = AP_HAL::millis();
    bool good = true;
    _pec_supported = true;
    uint16_t serial;
    if (read_word(SBS_CMD_SERIAL_NUMBER, serial)) {
        if (serial != slot.serial) {
            slot = Slot{};
            slot.serial = serial;
            if (!read_word(SBS_CMD_DESIGN_CAPACITY, slot.design_capacity)) {
                good = false;
            }
        }
    } else {
        good = false;
    }
    slot.powering = powering;
    if (!read_word(SBS_CMD_VOLTAGE, slot.voltage)) {
        good = false;
    }
    uint16_t cur;
    if (read_word(SBS_CMD_CURRENT, cur)) {
        slot.current = static_cast<int16_t>(cur);
    } else {
        good = false;
    }
    uint16_t remain_capa;
    if (read_word(SBS_CMD_REMAINING_CAPACITY, remain_capa)) {
        if (slot.remaining_capacity != 0 && remain_capa < slot.remaining_capacity) {
            uint16_t diff = slot.remaining_capacity - remain_capa;
            slot.consumed_mah += diff;
            _consumed_mah += diff;
        }
        slot.remaining_capacity = remain_capa;
    } else {
        good = false;
    }
    if (!read_word(SBS_CMD_FULL_CHARGE_CAPACITY, slot.full_charge_capacity)) {
        good = false;
    }
    if (!read_word(SBS_CMD_TEMPERATURE, slot.temperature)) {
        good = false;
    }
    uint8_t status[SMBUS_READ_BLOCK_MAXIMUM_TRANSFER];
    if (read_block(MAXELL_CMD_PF_STATUS, status, false) > 0) {
        if (status[0] != 0) {
            slot.fault = true;
        }
//        DEBUG_PRINTF("PF_Status(%u) = %02x%02x%02x%02x\n", slot_no, status[3], status[2], status[1], status[0]);
    } else {
        good = false;
    }
    for (int i = 0; i < MAXELL_NUM_CELLS; i++) {
        if (!read_word(MAXELL_CMD_CELL_VOLTAGE_1 - i, slot.cell_voltages[i])) {
            good = false;
        }
    }
    if (good) {
        slot.last_read_msec = AP_HAL::millis();
        slot.i2c_healthy = true;
    } else {
        if (slot.i2c_healthy && AP_HAL::millis() - slot.last_read_msec > AP_BATTMONITOR_SMBUS_TIMEOUT_MICROS / 1000) {
            slot.i2c_healthy = false;
            slot.reset_readings();
        }
    }
    return true;
}

void
AP_BattMonitor_SMBus_Maxell_Para::timer()
{
    if (_slots == nullptr) return;
    uint32_t now = AP_HAL::millis();
    if (!_state.healthy && now - _last_access_msec < I2C_ERROR_RETRY_INTERVAL_MSEC) {
        return;
    }
    uint8_t rval = 0x30; // slot 0 and 1 is powering
    if (_mon._volt_pin[_state.instance] >= 0) {
        _dev->set_address(I2C_SELECTOR_ADDRESS);
        if (!_dev->transfer(nullptr, 0, &rval, 1)) {
            _state.healthy = false;
            return;
        }
    }
    for (int i = 0; i < NUM_PARA; i++) {
        if (!read_slot(i, (rval & (0x10 << i))) != 0) {
            if (!_state.healthy) { // I2C selector not responding
                break;
            }
        }
    }
    if (!_state.healthy) {
        return;
    }
    _state.voltage = synth_voltage();
    _state.current_amps = synth_current_amps();
    _state.current_total_mah = synth_current_total_mah();
    _state.temperature = synth_temperature();
    _state.temperature_time = is_zero(_state.temperature + 273.0f) ? 0 : AP_HAL::millis();
    _state.is_powering_off = false;
    submit_cell_voltages();
    float capa = synth_pack_capacity();
    if (!is_zero(_mon._pack_capacity[_state.instance] - capa)) {
        _mon._pack_capacity[_state.instance].set_and_notify(capa);
    }
}

static bool
is_usable(const AP_BattMonitor_SMBus_Maxell_Para::Slot &slot)
{
    return slot.i2c_healthy && !slot.fault && slot.powering;
}

float
AP_BattMonitor_SMBus_Maxell_Para::synth_voltage() const {
    uint16_t max_vol = 0;
    for (int i = 0; i < NUM_PARA; i++) {
        if (is_usable(_slots[i]) && _slots[i].voltage > max_vol) {
            max_vol = _slots[i].voltage;
        }
    }
    return to_voltage(max_vol);
}

float
AP_BattMonitor_SMBus_Maxell_Para::synth_current_amps() const {
    int32_t total_current = 0;
    for (int i = 0; i < NUM_PARA; i++) {
        if (is_usable(_slots[i])) {
            total_current += _slots[i].current;
        }
    }
    return -to_ampare(total_current);
}

float
AP_BattMonitor_SMBus_Maxell_Para::synth_current_total_mah() const {
    return to_mah(_consumed_mah);
}

float
AP_BattMonitor_SMBus_Maxell_Para::synth_temperature() const {
    uint16_t max_temp = 0;
    for (int i = 0; i < NUM_PARA; i++) {
        if (is_usable(_slots[i]) && _slots[i].temperature > max_temp) {
            max_temp = _slots[i].temperature;
        }
    }
    return to_celsius(max_temp);
}

void
AP_BattMonitor_SMBus_Maxell_Para::submit_cell_voltages() {
    for (int i = 0; i < NUM_PARA; i++) {
        if (is_usable(_slots[i])) {
            for (int s = 0; s < MAXELL_NUM_CELLS; s++) {
                _state.cell_voltages.cells[s] = _slots[i].cell_voltages[s];
            }
            break;
        }
    }
}

float
AP_BattMonitor_SMBus_Maxell_Para::synth_pack_capacity() const {
    uint32_t total_capa = 0;
    for (int i = 0; i < NUM_PARA; i++) {
        if (is_usable(_slots[i])) {
            total_capa += _slots[i].full_charge_capacity;
        }
    }
    return to_mah(total_capa);
}

float
AP_BattMonitor_SMBus_Maxell_Para::synth_pack_remaining() const {
    uint32_t total_remains = 0;
    for (int i = 0; i < NUM_PARA; i++) {
        if (is_usable(_slots[i])) {
            total_remains += _slots[i].remaining_capacity;
        }
    }
    return to_mah(total_remains);
}

float
AP_BattMonitor_SMBus_Maxell_Para::synth_design_capacity() const {
    float max_capacity = 0;
    for (int i = 0; i < NUM_PARA; i++) {
        if (_slots[i].design_capacity != 0) {
            float capa = to_mah(_slots[i].design_capacity);
            if (capa > max_capacity) {
                max_capacity = capa;
            }
        }
    }
    return max_capacity * NUM_PARA;
}

uint8_t
AP_BattMonitor_SMBus_Maxell_Para::capacity_remaining_pct() const {
    float capa = synth_design_capacity();
    if (is_zero(capa)) {
        return 0.0f;
    }
    return synth_pack_remaining() / capa * 100;
}

void
AP_BattMonitor_SMBus_Maxell_Para::read() {
}

void
AP_BattMonitor_SMBus_Maxell_Para::write_log() const
{
#if NUM_PARA >= 1
    slot_log(_slots[0], LOG_CURRENT_MSG);
#endif
#if NUM_PARA >= 2
    slot_log(_slots[1], LOG_CURRENT2_MSG);
#endif
}

void
AP_BattMonitor_SMBus_Maxell_Para::slot_log(Slot &slot, uint8_t msg_no) const
{
    struct log_Current output = {
            LOG_PACKET_HEADER_INIT(msg_no),
            time_us: AP_HAL::micros64(),
            battery_voltage: to_voltage(slot.voltage),
            current_amps: -to_ampare(slot.current),
            current_total: to_mah(slot.consumed_mah),
            temperature: static_cast<int16_t>(to_celsius(slot.temperature) * 100)
    };
    // check battery structure can hold all cells
    static_assert(MAXELL_NUM_CELLS <= (sizeof(output.cell_voltages) / sizeof(output.cell_voltages[0])),
                  "Battery cell number doesn't match in library and log structure");

    for (uint8_t i = 0; i < MAXELL_NUM_CELLS; i++) {
        output.cell_voltages[i] = slot.cell_voltages[i];
    }
    if (MAXELL_NUM_CELLS + 3 <= sizeof(output.cell_voltages) / sizeof(output.cell_voltages[0])) {
        output.cell_voltages[MAXELL_NUM_CELLS + 0] = to_mah(slot.design_capacity);
        output.cell_voltages[MAXELL_NUM_CELLS + 1] = to_mah(slot.full_charge_capacity);
        output.cell_voltages[MAXELL_NUM_CELLS + 2] = to_mah(slot.remaining_capacity);
    }

    DataFlash_Class::instance()->WriteBlock(&output, sizeof(output));
}

float
AP_BattMonitor_SMBus_Maxell_Para::to_voltage(uint16_t val) const
{
    return val * 1e-3f;
}

float AP_BattMonitor_SMBus_Maxell_Para::to_ampare(int32_t val) const
{
    return val * get_I_scale() * 1e-3f;
}
float AP_BattMonitor_SMBus_Maxell_Para::to_mah(uint32_t val) const
{
    return val * get_I_scale();
}

float AP_BattMonitor_SMBus_Maxell_Para::to_celsius(uint16_t val) const
{
    return val * 1e-1f - 273.0f;
}

float AP_BattMonitor_SMBus_Maxell_Para::capacity_remaining_mah() const
{
    return synth_pack_remaining();
}
