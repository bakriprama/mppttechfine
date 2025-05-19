#pragma once
#include "esphome.h"

class TechfineMPPT : public Component {
 public:
  UARTComponent *uart_;

  // Sensor yang akan diterbitkan
  Sensor *pv_voltage_sensor = new Sensor();
  Sensor *charge_current_sensor = new Sensor();
  Sensor *battery_voltage_sensor = new Sensor();
  Sensor *charge_power_sensor = new Sensor();
  Sensor *pv_power_sensor = new Sensor();
  Sensor *pv_current_sensor = new Sensor();
  Sensor *battery_soc_sensor = new Sensor();
  Sensor *battery_cells_sensor = new Sensor();
  Sensor *battery_type_sensor = new Sensor();
  Sensor *battery_temp_sensor = new Sensor();
  Sensor *mcu_temp_sensor = new Sensor();
  Sensor *heatsink_temp_sensor = new Sensor();
  Sensor *dc_load_percent_sensor = new Sensor();
  Sensor *daily_energy_sensor = new Sensor();
  Sensor *total_energy_sensor = new Sensor();
  Sensor *error_value_sensor = new Sensor();
  Sensor *bulk_voltage_sensor = new Sensor();
  Sensor *float_voltage_sensor = new Sensor();
  Sensor *low_cutoff_voltage_sensor = new Sensor();

  BinarySensor *dc_load_switch = new BinarySensor();
  BinarySensor *mppt_charging_switch = new BinarySensor();
  BinarySensor *battery_full_switch = new BinarySensor();
  BinarySensor *charge_limit_switch = new BinarySensor();

  TextSensor *software_version_sensor = new TextSensor();
  TextSensor *machine_type_sensor = new TextSensor();

  TechfineMPPT(UARTComponent *uart) { uart_ = uart; }

  void setup() override {
    // Nothing needed here for now
  }

  void loop() override {
    static unsigned long last_time = 0;
    unsigned long now = millis();
    if (now - last_time < 5000) // setiap 5 detik
      return;
    last_time = now;

    // Kirim perintah request data
    static const uint8_t req[] = {0x43, 0x4D, 0x48, 0x42, 0x06, 0x4D, 0x50, 0x50, 0x54, 0x48, 0x42};
    uart_->write_array(req, sizeof(req));
    delay(300);

    if (uart_->available() < 34)
      return;

    std::vector<uint8_t> buffer;
    while (uart_->available()) {
      buffer.push_back(uart_->read());
    }

    if (buffer.size() < 34)
      return;
    if (!(buffer[0] == 0x4D && buffer[1] == 0x56 && buffer[2] == 0x4D && buffer[3] == 0x50))
      return;

    int len = buffer[4];
    if (buffer.size() < 5 + len)
      return;

    uint8_t checksum = 0;
    for (int i = 0; i < len - 1; i++) {
      checksum += buffer[5 + i];
    }
    checksum &= 0xFF;
    if (checksum != buffer[5 + len - 1])
      return;

    auto get16 = [&](int l, int h) -> int {
      return buffer[5 + l] + (buffer[5 + h] << 8);
    };

    int pv_voltage = get16(10, 11);
    float charge_current = buffer[5 + 18] + buffer[5 + 54] / 100.0;
    float battery_voltage = get16(14, 15) / 10.0;
    float charge_power = battery_voltage * charge_current;
    float pv_power = pv_voltage >= 1 ? charge_power * 1.05 : 0.0;
    float pv_current = pv_voltage >= 1 ? pv_power / pv_voltage : 0.0;
    float daily_energy = (((get16(22, 23) * 1000) + get16(24, 25)) / 1000.0);
    float total_energy = ((get16(28, 29) * 10 + buffer[5 + 27]) / 10.0);

    pv_voltage_sensor->publish_state(pv_voltage);
    charge_current_sensor->publish_state(charge_current);
    battery_voltage_sensor->publish_state(battery_voltage);
    charge_power_sensor->publish_state(charge_power);
    pv_power_sensor->publish_state(pv_power);
    pv_current_sensor->publish_state(pv_current);
    battery_soc_sensor->publish_state(buffer[5 + 12]);
    battery_cells_sensor->publish_state(buffer[5 + 13]);
    battery_type_sensor->publish_state(buffer[5 + 16]);
    battery_temp_sensor->publish_state(buffer[5 + 17]);
    mcu_temp_sensor->publish_state(buffer[5 + 19]);
    heatsink_temp_sensor->publish_state(buffer[5 + 20]);
    dc_load_percent_sensor->publish_state(buffer[5 + 21]);
    daily_energy_sensor->publish_state(daily_energy);
    total_energy_sensor->publish_state(total_energy);
    error_value_sensor->publish_state(buffer[5 + 33]);
    bulk_voltage_sensor->publish_state(buffer[5 + 46] / 10.0);
    float_voltage_sensor->publish_state(buffer[5 + 47] / 10.0);
    low_cutoff_voltage_sensor->publish_state(buffer[5 + 45] / 10.0);

    dc_load_switch->publish_state((buffer[5 + 1] & 1) != 0);
    mppt_charging_switch->publish_state((buffer[5 + 1] & 2) != 0);
    battery_full_switch->publish_state((buffer[5 + 1] & 4) != 0);
    charge_limit_switch->publish_state((buffer[5 + 1] & 8) != 0);

    char version_str[10];
    snprintf(version_str, sizeof(version_str), "%d.%d", buffer[5 + 8], buffer[5 + 9]);
    software_version_sensor->publish_state(version_str);
    char machine_type_str[10];
    snprintf(machine_type_str, sizeof(machine_type_str), "%d", buffer[5 + 26]);
    machine_type_sensor->publish_state(machine_type_str);
  }
};
