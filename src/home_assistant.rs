use anyhow::{bail, Result};
use esp_idf_svc::mqtt::client::{EspMqttClient, MqttClientConfiguration, QoS};
use serde::Serialize;
use std::time::Duration;

// --- Configuration ---
const MQTT_BROKER_URL: &str = "mqtt://192.168.1.100:1883"; // <-- TODO: Change this to your MQTT broker
const MQTT_CLIENT_ID: &str = "esp32-switch-controller";
const DEVICE_ID: &str = "esp32_switch_controller_01";
const DEVICE_NAME: &str = "ESP32 Switch Controller";

/// A client to interact with Home Assistant via MQTT.
pub struct HaClient {
    client: EspMqttClient,
}

#[derive(Serialize)]
struct HaDevice<'a> {
    identifiers: [&'a str; 1],
    name: &'a str,
}

#[derive(Serialize)]
struct HaSensorConfig<'a> {
    name: String,
    unique_id: String,
    state_topic: String,
    device: HaDevice<'a>,
}

impl HaClient {
    /// Creates a new `HaClient` and connects to the MQTT broker.
    pub fn new() -> Result<Self> {
        let conf = MqttClientConfiguration {
            client_id: Some(MQTT_CLIENT_ID),
            ..Default::default()
        };
        let (client, _) = EspMqttClient::new_with_conn(MQTT_BROKER_URL, &conf)?;

        log::info!("MQTT client created and connected.");

        Ok(Self { client })
    }

    /// Publishes discovery messages for all 32 binary sensors.
    pub fn publish_discovery(&mut self) -> Result<()> {
        log::info!("Publishing Home Assistant discovery messages...");
        for i in 0..32 {
            let chip = i / 16;
            let pin = i % 16;
            let port = if pin < 8 { 'A' } else { 'B' };
            let pin_num = if pin < 8 { pin } else { pin - 8 };

            let config = HaSensorConfig {
                name: format!("Switch {}_{}{}", chip, port, pin_num),
                unique_id: format!("{}_switch_{}", DEVICE_ID, i),
                state_topic: Self::get_state_topic(i),
                device: HaDevice {
                    identifiers: [DEVICE_ID],
                    name: DEVICE_NAME,
                },
            };

            let topic = format!(
                "homeassistant/binary_sensor/{}_switch_{}/config",
                DEVICE_ID, i
            );
            let payload = serde_json::to_string(&config)?;

            self.client.publish(&topic, QoS::AtLeastOnce, true, payload.as_bytes())?;
            log::info!("Published discovery for sensor {}", i);
            // Small delay to avoid flooding the broker
            std::thread::sleep(Duration::from_millis(50));
        }
        log::info!("Discovery messages published.");
        Ok(())
    }

    /// Publishes the state for a specific sensor.
    pub fn publish_state(&mut self, sensor_index: u8, state: bool) -> Result<()> {
        if sensor_index >= 32 {
            bail!("Invalid sensor index: {}", sensor_index);
        }
        let topic = Self::get_state_topic(sensor_index);
        let payload = if state { "ON" } else { "OFF" };
        self.client.publish(&topic, QoS::AtLeastOnce, true, payload.as_bytes())?;
        Ok(())
    }

    fn get_state_topic(sensor_index: u8) -> String {
        format!("switch-controller/{}/sensor/{}/state", DEVICE_ID, sensor_index)
    }
} 