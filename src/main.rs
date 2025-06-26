mod home_assistant;

use std::sync::{Arc, Mutex};
use std::thread;

use anyhow::Result;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::*;
use esp_idf_hal::i2c::*;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use mcp23017::{MCP23017, Register};
use crate::home_assistant::HaClient;

type SharedI2c<'a> = Arc<Mutex<I2cDriver<'a>>>;
type SharedMcp<'a> = Arc<Mutex<MCP23017<SharedI2c<'a>>>>;

fn main() -> Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Starting up...");

    let peripherals = Peripherals::take().unwrap();
    let i2c = peripherals.i2c0;
    let sda = peripherals.pins.gpio4;
    let scl = peripherals.pins.gpio5;

    let config = I2cConfig::new().baudrate(100.kHz().into());
    let i2c_driver = I2cDriver::new(i2c, sda, scl, &config)?;

    let i2c_bus = Arc::new(Mutex::new(i2c_driver));

    log::info!("Configuring MCP23017 at 0x20");
    let mcp1 = Arc::new(Mutex::new(MCP23017::new(i2c_bus.clone(), 0x20)));
    configure_mcp(&mut mcp1.lock().unwrap())?;

    log::info!("Configuring MCP23017 at 0x21");
    let mcp2 = Arc::new(Mutex::new(MCP23017::new(i2c_bus.clone(), 0x21)));
    configure_mcp(&mut mcp2.lock().unwrap())?;

    // Create a channel to signal the interrupt
    let (tx, rx) = std::sync::mpsc::channel::<()>();

    // Setup interrupt pin
    let mut intr_pin = PinDriver::input(peripherals.pins.gpio0)?;
    intr_pin.set_pull(Pull::Up)?;
    intr_pin.set_interrupt_type(InterruptType::Falling)?;

    // Subscribe the interrupt. When it fires, send a message on the channel.
    let subscription = unsafe {
        intr_pin.subscribe(move || {
            // This is the ISR. It should be short.
            // We just send a message and return.
            // A non-blocking send is used to prevent blocking in the ISR.
            let _ = tx.try_send(());
        })?
    };
    // The subscription must be kept alive.
    std::mem::forget(subscription);

    intr_pin.enable_interrupt()?;

    // Initialize Home Assistant client
    let mut ha_client = HaClient::new()?;
    ha_client.publish_discovery()?;

    // Spawn a thread to handle interrupts and publish MQTT messages
    let handler_mcp1 = mcp1.clone();
    let handler_mcp2 = mcp2.clone();
    let mut handler_ha_client = HaClient::new()?;

    thread::spawn(move || {
        log::info!("Interrupt handler thread started.");
        loop {
            // Wait for a signal from the ISR
            if let Ok(_) = rx.recv() {
                log::info!("Interrupt received! Checking pin states.");

                // Check first MCP23017
                if let Err(e) = check_and_publish_states(&handler_mcp1, 0, &mut handler_ha_client) {
                    log::error!("Error checking MCP1: {:?}", e);
                }

                // Check second MCP23017
                if let Err(e) = check_and_publish_states(&handler_mcp2, 1, &mut handler_ha_client) {
                    log::error!("Error checking MCP2: {:?}", e);
                }
            }
        }
    });


    log::info!("Configuration complete. Waiting for interrupts.");

    // The main thread can sleep or do other things.
    loop {
        FreeRtos::delay_ms(1000);
    }
}

/// Reads the interrupt capture and GPIO registers to determine which pins changed state,
/// and publishes the new state to Home Assistant.
fn check_and_publish_states(mcp_arc: &SharedMcp, chip_index: u8, ha_client: &mut HaClient) -> Result<()> {
    let mut mcp = mcp_arc.lock().unwrap();

    // Read which pins caused the interrupt on port A and B
    let intcap_a = mcp.read_intcapa()?;
    let intcap_b = mcp.read_intcapb()?;
    let int_pins = (intcap_b as u16) << 8 | intcap_a as u16;

    if int_pins == 0 {
        // This can happen if the interrupt flag was cleared before we could read it.
        // Reading the GPIO registers will clear the interrupt. Let's do a full read.
        log::warn!("Interrupt triggered but INTCAP was clear. Reading all GPIOs for chip {}.", chip_index);
        let porta_val = mcp.read_gpioa()?;
        let portb_val = mcp.read_gpiob()?;
        let all_pins = (portb_val as u16) << 8 | porta_val as u16;
        for i in 0..16 {
             let sensor_index = chip_index * 16 + i;
             let state = (all_pins >> i) & 1 == 1;
             ha_client.publish_state(sensor_index, state)?;
        }
        return Ok(());
    }

    // Read the current GPIO values
    // NOTE: This read action also clears the interrupt flags on the MCP23017
    let gpio_a = mcp.read_gpioa()?;
    let gpio_b = mcp.read_gpiob()?;

    for i in 0..16 {
        // If this pin triggered the interrupt...
        if (int_pins >> i) & 1 == 1 {
            let sensor_index = chip_index * 16 + i;
            // ...get its current state
            let state = if i < 8 {
                (gpio_a >> i) & 1 == 1
            } else {
                (gpio_b >> (i - 8)) & 1 == 1
            };
            log::info!("Pin {} on chip {} changed to state {}", i, chip_index, state);
            // ...and publish it.
            ha_client.publish_state(sensor_index, state)?;
        }
    }

    Ok(())
}

/// Configures an MCP23017 for input with pull-ups and interrupt-on-change.
fn configure_mcp(mcp: &mut MCP23017<SharedI2c>) -> Result<()> {
    // Configure all pins as input
    mcp.write_register(Register::IODIRA, 0xFF)?;
    mcp.write_register(Register::IODIRB, 0xFF)?;

    // Enable pull-up resistors for all pins
    mcp.write_register(Register::GPPUA, 0xFF)?;
    mcp.write_register(Register::GPPUB, 0xFF)?;

    // Enable interrupt-on-change for all pins
    mcp.write_register(Register::GPINTENA, 0xFF)?;
    mcp.write_register(Register::GPINTENB, 0xFF)?;

    // Configure interrupts to trigger on change from previous value
    mcp.write_register(Register::INTCONA, 0x00)?;
    mcp.write_register(Register::INTCONB, 0x00)?;

    // Mirror INTA and INTB pins (OR logic)
    // This makes it so an interrupt on either port will trigger both INT pins.
    mcp.write_register(Register::IOCONA, 0b0100_0000)?;
    mcp.write_register(Register::IOCONB, 0b0100_0000)?;

    // Clear any existing interrupts by reading the capture registers
    mcp.read_intcapa()?;
    mcp.read_intcapb()?;

    Ok(())
}
