#![allow(dead_code)]

use esp32_nimble::{BLEAdvertisementData, BLEDevice, NimbleProperties, uuid128};
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::ledc::{LedcDriver, LedcTimerDriver};
use esp_idf_hal::ledc::config::TimerConfig;
use esp_idf_hal::prelude::Peripherals;

use crate::carrier::{Carrier, Motor, Signal};

mod carrier;

fn main() {
    let peripherals = Peripherals::take().expect("Cannot take peripherals.");

    let config = TimerConfig::default();
    let timer = LedcTimerDriver::new(peripherals.ledc.timer0, &config).expect("Cannot create pwm timer driver.");

    let carrier = Carrier::new(
        Motor::new(
            LedcDriver::new(peripherals.ledc.channel0, &timer, peripherals.pins.gpio23).unwrap(),
            LedcDriver::new(peripherals.ledc.channel1, &timer, peripherals.pins.gpio22).unwrap(),
        ).unwrap(),
        Motor::new(
            LedcDriver::new(peripherals.ledc.channel2, &timer, peripherals.pins.gpio16).unwrap(),
            LedcDriver::new(peripherals.ledc.channel3, &timer, peripherals.pins.gpio17).unwrap(),
        ).unwrap(),
        Motor::new(
            LedcDriver::new(peripherals.ledc.channel4, &timer, peripherals.pins.gpio13).unwrap(),
            LedcDriver::new(peripherals.ledc.channel5, &timer, peripherals.pins.gpio12).unwrap(),
        ).unwrap(),
        Motor::new(
            LedcDriver::new(peripherals.ledc.channel6, &timer, peripherals.pins.gpio26).unwrap(),
            LedcDriver::new(peripherals.ledc.channel7, &timer, peripherals.pins.gpio27).unwrap(),
        ).unwrap(),
    );

    let service_uuid = uuid128!("00000000-0000-0000-0000-000000000000");
    let characteristic_uuid = uuid128!("00000000-0000-0000-0000-000000000001");

    let device = BLEDevice::take();
    let server = device.get_server();
    let advertising = device.get_advertising();

    server.on_connect(move |_, connection| {
        println!("Connected: {:?}", connection);
    });

    let stop_the_car = carrier.clone();
    server.on_disconnect(move |connection, _| {
        stop_the_car.lock().unwrap().stop().unwrap();
        println!("Disconnected: {:?}", connection);
    });

    let service = server.create_service(service_uuid);
    let characteristic = service.lock().create_characteristic(
        characteristic_uuid,
        NimbleProperties::WRITE,
    );

    characteristic.lock().on_write(move |args| {
        let payload: [u8; 3] = args.recv_data().try_into().unwrap();
        carrier.lock().unwrap().handle(Signal::from(payload)).unwrap();
    });

    advertising.lock().set_data(
        BLEAdvertisementData::new().name("cargo-carrier-server").add_service_uuid(service_uuid)
    ).expect("Cannot set advertisement data.");

    advertising.lock().start().expect("Cannot start advertising.");

    loop {
        FreeRtos::delay_ms(10);
    }
}
