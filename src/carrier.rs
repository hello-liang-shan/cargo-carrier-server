use std::f32::consts::FRAC_PI_4;
use std::sync::{Arc, Mutex};

use esp_idf_hal::ledc::LedcDriver;

#[derive(Copy, Clone, Debug)]
pub struct Signal {
    x: u8,
    y: u8,
    r: u8,
}

pub struct Motor<'d> {
    in1: LedcDriver<'d>,
    in2: LedcDriver<'d>,
    min: u32,
}

pub struct Carrier<'d> {
    lf: Motor<'d>,
    rf: Motor<'d>,
    lr: Motor<'d>,
    rr: Motor<'d>,
}

impl<'d> Carrier<'d> {
    pub fn new(
        lf: Motor<'d>,
        rf: Motor<'d>,
        lr: Motor<'d>,
        rr: Motor<'d>,
    ) -> Arc<Mutex<Self>> {
        Arc::new(Mutex::new(
            Self { lf, rf, lr, rr }
        ))
    }

    pub fn handle(&mut self, signal: Signal) -> anyhow::Result<()> {
        let thrust = Thrust::from(signal);
        println!("Received signal: {:?}, updating thrust: {:?}", signal, thrust);

        self.lf.update(thrust.lf)?;
        self.rf.update(thrust.rf)?;
        self.lr.update(thrust.lr)?;
        self.rr.update(thrust.rr)?;

        Ok(())
    }

    pub fn stop(&mut self) -> anyhow::Result<()> {
        self.handle(Signal::default()).unwrap();
        Ok(())
    }
}

impl<'d> Motor<'d> {
    pub fn new(
        in1: LedcDriver<'d>,
        in2: LedcDriver<'d>,
    ) -> anyhow::Result<Self> {
        Ok(
            Self { in1, in2, min: 0 }
        )
    }

    pub fn min(&mut self, min: u32) {
        self.min = min;
    }

    pub fn update(&mut self, thrust: f32) -> anyhow::Result<()> {
        match thrust {
            v if v > 0.0 => self.forward(thrust)?,
            v if v < 0.0 => self.backward(thrust.abs())?,
            _ => self.stop()?,
        };

        Ok(())
    }

    fn stop(&mut self) -> anyhow::Result<()> {
        self.in1.set_duty(0)?;
        self.in2.set_duty(0)?;
        Ok(())
    }

    fn forward(&mut self, percent: f32) -> anyhow::Result<()> {
        self.in1.set_duty(self.duty(percent, self.in1.get_max_duty()))?;
        self.in2.set_duty(0)?;
        Ok(())
    }

    fn backward(&mut self, percent: f32) -> anyhow::Result<()> {
        self.in2.set_duty(self.duty(percent, self.in2.get_max_duty()))?;
        self.in1.set_duty(0)?;
        Ok(())
    }

    fn duty(&self, thrust: f32, max: u32) -> u32 {
        (((max as f32) * thrust).ceil() as u32).min(max).max(self.min)
    }
}

impl From<[u8; 3]> for Signal {
    fn from(value: [u8; 3]) -> Self {
        Self {
            x: value[0],
            y: value[1],
            r: value[2],
        }
    }
}

impl Default for Signal {
    fn default() -> Self {
        Self {
            x: 0,
            y: 0,
            r: 0,
        }
    }
}

#[derive(Copy, Clone, Debug)]
struct Thrust {
    lf: f32,
    rf: f32,
    lr: f32,
    rr: f32,
}

impl From<Signal> for Thrust {
    fn from(value: Signal) -> Self {
        let x = ((value.x as i8) as f32) / 127.0;
        let y = ((value.y as i8) as f32) / 127.0;
        let r = ((value.r as i8) as f32) / 127.0;

        let p = x.hypot(y);
        let t = y.atan2(x) - FRAC_PI_4;

        let sin = t.sin();
        let cos = t.cos();
        let max = sin.abs().max(cos.abs());

        let fix = p + r.abs();

        let mut lf = p * cos / max + r;
        let mut rf = p * sin / max - r;
        let mut lr = p * sin / max + r;
        let mut rr = p * cos / max - r;

        if fix > 1.0 {
            lf /= fix;
            rf /= fix;
            lr /= fix;
            rr /= fix;
        }

        Self { lf, rf, lr, rr }
    }
}
