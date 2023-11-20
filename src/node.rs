use embedded_hal::adc::OneShot;
use rp_pico::hal::adc::{Adc, TempSense};

use usecop::Result;

pub type SecNode = usecop::node::SecNode<MyModules, 6>;

pub fn create(adc: Adc, sensor: TempSense) -> SecNode {
    SecNode::new("rpi", "RPi Pico W5500 demo", MyModules {
        temp: Temp { adc, sensor, internals: Default::default() },
    })
}

#[derive(usecop_derive::Modules)]
pub struct MyModules {
    temp: Temp,
}

#[derive(usecop_derive::Module)]
#[secop(interface = "Readable")]
#[secop(description = "test")]
#[secop(param(name = "value", doc = "main value of the temperature",
              readonly = true,
              datainfo(double(unit="degC"))))]
#[secop(param(name = "status", doc = "status of readout",
              readonly = true,
              datainfo(tuple(
                  member(enum_(
                      member(name="idle", value=100),
                      member(name="busy", value=300),
                      member(name="error", value=400),
                  )),
                  member(str(maxchars=8)),
              ))))]
#[secop(command(name = "buzz", doc = "buzz it!",
                argtype(double(unit="Hz")),
                restype(null())))]
struct Temp {
    internals: usecop::node::ModuleInternals,
    adc: Adc,
    sensor: TempSense,
}

impl Temp {
    fn read_value(&mut self) -> Result<f64> {
        let refv = 3.3;
        let adc_value: u16 = self.adc.read(&mut self.sensor).unwrap();
        let vbe: f64 = f64::from(adc_value) * refv / 4096.0;
        Ok(27f64 - (vbe - 0.706) / 0.001721)
    }

    fn read_status(&mut self) -> Result<(i32, &str)> {
        Ok((100, "all good, trust me"))
    }

    fn do_buzz(&mut self, _arg: f64) -> Result<()> {
        Ok(())
    }
}
