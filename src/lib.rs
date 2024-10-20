use std::collections::HashMap;
use std::f64::consts::PI;
pub struct Tire {
    pub id: usize,
    pub raito: f64,
}

pub struct Chassis {
    pub fl: Tire,
    pub fr: Tire,
    pub b: Tire,
}

pub struct OmniSetting {
    pub chassis: Chassis,
    pub max_power_input: f64,
    pub max_power_output: f64,
    pub max_revolution: f64,
}

impl OmniSetting {
    fn first_step(&self, linear_x: f64, linear_y: f64) -> [f64; 2] {
        let theta: f64 = linear_y.atan2(linear_x);
        let power: f64 = (linear_x.powf(2.) + linear_y.powf(2.))
            .sqrt()
            .min(self.max_power_input);
        [theta, power]
    }

    pub fn move_chassis(
        &self,
        linear_x: f64,
        linear_y: f64,
        angular_z: f64,
    ) -> HashMap<usize, f64> {
        let _revolution = angular_z;
        let [_theta, _power] = &self.first_step(linear_x, linear_y);

        let mut motor_power: HashMap<usize, f64> = HashMap::new();
        motor_power.insert(
            self.chassis.fr.id,
            (_theta - (PI * 1. / 6.)).sin() * self.chassis.fr.raito,
        );
        motor_power.insert(
            self.chassis.fl.id,
            (_theta - (PI * 5. / 6.)).sin() * self.chassis.fl.raito,
        );
        motor_power.insert(
            self.chassis.b.id,
            (_theta - (PI * 9. / 6.)).sin() * self.chassis.b.raito,
        );

        let standard_power: f64 = {
            [
                motor_power.values().fold(0.0 / 0.0, |m, v| v.max(m)).abs(),
                motor_power.values().fold(0.0 / 0.0, |m, v| v.min(m)).abs(),
            ]
            .iter()
            .fold(0.0 / 0.0, |m, v| v.max(m))
        };

        let mut re_motor_power = HashMap::new();
        for i in motor_power.keys() {
            let mut _tmp = 0.;
            _tmp = self.max_power_output * (_power / self.max_power_input) * motor_power[i]
                / standard_power
                + self.max_power_output * (_revolution / self.max_revolution);

            _tmp = _tmp.max(-self.max_power_output);
            _tmp = _tmp.min(self.max_power_output);

            re_motor_power.insert(*i, _tmp);
        }

        re_motor_power
    }
}

#[test]
fn test() {
    const CHASSIS: Chassis = Chassis {
        fl: Tire { id: 0, raito: 1. },
        fr: Tire { id: 1, raito: 1. },
        b: Tire { id: 2, raito: 1. },
    };

    const MAX_POWER_INPUT: f64 = 1.;
    const MAX_POWER_OUTPUT: f64 = 1.;
    const MAX_REVOLUTION: f64 = 5400.;

    let omni_setting = OmniSetting {
        chassis: CHASSIS,
        max_power_input: MAX_POWER_INPUT,
        max_power_output: MAX_POWER_OUTPUT,
        max_revolution: MAX_REVOLUTION,
    };

    let motor_power = omni_setting.move_chassis(0., 0., 0.);

    let y = motor_power[&1] * (PI * 1. / 6.).sin() - motor_power[&0] * (PI * 5. / 6.).sin();
    let x = motor_power[&1] * (PI * 1. / 6.).cos() - motor_power[&0] * (PI * 5. / 6.).cos()
        + motor_power[&2] * (PI * 9. / 6.).cos();

    println!("{:?}", (x, y));
    let mut keys = motor_power.keys().cloned().collect::<Vec<usize>>();
    keys.sort();

    for key in keys {
        println!("id: {},value: {}", key, motor_power[&key])
    }
}
