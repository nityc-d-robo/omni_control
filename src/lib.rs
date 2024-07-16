use std::collections::HashMap;
use std::f64::consts::PI;
pub struct Tire {
    pub id: usize,
    pub raito: f64,
}

pub struct Chassis {
    pub fl: Tire,
    pub fr: Tire,
    pub br: Tire,
    pub bl: Tire,
}

pub struct OmniSetting {
    pub chassis: Chassis,
    pub max_pawer_input: f64,
    pub max_pawer_output: f64,
    pub max_revolution: f64,
}

impl OmniSetting {
    fn first_step(&self, linear_x: f64, linear_y: f64) -> [f64; 2] {
        let theta: f64 = linear_y.atan2(-linear_x);
        let power: f64 = (linear_x.powf(2.) + linear_y.powf(2.))
            .sqrt()
            .min(self.max_pawer_input);
        [theta, power]
    }

    pub fn move_chassis(&self, linear_x: f64, linear_y: f64, angular_z: f64) -> HashMap<usize, f64> {
        let _revolution = angular_z;
        let [_theta, _pawer] = &self.first_step(linear_x, linear_y);

        let mut motor_power: HashMap<usize, f64> = HashMap::new();
        motor_power.insert(
            self.chassis.fr.id,
            (_theta - (PI * 1. / 4.)).sin() * self.chassis.fr.raito,
        );
        motor_power.insert(
            self.chassis.fl.id,
            (_theta + (PI * 5. / 4.)).sin() * self.chassis.fl.raito,
        );
        motor_power.insert(
            self.chassis.br.id,
            (_theta + (PI * 1. / 4.)).sin() * self.chassis.br.raito,
        );
        motor_power.insert(
            self.chassis.bl.id,
            (_theta + (PI * 3. / 4.)).sin() * self.chassis.bl.raito,
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
            _tmp = self.max_pawer_output * (_pawer / self.max_pawer_input) * motor_power[i]
                / standard_power
                + self.max_pawer_output * (_revolution / self.max_revolution);

            _tmp = _tmp.max(-self.max_pawer_output);
            _tmp = _tmp.min(self.max_pawer_output);

            re_motor_power.insert(*i, _tmp);
        }

        re_motor_power
    }
}
