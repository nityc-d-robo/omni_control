use ndarray::prelude::{arr1, arr2};
use std::collections::HashMap;
pub struct Tire {
    pub id: usize,
    pub raito: f64,
}

pub struct Chassis {
    pub f: Tire,
    pub bl: Tire,
    pub br: Tire,
}

pub struct OmniSetting {
    pub chassis: Chassis,
    pub max_power_input: f64,
    pub max_power_output: f64,
    pub radius: f64,
}

impl OmniSetting {
    fn first_step(&self, linear_x: f64, linear_y: f64) -> f64 {
        let power: f64 = (linear_x.powf(2.) + linear_y.powf(2.))
            .sqrt()
            .min(self.max_power_input);

        power
    }

    pub fn move_chassis(
        &self,
        linear_x: f64,
        linear_y: f64,
        angular_z: f64,
    ) -> HashMap<usize, f64> {
        // 回転成分はあと
        let control_matrixn = arr2(&[
            [1., 0.],
            [-1. / 2., 3_f64.sqrt() / 2.],
            [-1. / 2., -1. * 3_f64.sqrt() / 2.],
        ]);

        let vx_vy = arr1(&[linear_x, linear_y]);
        let v1_v2_v3 = control_matrixn.dot(&vx_vy);

        let mut motor_power = HashMap::new();
        motor_power.insert(self.chassis.f.id, v1_v2_v3[[0]] * self.chassis.f.raito);
        motor_power.insert(self.chassis.bl.id, v1_v2_v3[[1]] * self.chassis.bl.raito);
        motor_power.insert(self.chassis.br.id, v1_v2_v3[[2]] * self.chassis.br.raito);

        println!("{:?}", motor_power);

        // Ratio adjustment
        {
            let _power = self.first_step(linear_x, linear_y);

            let standard_power: f64 = {
                [
                    motor_power.values().fold(0.0 / 0.0, |m, v| v.max(m)).abs(),
                    motor_power.values().fold(0.0 / 0.0, |m, v| v.min(m)).abs(),
                ]
                .iter()
                .fold(0.0 / 0.0, |m, v| v.max(m))
            };

            println!("{:?}", standard_power);

            let mut re_motor_power = HashMap::new();
            for i in motor_power.keys() {
                let mut _tmp = 0.;

                _tmp = if standard_power != 0. {
                    self.max_power_output * (_power / self.max_power_input) * motor_power[i]
                        / standard_power
                } else {
                    0.
                } + self.radius * angular_z;


                _tmp = _tmp.max(-self.max_power_output);
                _tmp = _tmp.min(self.max_power_output);

                re_motor_power.insert(*i, _tmp);
            }
            re_motor_power
        }
    }
}

#[test]
fn test() {
    const MAX_PAWER_INPUT: f64 = 160.;
    const MAX_PAWER_OUTPUT: f64 = 999.;
    const CHASSIS: Chassis = Chassis {
        bl: Tire { id: 0, raito: 1. },
        br: Tire { id: 1, raito: 1. },
        f: Tire { id: 2, raito: 1. },
    };

    let omni_setting = OmniSetting {
        chassis: CHASSIS,
        max_power_input: MAX_PAWER_INPUT,
        max_power_output: MAX_PAWER_OUTPUT,
        radius: 0.5,
    };

    println!("{:?}", omni_setting.move_chassis(0., 0., 5400.))
}
