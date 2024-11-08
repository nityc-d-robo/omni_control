use ndarray::prelude::{arr1, arr2};
use std::collections::HashMap;
const MIN:f64 = 60.;

pub struct Tire {
    pub id: usize,
    pub radius: f64,// m
}

pub struct Chassis {
    pub f: Tire,
    pub bl: Tire,
    pub br: Tire,
}

pub struct OmniSetting {
    pub chassis: Chassis,
    pub radius: f64, // m
}

impl OmniSetting {
    fn ms_rpm(&self,radius:f64,ms:f64) -> f64 {
        (ms/radius)*MIN
    }

    pub fn move_chassis(
        &self,
        linear_x: f64,
        linear_y: f64,
        angular_z: f64,
    ) -> HashMap<usize, f64> {
        // 回転成分はあと
        let control_matrixn = arr2(&[
            [1., 0.,self.radius],
            [-1. / 2., 3_f64.sqrt() / 2.,self.radius],
            [-1. / 2., -1. * 3_f64.sqrt() / 2.,self.radius],
        ]);

        let vx_vy_az = arr1(&[linear_x, linear_y,angular_z]);

        // m/s
        let v1_v2_v3 = control_matrixn.dot(&vx_vy_az);

        let mut motor_power = HashMap::new();
        motor_power.insert(self.chassis.f.id, self.ms_rpm(self.chassis.f.radius, v1_v2_v3[[0]]));
        motor_power.insert(self.chassis.bl.id, self.ms_rpm(self.chassis.bl.radius, v1_v2_v3[[1]]));
        motor_power.insert(self.chassis.br.id, self.ms_rpm(self.chassis.f.radius, v1_v2_v3[[2]]));

        motor_power

        // Ratio adjustment
    }
}

#[test]
fn test() {
    const CHASSIS: Chassis = Chassis {
        bl: Tire { id: 0, radius: 0.1 },
        br: Tire { id: 1, radius: 0.1 },
        f: Tire { id: 2, radius:  0.1 },
    };

    let omni_setting = OmniSetting {
        chassis: CHASSIS,
        radius: 0.5,
    };

    println!("{:?}", omni_setting.move_chassis(1., 0., 0.))
}
