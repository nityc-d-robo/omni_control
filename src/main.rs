use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    pr_info,
    topic::publisher::Publisher,
};

use drobo_interfaces::msg::MdLibMsg;
use safe_drive::msg::common_interfaces::geometry_msgs::msg;
mod lib;

use lib::{Chassis, OmniSetting, Tire};

// const OMNI_DIA:f64 =  0.1;
const CHASSIS: Chassis = Chassis {
    bl: Tire { id: 0, radius: 0.1 },
    br: Tire { id: 1, radius: 0.1 },
    f: Tire { id: 2,  radius: 0.1 },
};

fn main() -> Result<(), DynError> {
    let omni_setting = OmniSetting {
        chassis: CHASSIS,
        radius: 0.5,
    };

    // for debug

    let ctx = Context::new()?;
    let node = ctx.create_node("omni_control", None, Default::default())?;
    let subscriber = node.create_subscriber::<msg::Twist>("cmd_vel", None)?;
    let publisher =
        node.create_publisher::<drobo_interfaces::msg::MdLibMsg>("md_driver_topic", None)?;
    let mut selector = ctx.create_selector()?;

    selector.add_subscriber(subscriber, {
        Box::new(move |msg| {
            let _logger = Logger::new("omni_controll");

            let motor_power =
                omni_setting.move_chassis(-msg.linear.x, -msg.linear.y, -msg.angular.z);
            pr_info!(
                _logger,
                "{:?}",
                &[msg.linear.x, msg.linear.y, msg.angular.z]
            );

            pr_info!(_logger, "{:?}", &motor_power);

            for i in 0..=2 as usize {
                send_pwm(
                    i as u32,
                    0,
                    motor_power[&i] >= 0.,
                    motor_power[&i].abs() as u32,
                    &publisher,
                );
            }
        })
    });

    loop {
        selector.wait()?;
    }
}

fn send_pwm(
    _address: u32,
    _semi_id: u32,
    _phase: bool,
    _power: u32,
    publisher: &Publisher<MdLibMsg>,
) {
    let mut msg = drobo_interfaces::msg::MdLibMsg::new().unwrap();
    msg.address = _address as u8;
    msg.semi_id = _semi_id as u8;
    msg.mode = 3 as u8; //MotorLibのPWMモードに倣いました
    msg.phase = _phase as bool;
    msg.power = _power as i16;

    publisher.send(&msg).unwrap()
}
