#[allow(unused_imports)]
use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    msg::common_interfaces::geometry_msgs::msg::Twist,
    topic::{publisher::Publisher, subscriber},
};

use motor_lib::{
    USBHandle,
    Error,
    md,
    GrpcHandle
};

use safe_drive::msg::common_interfaces::geometry_msgs::msg;
use safe_drive::pr_info;
use std::f64::consts::PI;
use std::{cell::RefCell, rc::Rc};

// 構造体の宣言
struct Tire {
    id: usize,
    raito: f64,
}
// 4輪
struct Chassis {
    fl: Tire,
    fr: Tire,
    br: Tire,
    bl: Tire,
}
// 構造体を作成
const CHASSIS: Chassis = Chassis {
    fl: Tire { id: 1, raito: 1. },
    fr: Tire { id: 0, raito: 1. },
    br: Tire { id: 3, raito: 1. },
    bl: Tire { id: 2, raito: 1. },
};

// 定数の宣言
const MAX_PAWER_INPUT: f64 = 160.;
const MAX_PAWER_OUTPUT: f64 = 999.;
const MAX_REVOLUTION: f64 = 5400.;

fn main() -> Result<(), DynError> {
    let handle = GrpcHandle::new("http://127.0.0.1:50051");

    let _logger = Logger::new("robot_2");

    let ctx = Context::new()?;
    let node = ctx.create_node("robot_2", None, Default::default())?;
    let subscriber = node.create_subscriber::<msg::Twist>("cmd_vel", None)?;
    let mut selector = ctx.create_selector()?;

    selector.add_subscriber(subscriber, {
        Box::new(move |msg| {
            let topic_callback_data = topic_callback(msg);
            move_chassis(
                topic_callback_data[0],
                topic_callback_data[1],
                topic_callback_data[2],
                &handle,
            );
        })
    });
    loop {
        selector.wait()?;
    }
}

fn topic_callback(msg: subscriber::TakenMsg<Twist>) -> [f64; 3] {
    // for debug

    let theta: f64 = msg.linear.y.atan2(-msg.linear.x);
    let pawer: f64 = (msg.linear.x.powf(2.) + msg.linear.y.powf(2.))
        .sqrt()
        .min(MAX_PAWER_INPUT);

    [theta, pawer, msg.angular.z]
}

fn move_chassis(_theta: f64, _pawer: f64, _revolution: f64, handle: &GrpcHandle) {
    // for debug
    let _logger = Logger::new("robot_2");

    let mut motor_power: [f64; 4] = [0.; 4];
    motor_power[CHASSIS.fr.id] = (_theta - (PI * 1. / 4.)).sin() * CHASSIS.fr.raito;
    motor_power[CHASSIS.fl.id] = (_theta + (PI * 5. / 4.)).sin() * CHASSIS.fl.raito;
    motor_power[CHASSIS.br.id] = (_theta + (PI * 1. / 4.)).sin() * CHASSIS.br.raito;
    motor_power[CHASSIS.bl.id] = (_theta + (PI * 3. / 4.)).sin() * CHASSIS.bl.raito;

    let standard_power: f64 = {
        [
            motor_power.iter().fold(0.0 / 0.0, |m, v| v.max(m)).abs(),
            motor_power.iter().fold(0.0 / 0.0, |m, v| v.min(m)).abs(),
        ]
        .iter()
        .fold(0.0 / 0.0, |m, v| v.max(m))
    };

    for i in 0..motor_power.len() {
        motor_power[i] = MAX_PAWER_OUTPUT * (_pawer / MAX_PAWER_INPUT) * motor_power[i]
            / standard_power
            + MAX_PAWER_OUTPUT * (_revolution / MAX_REVOLUTION);

        motor_power[i] = motor_power[i].max(-MAX_PAWER_OUTPUT);
        motor_power[i] = motor_power[i].min(MAX_PAWER_OUTPUT);

        // pr_info!(_logger, "here!");
        md::send_pwm(handle, i as u8, motor_power[i] as i16);
        // md::send_speed(handle, i as u8, motor_power[i] as i16);
    }

    pr_info!(
        _logger,
        "fl : {} fr : {} br : {} bl : {} PA : {} ø : {}",
        motor_power[CHASSIS.fr.id],
        motor_power[CHASSIS.fl.id],
        motor_power[CHASSIS.br.id],
        motor_power[CHASSIS.bl.id],
        _pawer,
        _theta / PI * 180.
    );
}