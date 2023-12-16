use safe_drive::{
    context::Context, error::DynError, logger::Logger, msg::common_interfaces::geometry_msgs::msg::Twist, topic::{subscriber, publisher::Publisher},
};

use safe_drive::msg::common_interfaces::geometry_msgs::msg;
use drobo_interfaces::msg::MdLibMsg;
use std::f64::consts::PI;

enum Chassis {
    FL = 0,
    FR = 1,
    BR = 2, 
    BL = 3,
}


const OMNI_DIA:f64 =  0.1;

fn main() -> Result<(), DynError>{
    let ctx = Context::new()?;
    let node = ctx.create_node("omni_controll", None, Default::default())?;
    let subscriber = node.create_subscriber::<msg::Twist>("cmd_vel", None)?;
    let publisher = node.create_publisher::<drobo_interfaces::msg::MdLibMsg>("md_driver_topic", None)?;

    let _logger = Logger::new("omni_controll");

    let mut selector = ctx.create_selector()?;

    
    selector.add_subscriber(
        subscriber,
        {
            Box::new(move |msg| {
                // pr_info!(logger, "receive: {:?}", msg.linear);
                let topic_callback_data = topic_callback(msg);
                move_chassis(topic_callback_data[0],topic_callback_data[1],topic_callback_data[2],&publisher);
            })
        },
    );

    loop {
        selector.wait()?;
    }
}


fn topic_callback(msg: subscriber::TakenMsg<Twist>) -> [f64;3] {

    let xrpm:f64 = msg.linear.x / (2.0 * PI * OMNI_DIA) * 60.0;
    let yrpm:f64 = msg.linear.y / (2.0 * PI * OMNI_DIA) * 60.0;
    let yaw:f64 = msg.angular.z;

    return [xrpm,yrpm,yaw];
}

fn move_chassis(_x:f64, _y:f64, _yaw:f64,publisher:&Publisher<MdLibMsg>){

    let mut motor_power:[i32;4] = [0;4];

    let power_level: f64 =(_x * _x + _y * _y).sqrt();

    if power_level == 0.0 {

        for i in 0..4{
            send_pwm(i,0,false,0,publisher);
        }
    }

    let yaw = 
        if (_yaw * 100.0) > 50.0{
            0.5
        }
        else if (_yaw * 100.0) < -50.0{
            -0.5
        }
        else{
            _yaw
        };

    motor_power[Chassis::BL as usize] = (-(PI / 4.0).sin() * _x + (PI / 4.0).cos() * _y + yaw * 10.0) as i32;
    motor_power[Chassis::BR as usize] = (-(3.0 * PI / 4.0).sin() * _x + (3.0 * PI / 4.0).cos() * _y + yaw * 10.0) as i32;
    motor_power[Chassis::FR as usize] = (-(5.0 * PI / 4.0).sin() * _x + (5.0 * PI / 4.0).cos() * _y + yaw * 10.0) as i32;
    motor_power[Chassis::FL as usize] = (-(7.0 * PI / 4.0).sin() * _x + (7.0 * PI / 4.0).cos() * _y + yaw * 10.0) as i32;

    for i in 0..4 {
        let mut _power :i32 ;
        let mut _phase:bool;

        if motor_power[i] > 0 {
          _phase = false;
          _power = motor_power[i];
        } else {
          _phase = true;
          _power = -motor_power[i];
        }

        send_pwm(i as u32,0,_phase, _power,publisher);
    }
    
}

fn send_pwm(_address:u32, _semi_id:u32,_phase:bool,_power:i32,publisher:&Publisher<MdLibMsg>){
    let mut msg = drobo_interfaces::msg::MdLibMsg::new().unwrap();
    msg.address = _address as u8;
    msg.semi_id = _semi_id as u8;
    msg.mode = 2 as u8; //MotorLibのPWMモードに倣いました
    msg.phase = _phase as bool;
    msg.power = _power as u16;

    publisher.send(&msg).unwrap()

    
}