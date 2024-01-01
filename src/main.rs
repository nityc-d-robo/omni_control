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


// const OMNI_DIA:f64 =  0.1;
const  MAX_PAWER_INPUT:f64 = 160.;
const  MAX_PAWER_OUTPUT:f64 = 999.;

fn main() -> Result<(), DynError>{

    // for debug
    let _logger = Logger::new("omni_controll");


    let ctx = Context::new()?;
    let node = ctx.create_node("omni_control", None, Default::default())?;
    let subscriber = node.create_subscriber::<msg::Twist>("cmd_vel", None)?;
    let publisher = node.create_publisher::<drobo_interfaces::msg::MdLibMsg>("md_driver_topic", None)?;
    let mut selector = ctx.create_selector()?;
    
    selector.add_subscriber(
        subscriber,
        {
            Box::new(move |msg| {
                let topic_callback_data = topic_callback(msg);

                // safe_drive::pr_info!(_logger,"{}",topic_callback_data[2]);

                move_chassis(topic_callback_data[0],topic_callback_data[1],topic_callback_data[2],&publisher);
            })
        },
    );

    loop {
        selector.wait()?;
    }
}


fn topic_callback(msg: subscriber::TakenMsg<Twist>) -> [f64;3]{
    // for debug
    let _logger = Logger::new("omni_controll");
    
    let theta:f64 = msg.linear.y.atan2(-msg.linear.x);
    let pawer:f64 = (msg.linear.x.powf(2.) + msg.linear.y.powf(2.)).sqrt().min(MAX_PAWER_INPUT);
    

    [theta,pawer,msg.angular.z]
    
}

fn move_chassis(_theta:f64, _pawer:f64, _revolution:f64,publisher:&Publisher<MdLibMsg>){

    // for debug
    let _logger = Logger::new("omni_controll");


    let mut motor_power:[f64;4] = [0.;4];

    motor_power[Chassis::FR as usize] = (_theta-(PI * 1./4.)).sin(); 
    motor_power[Chassis::FL as usize] = (_theta+(PI * 5./4.)).sin();
    motor_power[Chassis::BR as usize] = (_theta+(PI * 1./4.)).sin();
    motor_power[Chassis::BL as usize] = (_theta+(PI * 3./4.)).sin();



    let standard_power:f64 = {
            [
                motor_power.iter().fold(0.0/0.0, |m, v| v.max(m)).abs(),
                motor_power.iter().fold(0.0/0.0, |m, v| v.min(m)).abs()

            ].iter().fold(0.0/0.0, |m, v| v.max(m)) 
        };

    for i in 0..motor_power.len() {

        motor_power[i] = MAX_PAWER_OUTPUT * (_pawer/MAX_PAWER_INPUT) * motor_power[i]/standard_power 
                                                    + MAX_PAWER_OUTPUT*(_revolution/MAX_PAWER_INPUT);

        // 
        motor_power[i] = motor_power[i].max(-MAX_PAWER_OUTPUT);
        motor_power[i] = motor_power[i].min(MAX_PAWER_OUTPUT);

        send_pwm(i as u32,0,motor_power[i]>0., motor_power[i] as u32,publisher);
    }

    // safe_drive::pr_info!(_logger,"FL : {} FR : {} BR : {} BL : {} PA : {} ø : {}",
    //     motor_power[Chassis::FL as usize],
    //     motor_power[Chassis::FR as usize],
    //     motor_power[Chassis::BR as usize],
    //     motor_power[Chassis::BL as usize],
    //     _pawer,
    //     _theta/PI*180.
    // );
}

fn send_pwm(_address:u32, _semi_id:u32,_phase:bool,_power:u32,publisher:&Publisher<MdLibMsg>){
    let mut msg = drobo_interfaces::msg::MdLibMsg::new().unwrap();
    msg.address = _address as u8;
    msg.semi_id = _semi_id as u8;
    msg.mode = 2 as u8; //MotorLibのPWMモードに倣いました
    msg.phase = _phase as bool;
    msg.power = _power as u16;

    publisher.send(&msg).unwrap()

}