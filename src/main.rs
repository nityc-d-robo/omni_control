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
const  MAX_PAWER_OUTPUT:i32 = 999;

fn main() -> Result<(), DynError>{
    let ctx = Context::new()?;
    let node = ctx.create_node("omni_controll", None, Default::default())?;
    let subscriber = node.create_subscriber::<msg::Twist>("cmd_vel", None)?;
    let publisher = node.create_publisher::<drobo_interfaces::msg::MdLibMsg>("md_driver_topic", None)?;

    // let _logger = Logger::new("omni_controll");

    let mut selector = ctx.create_selector()?;

    
    selector.add_subscriber(
        subscriber,
        {
            Box::new(move |msg| {
                // pr_info!(logger, "receive: {:?}", msg.linear);
                let topic_callback_data = topic_callback(msg);
                // safe_drive::pr_info!(logger,"an:{},pa:{}",topic_callback_data[0],topic_callback_data[1]);
                move_chassis(topic_callback_data[0],topic_callback_data[1],topic_callback_data[2],&publisher);
            })
        },
    );

    loop {
        selector.wait()?;
    }
}


fn topic_callback(msg: subscriber::TakenMsg<Twist>) -> [f64;3]{

    
    let theta:f64 = msg.linear.y.atan2(-msg.linear.x);
    let pawer = (msg.linear.x.powf(2.) + msg.linear.y.powf(2.)).sqrt();
    

    [theta,pawer,msg.angular.z]
    
}

fn move_chassis(_theta:f64, _pawer:f64, _yaw:f64,publisher:&Publisher<MdLibMsg>){

    let mut motor_power:[i32;4] = [0;4];
    let logger = Logger::new("omni_controll");

    


    motor_power[Chassis::FR as usize] = (MAX_PAWER_OUTPUT as f64 * (_pawer/MAX_PAWER_INPUT) *  (_theta-(PI * 1./4.)).sin()   )  as i32; 
    motor_power[Chassis::FL as usize] = (MAX_PAWER_OUTPUT as f64 * (_pawer/MAX_PAWER_INPUT) *  (_theta+(PI * 5./4.)).sin()  )  as i32;
    motor_power[Chassis::BR as usize] = (MAX_PAWER_OUTPUT as f64 * (_pawer/MAX_PAWER_INPUT) *  (_theta+(PI * 1./4.)).sin()  )  as i32;
    motor_power[Chassis::BL as usize] = (MAX_PAWER_OUTPUT as f64 * (_pawer/MAX_PAWER_INPUT) *  (_theta+(PI * 3./4.)).sin()  )  as i32;

    safe_drive::pr_info!(logger,"FL : {} FR : {} BR : {} BL : {}",
    motor_power[Chassis::FL as usize],
    motor_power[Chassis::FR as usize],
    motor_power[Chassis::BR as usize],
    motor_power[Chassis::BL as usize]);



    for i in 0..4 {
        
        send_pwm(i as u32,0,motor_power[i]>0, motor_power[i] as u32,publisher);
    }
    
    
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