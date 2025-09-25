use manager::Manager;

pub const imu_port: i32 = 5555;
pub const video_port: i32 = 5556;
pub const loopback_addr: &str = "127.0.0.1";



pub fn main(){
    imu_addr =  format!("{}:{}", loopback_addr, imu_port);
    video_addr = format!("{}:{}", loopback_addr, video_port);
    let manager = Manager::start(imu_addr.as_str(), video_addr.as_str()).unwrap();
   
    imu_listener = manager.imu_listener;
    vid_listener = manager.vid_listener;


    
    loop {
        StabilizationManager.process();
        std::thread::sleep(std::time::Duration::from_secs(60));
    }

}