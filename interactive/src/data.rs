use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
#[derive(Debug)]
#[derive(Clone)]
pub struct Pose {
    pub position: Position,
    pub orientation: Orientation,
    // frame_name :String,
}

#[derive(Serialize, Deserialize)]
#[derive(Debug)]
#[derive(Clone)]
pub struct Position {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    // frame_name :String,
}

#[derive(Serialize, Deserialize)]
#[derive(Debug)]
#[derive(Clone)]
pub struct Orientation {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
    // frame_name :String,
}


#[derive(Serialize, Deserialize)]
#[derive(Debug)]
#[derive(Clone)]
pub struct Poi {
    pub name: String,
    pub pose: Pose,
    // frame_name :String,
}


#[derive(Clone, Debug)]
pub struct RobotParams {
    pub trans_vel: f32,
    pub angular_vel: f32,
}

#[derive(Clone, Debug)]
pub struct RobotState {
    pub state :String,
    pub t: f64,
    pub current_pose: Option<Pose>,
    pub battery: (f64,f64),
    pub params :RobotParams,
}