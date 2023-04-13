use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Pose {
    pub position: Position,
    pub orientation: Orientation,
    // pub frame: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Position {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    // pub frame_name: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Orientation {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
    // pub frame_name: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Location {
    pub name: String,
    pub pose: Pose,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct PlanStep {
    pub location: Location,
    pub time :f64,
    pub remaining_battery: f64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct PlanStatus {
    pub status_msg :String,
    pub solver: String,
    pub total_cost: f64,
    pub robot_plans: Vec<Vec<PlanStep>>,
    pub plan_version :u64,
    pub plan_reason :Option<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct BatteryConstraint {
    pub charger_location: Option<Location>,
    pub battery_distance: f64,
    pub remaining_distance: f64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct RobotParams {
    pub speed :f64,
    pub rotation_speed :f64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct RobotState {
    pub parameters :RobotParams,
    pub battery_constraint: BatteryConstraint,
    pub current_location: Option<Location>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum WaypointStatus {
    Pending,
    Success,
    Failure,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Waypoint {
    pub status: WaypointStatus,
    pub is_charger: bool,
    pub location: Location,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ControllerStatus {
    pub waypoints: Vec<Waypoint>,
    pub robots: Vec<RobotState>,
    pub plan: PlanStatus,
}
