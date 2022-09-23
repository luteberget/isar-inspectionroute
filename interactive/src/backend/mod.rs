use crate::{
    data::{Poi, RobotState},
    planner::PoiSequence,
};

pub mod isar;

#[derive(Debug, Clone)]

pub enum Event {
    TaskStart(String),
    TaskEnd(String, bool),
}

pub trait Backend {
    fn set_plan(&mut self, plan_counter: usize, plan: &PoiSequence, dock: &Option<Poi>);
    fn try_recv_event(&mut self) -> Option<(std::time::Instant, Event)>;
    fn backend_description(&self) -> &str;
    fn current_robot_state(&self) -> &RobotState;
}
