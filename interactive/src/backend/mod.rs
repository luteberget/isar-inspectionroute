use crate::data::{Location,  RobotState};

pub mod isar;

#[derive(Clone, Copy, Hash, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct TaskId(pub usize);

#[derive(Debug, Clone)]

#[allow(unused)]
pub enum Task {
    Inspect(Location),
    Charge(Location)
}

#[derive(Debug, Clone)]

pub enum Event {
    RobotState(RobotState),
    Task(TaskId, TaskEvent),
}

#[derive(Debug, Clone, Copy)]
#[allow(unused)]
pub enum TaskEvent {
    Success,
    Failure,
}

pub trait Backend {
    fn set_mission(&mut self, task: &[Task]) -> Vec<TaskId>;
    fn try_recv_event(&mut self) -> Option<(std::time::Instant, Event)>;
    fn backend_name(&self) -> &str;
    fn current_robot_state(&self) -> &RobotState;
}
