use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};

use crate::{
    backend::{Event, Task, TaskId},
    data::{Location, Poi, RobotState},
    dvrp,
};

#[derive(Debug)]
pub struct TemporalPlan {
    pub generation :usize,
    pub battery_profile: Vec<(f64, f64)>,
    pub drive_activities: Vec<(PoiTime, PoiTime)>,
    pub inspection_activities: Vec<(f64, f64)>,
}

#[derive(Debug)]
pub struct PoiTime {
    pub poi: Poi,
    pub time: f64,
}

#[derive(Debug)]
pub struct PlanJob {
    pub pois: Vec<Poi>,
    pub robot_state: RobotState,
    pub dock: Option<Poi>,
}

pub struct Planner {
    pub plan: Option<TemporalPlan>,
    n_pois: usize,
    prev_state: Option<RobotState>,
    tx_job: std::sync::mpsc::Sender<PlanJob>,
    rx_plan: std::sync::mpsc::Receiver<TemporalPlan>,
    planner_running: std::sync::Arc<std::sync::atomic::AtomicBool>,
}

pub fn estimated_distance(a: &Location, b: &Location) -> f64 {
    let dx = a.pose.x - b.pose.x;
    let dy = a.pose.y - b.pose.y;
    let dz = a.pose.z - b.pose.z;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

pub fn estimated_travel_time(dist: f64) -> f64 {
    dist
}

pub fn estimate_battery_usage(dist :f64) -> f64 {
    dist * 0.1
}

impl Planner {
    pub fn new() -> Self {
        let (tx_job, rx_job) = std::sync::mpsc::channel();
        let (tx_plan, rx_plan) = std::sync::mpsc::channel();

        let planner_running = Arc::new(false.into());
        let planner_running_thread = Arc::clone(&planner_running);
        std::thread::spawn(move || {
            planner_thread(planner_running_thread, rx_job, move |p| {
                tx_plan.send(p).unwrap()
            })
        });

        Planner {
            plan: None,
            n_pois: 0,
            prev_state: None,
            tx_job,
            rx_plan,
            planner_running,
        }
    }

    pub fn get_plan(&self) -> &Option<TemporalPlan> {
        &self.plan
    }

    pub fn dispatch(&mut self, mut run: impl FnMut(Task) -> TaskId) {
        todo!()
    }

    pub fn update_event(&mut self, ev: Event) {
        todo!()
    }

    pub fn is_planner_running(&self) -> bool {
        self.planner_running.load(Ordering::Relaxed)
    }

    pub fn process_messages(&mut self) {
        while let Ok(x) = self.rx_plan.try_recv() {
            println!("NEW PLAN {:?}", x);
            self.plan = Some(x);
        }
    }

    pub fn update_state(&mut self, pois: &[Poi], dock :&Option<Poi>, next_state: &RobotState) {
        let mut need_to_plan = false;

        if self.n_pois != pois.len() {
            self.n_pois = pois.len();
            need_to_plan = true;
        }

        if let Some(prev_state) = self.prev_state.as_ref() {
            need_to_plan |= unexpected_location(prev_state, next_state)
                || unexpected_battery(prev_state, next_state);
        } else {
            need_to_plan = true;
        }

        self.prev_state = Some(next_state.clone());

        if need_to_plan {
            let job = PlanJob {
                pois: pois.to_vec(),
                robot_state: next_state.clone(),
                dock: dock.clone(),
            };
            self.tx_job.send(job).ok();
        }
    }
}

fn unexpected_location(prev_state: &RobotState, next_state: &RobotState) -> bool {
    if let (Some(prev_loc), Some(new_loc)) =
        (prev_state.location.as_ref(), next_state.location.as_ref())
    {
        if estimated_distance(prev_loc, new_loc) > 10.0 {
            // Robot jumped 10 m. Replan.
            return true;
        }
    }
    false
}

fn unexpected_battery(prev_state: &RobotState, next_state: &RobotState) -> bool {
    if prev_state.battery.0 > 0. || next_state.battery.0 > 0. {
        let ref_batt = prev_state.battery.0.max(next_state.battery.0);
        if (prev_state.battery.0 - next_state.battery.0) / ref_batt > 0.1 {
            // Battery changed by over 10%. Replan.
            return true;
        }
    }
    false
}

fn planner_thread(
    planner_running: Arc<AtomicBool>,
    get_job: std::sync::mpsc::Receiver<PlanJob>,
    mut set_plan: impl FnMut(TemporalPlan),
) {
    let mut counter = 0;
    loop {
        let job = {
            let mut job = match get_job.recv() {
                Ok(job) => job,
                Err(_) => return,
            };

            while let Ok(next_job) = get_job.try_recv() {
                job = next_job;
            }
            job
        };

        planner_running.store(true, Ordering::Relaxed);
        let mut result = dvrp::dvrp(job);
        planner_running.store(false, Ordering::Relaxed);
        result.generation = counter;
        counter += 1;
        set_plan(result);
    }
}
