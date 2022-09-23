use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};

use crate::{
    backend::Event,
    data::{Pose, Poi, RobotState},
    solver, app::PendingPoi,
};

#[derive(Debug)]
pub struct TemporalPlan {
    pub generation: usize,
    pub battery_profile: Vec<(f64, f64)>,
    pub drive_activities: Vec<(PoiTime, PoiTime)>,
    pub inspection_activities: Vec<(f64, f64)>,
}

pub type PoiSequence = Vec<Vec<Poi>>;

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
    pub plan: Option<PoiSequence>,
    n_pois: usize,
    n_docks: usize,
    prev_state: Option<RobotState>,
    tx_job: std::sync::mpsc::Sender<PlanJob>,
    rx_plan: std::sync::mpsc::Receiver<PoiSequence>,
    planner_running: std::sync::Arc<std::sync::atomic::AtomicBool>,
    pub plan_counter: usize,
}

pub fn estimated_distance(a: &Pose, b: &Pose) -> f64 {
    let dx = a.position.x - b.position.x;
    let dy = a.position.y - b.position.y;
    let dz = a.position.z - b.position.z;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

pub fn estimated_travel_time(dist: f64) -> f64 {
    dist
}

pub fn estimate_battery_usage(dist: f64) -> f64 {
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
            n_docks: 0,
            prev_state: None,
            tx_job,
            rx_plan,
            planner_running,
            plan_counter: 0,
        }
    }

    pub fn get_temporal_plan(
        &self,
        robot_state: &RobotState,
        dock: &Option<Poi>,
    ) -> Option<TemporalPlan> {
        self.plan
            .as_ref()
            .map(|p| integrate_plan(p, robot_state, dock))
        // &self.plan
    }

    pub fn get_plan_sequence(&self) -> (usize, &Option<PoiSequence>) {
        (self.plan_counter, &self.plan)
    }

    pub fn update_event(&mut self, ev: &Event) {
        match ev {
            Event::TaskEnd(poi, _) => {
                if let Some(seqs) = self.plan.as_mut() {
                    for seq in seqs.iter_mut() {
                        seq.retain(|p| &p.name != poi);
                    }
                }

                // A POI succeeded or failed. We expect one less POI in the next update.
                if self.n_pois > 0 {
                    self.n_pois -= 1;
                }
            }
            Event::TaskStart(_) => {},
        }
    }

    pub fn is_planner_running(&self) -> bool {
        self.planner_running.load(Ordering::Relaxed)
    }

    pub fn process_messages(&mut self) {
        while let Ok(x) = self.rx_plan.try_recv() {
            println!("NEW PLAN {:?}", x);
            self.plan = Some(x);
            self.plan_counter += 1;
        }
    }

    pub fn update_state(&mut self, pois: &[PendingPoi], dock: &Option<Poi>, next_state: &RobotState) {
        let mut need_to_plan = false;

        if self.n_pois != pois.len() {
            self.n_pois = pois.len();
            need_to_plan = true;
        }

        let n_docks = if dock.is_some() { 1 } else { 0 };
        if self.n_docks != n_docks {
            self.n_docks = n_docks;
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
            println!("Need to plan.");
            let job = PlanJob {
                pois: pois.iter().map(|p| p.poi.clone()).collect(),
                robot_state: next_state.clone(),
                dock: dock.clone(),
            };
            self.tx_job.send(job).ok();
        }
    }
}

fn unexpected_location(prev_state: &RobotState, next_state: &RobotState) -> bool {
    if let (Some(prev_loc), Some(new_loc)) =
        (prev_state.current_pose.as_ref(), next_state.current_pose.as_ref())
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
    mut set_plan: impl FnMut(PoiSequence),
) {
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
        let result = solver::solve(job);
        planner_running.store(false, Ordering::Relaxed);
        set_plan(result);
    }
}

const INSPECTION_DURATION: f64 = 5.0;
const TOTAL_CHARGE_TIME: f64 = 10.0;

fn integrate_plan(
    sequence: &PoiSequence,
    robot_state: &RobotState,
    dock: &Option<Poi>,
) -> TemporalPlan {
    let mut battery_profile = vec![];
    let mut drive_activities = vec![];
    let mut inspection_activities = vec![];

    if sequence.is_empty() || sequence[0].is_empty() {
        return TemporalPlan {
            generation: 0,
            battery_profile,
            drive_activities,
            inspection_activities,
        };
    }

    let mut current_time = robot_state.t;
    let mut current_poi = Poi {
        name: "Current location".to_string(),
        pose: robot_state
            .current_pose
            .clone()
            .unwrap_or_else(|| sequence[0][0].pose.clone()),
    };

    let mut current_battery = robot_state.battery.0;
    battery_profile.push((current_time, current_battery));

    for vehicle in sequence.iter() {
        for poi in vehicle.iter() {
            // Go to the place
            let travel_time =
                estimated_travel_time(estimated_distance(&current_poi.pose, &poi.pose));
            drive_activities.push((
                PoiTime {
                    poi: current_poi.clone(),
                    time: current_time,
                },
                PoiTime {
                    poi: poi.clone(),
                    time: current_time + travel_time,
                },
            ));
            current_time += travel_time;
            current_battery -= estimate_battery_usage(travel_time);
            current_poi = poi.clone();
            battery_profile.push((current_time, current_battery));

            // Do the inspection
            inspection_activities.push((current_time, current_time + INSPECTION_DURATION));
            current_time += INSPECTION_DURATION;
            battery_profile.push((current_time, current_battery));
        }

        if let Some(dock) = dock.as_ref() {
            // Go to charger
            let travel_time =
                estimated_travel_time(estimated_distance(&current_poi.pose, &dock.pose));
            drive_activities.push((
                PoiTime {
                    poi: current_poi.clone(),
                    time: current_time,
                },
                PoiTime {
                    poi: dock.clone(),
                    time: current_time + travel_time,
                },
            ));
            current_time += travel_time;
            current_battery -= estimate_battery_usage(travel_time);
            current_poi = dock.clone();
            battery_profile.push((current_time, current_battery));

            // wait for charging
            let charge_time = (robot_state.battery.1 - current_battery) * TOTAL_CHARGE_TIME
                / robot_state.battery.1;
            current_time += charge_time;
            current_battery = robot_state.battery.1;
            battery_profile.push((current_time, current_battery));
        }
    }

    TemporalPlan {
        generation: 0,
        battery_profile,
        drive_activities,
        inspection_activities,
    }
}
