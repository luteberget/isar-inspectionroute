use std::{collections::VecDeque, time::Instant};

use serde_json::json;
use ureq::serde::Deserialize;

use super::*;
use crate::data::*;

pub type IsarMQTTClient = (
    paho_mqtt::Client,
    paho_mqtt::Receiver<Option<paho_mqtt::Message>>,
);

pub struct RobotAddr {
    pub name: String,
    host: String,
    port: usize,
}

#[derive(Default)]
pub struct IsarConnectionBuilder {
    address: String,
    connection: Option<IsarMQTTClient>,
    robots: Option<(usize, Vec<RobotAddr>)>,
    error_msg: Option<String>,
}

fn parse_robot_addr(s: &str) -> Option<RobotAddr> {
    let json: serde_json::Value = serde_json::from_str(s).ok()?;
    let name = json
        .get("robot_id")
        .and_then(serde_json::Value::as_str)?
        .to_string();
    let host = json
        .get("host")
        .and_then(serde_json::Value::as_str)?
        .to_string();
    let port = json.get("port").and_then(serde_json::Value::as_i64)? as usize;
    Some(RobotAddr { name, host, port })
}

impl IsarConnectionBuilder {
    pub fn new() -> Self {
        let mut conn = Self {
            address: "tcp://localhost:1883".to_string(),
            ..Default::default()
        };
        conn.connect();
        conn
    }

    pub fn process_messages(&mut self) {
        if let Some(connection) = self.connection.as_mut() {
            while let Ok(Some(msg)) = connection.1.try_recv() {
                if msg.topic().ends_with("/robot") {
                    if let Some(robot_addr) = parse_robot_addr(&msg.payload_str()) {
                        if let Some(robots) = self.robots.as_mut() {
                            if !robots.1.iter().any(|a| a.name == robot_addr.name) {
                                robots.1.push(robot_addr);
                            }
                        } else {
                            self.robots = Some((0, vec![robot_addr]))
                        }
                    } else {
                        println!(
                            "Warning could not parse robot address message: {}",
                            msg.payload_str()
                        );
                    }
                }
            }
        }
    }

    pub fn edit_address(&mut self) -> &mut String {
        &mut self.address
    }

    pub fn connect(&mut self) {
        self.robots = None;
        self.error_msg = None;
        self.connection = None;
        let mqtt_opts = paho_mqtt::CreateOptionsBuilder::new()
            .server_uri(&self.address)
            .finalize();
        let mqtt_conn_opts = paho_mqtt::ConnectOptionsBuilder::new()
            .keep_alive_interval(std::time::Duration::from_secs(20))
            .clean_session(true)
            .finalize();

        let mqtt_client = paho_mqtt::Client::new(mqtt_opts).unwrap();
        let mqtt_rx = mqtt_client.start_consuming();
        match mqtt_client.connect(mqtt_conn_opts) {
            Ok(_res) => {
                mqtt_client.subscribe("isar/+/robot", 0).unwrap();
                self.connection = Some((mqtt_client, mqtt_rx));
            }
            Err(msg) => {
                self.error_msg = Some(msg.to_string());
            }
        }
    }

    pub fn into_backend(self) -> IsarBackend {
        let connection = self.connection.unwrap();
        let (idx, mut robots) = self.robots.unwrap();
        let robot = robots.remove(idx);
        connection.0.unsubscribe("isar/+/robot").unwrap();
        IsarBackend::new(robot, connection)
    }

    pub fn get_status(&mut self) -> (Option<&mut (usize, Vec<RobotAddr>)>, &str) {
        if let Some(msg) = self.error_msg.as_ref() {
            return (None, msg.as_str());
        }

        let msg = if self.connection.is_none() {
            "Not connected"
        } else if self.connection.is_some() && self.robots.is_none() {
            "Waiting for robot list."
        } else {
            "Connected"
        };

        (self.robots.as_mut(), msg)
    }
}

pub struct IsarBackend {
    description: String,
    robot_addr: RobotAddr,
    events_queue: VecDeque<(Instant, Event)>,
    robot_state: RobotState,
    connection: IsarMQTTClient,
    n_msgs: usize,
    prev_plan_counter: usize,
    running_mission: Option<IsarMission>,
}

struct IsarMission {
    id: String,
    tasks: Vec<IsarTask>,
}

struct IsarTask {
    poi_name: String,
    id: String,
}

impl IsarBackend {
    pub fn new(robot_addr: RobotAddr, connection: IsarMQTTClient) -> Self {
        let description = format!("ISAR:{}", robot_addr.name);
        let trans_vel = 0.35;
        let angular_vel = 0.9;
        let robot_params = RobotParams {
            trans_vel,
            angular_vel,
        };

        connection
            .0
            .subscribe(&format!("isar/{}/state", robot_addr.name), 0)
            .unwrap();
        connection
            .0
            .subscribe(&format!("isar/{}/task", robot_addr.name), 0)
            .unwrap();
        connection
            .0
            .subscribe(&format!("isar/{}/battery", robot_addr.name), 0)
            .unwrap();
        connection
            .0
            .subscribe(&format!("isar/{}/pose", robot_addr.name), 0)
            .unwrap();

        IsarBackend {
            events_queue: Default::default(),
            robot_state: RobotState {
                state: "unknown".to_string(),
                t: 0.0,
                location: None,
                battery: (1.0, 1.0),
                params: robot_params,
            },
            description,
            robot_addr,
            connection,
            n_msgs: 0,
            prev_plan_counter: usize::MAX,
            running_mission: None,
        }
    }

    fn incr_messages(&mut self, n: usize) {
        self.n_msgs += n;
        self.description = format!("ISAR:{} ({} msgs)", self.robot_addr.name, self.n_msgs);
    }

    fn process_mqtt_msgs(&mut self) {
        while let Ok(Some(msg)) = self.connection.1.try_recv() {
            self.incr_messages(1);

            let json: serde_json::Value = match serde_json::from_str(&msg.payload_str()) {
                Ok(j) => j,
                Err(msg) => {
                    println!("Warning: JSON parse error: {:?}", msg);
                    continue;
                }
            };

            if msg.topic().ends_with("/state") {
                let state = match json.get("state").and_then(serde_json::Value::as_str) {
                    Some(l) => l,
                    None => {
                        println!("Warning: could not parse state. ISAR version conflict?");
                        continue;
                    }
                };
                self.robot_state.state = state.to_string();
            } else if msg.topic().ends_with("/task") {
                #[derive(Deserialize)]
                struct TaskStatusJson {
                    robot_id: String,
                    mission_id: Option<String>,
                    task_id: Option<String>,
                    status: Option<String>,
                }

                let task_json: TaskStatusJson = match serde_json::from_str(&msg.payload_str()) {
                    Ok(j) => j,
                    Err(msg) => {
                        println!("Warning: JSON parse error: {:?}", msg);
                        continue;
                    }
                };

                if task_json.robot_id != self.robot_addr.name {
                    println!(
                        "Warning: robot name does not match in task message. Reading it anyway."
                    );
                }

                let status_success = task_json.status.as_deref() == Some("partially_successful")
                    || task_json.status.as_deref() == Some("successful");

                let status_failues = task_json.status.as_deref() == Some("failed")
                    || task_json.status.as_deref() == Some("cancelled");

                if !status_success && !status_failues {
                    println!("Warning: unexpected task status {:?}", task_json.status);
                } else if let Some(mission) = self.running_mission.as_ref() {
                    if task_json.mission_id.as_deref() != Some(&mission.id) {
                        println!(
                            "Warning: mission id does not match. Checking for known tasks anyway."
                        );
                    }
                    if let Some(task_id) = task_json.task_id.as_ref() {
                        while let Some(mission_task) =
                            mission.tasks.iter().find(|t| &t.id == task_id)
                        {
                            self.events_queue.push_back((
                                Instant::now(),
                                Event::Task(mission_task.poi_name.clone(), status_success),
                            ));
                        }
                    } else {
                        println!("Warning: no task id in task status message.");
                    }
                } else {
                    println!("Warning: Received task status, but there is no current mission.");
                }
            } else if msg.topic().ends_with("/battery") {
                let level = match json
                    .get("battery_level")
                    .and_then(serde_json::Value::as_f64)
                {
                    Some(l) => l,
                    None => {
                        println!("Warning: could not parse battery level. ISAR version conflict?");
                        continue;
                    }
                };

                let max_battery = self.robot_state.battery.1.max(level);
                self.robot_state.battery = (level, max_battery);
            } else if msg.topic().ends_with("/pose") {
                let pose = match json.get("pose").and_then(serde_json::Value::as_object) {
                    Some(x) => x,
                    None => {
                        println!("Warning: could not parse position. ISAR version conflict?");
                        continue;
                    }
                };

                let (pos, ori) = match (
                    pose.get("position").and_then(serde_json::Value::as_object),
                    pose.get("orientation")
                        .and_then(serde_json::Value::as_object),
                ) {
                    (Some(pos), Some(ori)) => (pos, ori),
                    _ => {
                        println!("Warning: could not parse position. ISAR version conflict?");
                        continue;
                    }
                };

                let (x, y, z) = (
                    pos.get("x").unwrap().as_f64().unwrap(),
                    pos.get("y").unwrap().as_f64().unwrap(),
                    pos.get("z").unwrap().as_f64().unwrap(),
                );

                let pose = Pose { x, y, z };

                let (x, y, z, w) = (
                    ori.get("x").unwrap().as_f64().unwrap(),
                    ori.get("y").unwrap().as_f64().unwrap(),
                    ori.get("z").unwrap().as_f64().unwrap(),
                    ori.get("w").unwrap().as_f64().unwrap(),
                );

                let orientation = Orientation { x, y, z, w };

                self.robot_state.location = Some(Location { pose, orientation });
            } else {
                println!("Warning: unknown topic {}", msg.topic());
            }
        }
    }
}

impl Backend for IsarBackend {
    fn try_recv_event(&mut self) -> Option<(std::time::Instant, Event)> {
        self.process_mqtt_msgs();
        self.events_queue.pop_front()
    }

    fn backend_description(&self) -> &str {
        &self.description
    }

    fn current_robot_state(&self) -> &RobotState {
        &self.robot_state
    }

    fn set_plan(&mut self, plan_counter: usize, plan: &PoiSequence, dock: &Option<Poi>) {
        if plan_counter == self.prev_plan_counter {
            return;
        }

        let first_poi = plan.iter().flat_map(|s| s.iter()).next().map(|n| &n.name);
        let current_first_poi = self
            .running_mission
            .as_ref()
            .and_then(|m| m.tasks.first())
            .map(|n| &n.poi_name);

        if first_poi == current_first_poi {
            return;
        }

        self.prev_plan_counter = plan_counter;

        // Cancel currently running mission
        if let Some(_mission) = self.running_mission.as_ref() {
            isar_stop_mission(&self.robot_addr);
            self.running_mission = None;
        }

        #[derive(Deserialize)]
        struct IsarMissionJson {
            id: String,
            tasks: Vec<IsarTaskJson>,
        }

        #[derive(Deserialize)]
        struct IsarTaskJson {
            id: String,
        }

        enum PlanStepType {
            Inspection,
            Charge,
        }

        let mut plan_steps = vec![];
        for seq in plan.iter() {
            for poi in seq.iter() {
                plan_steps.push((PlanStepType::Inspection, poi.clone()));
            }
            if let Some(dock) = dock.as_ref() {
                plan_steps.push((PlanStepType::Charge, dock.clone()));
            }
        }

        let plan_json = json!({
            "mission_definition": {
                "tasks": plan_steps.iter().map(|(_,x)| isar_plan_step(x)).collect::<Vec<_>>(),
            }
        });

        let start_mission_addr = format!(
            "{}/schedule/start-mission",
            isar_base_addr(&self.robot_addr)
        );
        let req = ureq::post(&start_mission_addr).send_json(plan_json);
        if let Ok(response) = req {
            if let Ok(mission) = response.into_json::<IsarMissionJson>() {
                if mission.tasks.len() != plan_steps.len() {
                    println!("Warning: the number of tasks in the mission is not as requested.");
                } else {
                    self.running_mission = Some(IsarMission {
                        id: mission.id,
                        tasks: mission
                            .tasks
                            .into_iter()
                            .zip(plan_steps.into_iter())
                            .filter_map(|(task, (steptype, poi))| {
                                matches!(steptype, PlanStepType::Inspection).then(|| IsarTask {
                                    id: task.id,
                                    poi_name: poi.name,
                                })
                            })
                            .collect(),
                    })
                }
            } else {
                println!("Warning: could not parse ISAR mission.");
            }
        } else {
            println!("Warning: start mission failed: {:?}", req);
        }
    }
}

fn isar_stop_mission(addr: &RobotAddr) {
    let stop_mission_addr = format!("{}/schedule/stop-mission", isar_base_addr(addr));
    let req = ureq::post(&stop_mission_addr).call();
    if let Ok(response) = req {
        match response.into_json::<serde_json::Value>() {
            Ok(json) => {
                println!("Stop mission: {:?}", json);
            }
            Err(err) => {
                println!("Stop mission invalid response: {:?}", err);
            }
        }
    }
}

fn isar_base_addr(addr: &RobotAddr) -> String {
    format!("http://{}:{}", addr.host, addr.port)
}

fn isar_plan_step(poi: &Poi) -> serde_json::Value {
    json!({
        "pose": {
            "position": {
                "x": poi.location.pose.x,
                "y": poi.location.pose.y,
                "z": poi.location.pose.z,
                "frame_name": "asset",
            },
            "orientation": {
                "x": poi.location.orientation.x,
                "y": poi.location.orientation.y,
                "z": poi.location.orientation.z,
                "w": poi.location.orientation.w,
                "frame_name": "asset",
            },
            "frame_name": "asset",
        },
        "tag": poi.name,
        "inspection_target": {
            "x": poi.location.pose.x,
            "y": poi.location.pose.y,
            "z": poi.location.pose.z,
            "frame_name": "asset",
        },
        "inspection_types": ["Image"],
    })
}
