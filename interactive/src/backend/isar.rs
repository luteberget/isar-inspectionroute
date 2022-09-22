use std::{
    collections::VecDeque,
    sync::{Arc, Mutex},
    time::Instant,
};

use super::*;
use crate::data::*;

pub type IsarMQTTClient = (
    paho_mqtt::Client,
    paho_mqtt::Receiver<Option<paho_mqtt::Message>>,
);

#[derive(Default)]
pub struct IsarConnectionBuilder {
    address: String,
    connection: Option<IsarMQTTClient>,
    robots: Option<(usize, Vec<String>)>,
    error_msg: Option<String>,
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
                    let json: Result<serde_json::Value, _> =
                        serde_json::from_str(&msg.payload_str());
                    if let Ok(json) = json {
                        if let Some(robot_name) =
                            json.get("robot_id").and_then(serde_json::Value::as_str)
                        {
                            if let Some(robots) = self.robots.as_mut() {
                                if !robots.1.iter().any(|x| x == robot_name) {
                                    robots.1.push(robot_name.to_string());
                                }
                            } else {
                                self.robots = Some((0, vec![robot_name.to_string()]))
                            }
                        }
                    } else {
                        println!("Error parsing MQTT message {:?}", msg);
                    }
                } else {
                    println!("Wrong message.");
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

    pub fn get_status(&mut self) -> (Option<&mut (usize, Vec<String>)>, &str) {
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
    robot_name: String,
    events_queue: Arc<Mutex<VecDeque<(Instant, Event)>>>,
    robot_state: RobotState,
    connection: IsarMQTTClient,
    n_msgs: usize,
}

impl IsarBackend {
    pub fn new(robot_name: String, connection: IsarMQTTClient) -> Self {
        let description = format!("ISAR:{}", robot_name);
        let trans_vel = 0.35;
        let angular_vel = 0.9;
        let robot_params = RobotParams {
            trans_vel,
            angular_vel,
        };

        connection
            .0
            .subscribe(&format!("isar/{}/state", robot_name), 0)
            .unwrap();
        connection
            .0
            .subscribe(&format!("isar/{}/task", robot_name), 0)
            .unwrap();
        connection
            .0
            .subscribe(&format!("isar/{}/battery", robot_name), 0)
            .unwrap();
        connection
            .0
            .subscribe(&format!("isar/{}/pose", robot_name), 0)
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
            robot_name,
            connection,
            n_msgs: 0,
        }
    }

    fn incr_messages(&mut self, n: usize) {
        self.n_msgs += n;
        self.description = format!("ISAR:{} ({} msgs)", self.robot_name, self.n_msgs);
    }

    fn process_mqtt_msgs(&mut self) {
        while let Ok(Some(msg)) = self.connection.1.try_recv() {
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
                println!("task  {:?}", json);
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
    fn set_mission(&mut self, task: &[Task]) -> Vec<TaskId> {
        todo!()
    }

    fn try_recv_event(&mut self) -> Option<(std::time::Instant, Event)> {
        self.process_mqtt_msgs();

        None
    }

    fn backend_name(&self) -> &str {
        &self.description
    }

    fn current_robot_state(&self) -> &RobotState {
        &self.robot_state
    }
}
