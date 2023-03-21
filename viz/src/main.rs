use std::time::Instant;

use eframe::{
    egui::{self, plot::PlotPoints, Visuals},
    epaint::Color32,
};
use paho_mqtt::Receiver;

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
    pub remaining_battery: f64,
}


#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct RobotPlan {
    pub plan: Vec<PlanStep>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct PlanStatus {
    pub solver: String,
    pub total_cost: f64,
    pub plan :Vec<RobotPlan>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct BatteryConstraint {
    pub charger_location: Location,
    pub battery_distance: f64,
    pub remaining_distance: f64,
}

struct RobotState {
    battery_constraint: Option<BatteryConstraint>,
    current_location: Option<Location>,
}
struct PlanApp {
    _mqtt_client: paho_mqtt::Client,
    mqtt_receiver: Receiver<Option<paho_mqtt::Message>>,
    robots :Vec<RobotState>,
    waypoints: Option<Vec<(Location, Instant)>>,
    current_plan: Option<PlanStatus>,
    previous_plan: Option<PlanStatus>,
}

fn draw_arrow(ui: &mut egui::plot::PlotUi, from: Option<&Pose>, to: &Pose) {
    ui.arrows(
        egui::plot::Arrows::new(
            [
                from.map(|l| l.position.x).unwrap_or(to.position.x - 3.0),
                from.map(|l| l.position.y).unwrap_or(to.position.y - 3.0),
            ],
            [to.position.x, to.position.y],
        )
        .color(Color32::BLACK),
    );
}

impl PlanApp {
    pub fn get_messages(&mut self) {
        while let Ok(Some(msg)) = self.mqtt_receiver.try_recv() {
            println!("topic {} payload {}", msg.topic(), msg.payload_str());
            match msg.topic() {
                // "isar/william/pose" => {
                //     let json =
                //         serde_json::from_str::<serde_json::Value>(&msg.payload_str()).unwrap();
                //     let pose_json = json.as_object().unwrap().get("pose").unwrap().clone();

                //     self.current_location = Some(Location {
                //         name: "current_location".to_string(),
                //         pose: serde_json::from_value(pose_json).unwrap(),
                //     });
                // }
                "planner/battery_constraint" => {
                    self.battery_constraint =
                        Some(serde_json::from_str(&msg.payload_str()).unwrap());
                }
                "planner/status" => {
                    self.current_plan = Some(serde_json::from_str(&msg.payload_str()).unwrap());
                }
                "planner/waypoints" => {
                    self.waypoints = Some(serde_json::from_str(&msg.payload_str()).unwrap());
                }
                _ => {
                    println!("Warning unknown topic");
                }
            }
        }
    }

    pub fn draw_gui(&mut self, ctx: &eframe::egui::Context) {
        egui::CentralPanel::default().show(ctx, |ui| {
            egui::SidePanel::left("left_panel")
                .default_width(500.0)
                .show_inside(ui, |ui| {
                    if let Some(batt) = &self.battery_constraint {
                        ui.label(format!("Battery distance {:.2}", batt.battery_distance));
                        ui.label(format!(
                            "Battery full distance {:.2}",
                            batt.remaining_distance
                        ));
                        ui.label(format!(
                            "Battery charger location {:?}",
                            batt.charger_location
                        ));
                    } else {
                        ui.label(format!("Battery unknown"));
                    }

                    if let Some(wps) = &self.waypoints {
                        ui.label(format!("Waypoints: {}", wps.len()));
                    } else {
                        ui.label(format!("Waypoints unknown"));
                    }

                    if let Some(loc) = &self.current_location {
                        ui.label(format!("Location: {:?}", loc));
                    } else {
                        ui.label(format!("Location unknown"));
                    }

                    if let Some(status) = &self.current_plan {
                        ui.label(format!("Solver: {}", &status.solver));
                        ui.label(format!("Plan cost: {:.2}", &status.total_cost));
                        ui.label(format!("Plan steps: {}", &status.plan.len()));

                        for (i, step) in status.plan.iter().enumerate() {
                            ui.label(format!(
                                "Step {}: {}, batt:{:.2}",
                                i, step.location.name, step.remaining_battery
                            ));
                            // ui.label(format!("Solver: {}", &status.solver));
                        }
                    } else {
                        ui.label(format!("Status unknown"));
                    }
                });

            // Map
            ui.heading("Map");

            let plot = egui::plot::Plot::new("map_plot").data_aspect(1.0);
            // .custom_label_func(|name, value| {
            //     if !name.is_empty() {
            //         format!("{}: {:.*}%", name, 1, value.y)
            //     } else {
            //         "".to_string()
            //     }
            // })

            plot.show(ui, |plot_ui| {
                // Current location.
                if let Some(loc) = self.current_location.as_ref() {
                    plot_ui.points(
                        egui::plot::Points::new(PlotPoints::from([
                            loc.pose.position.x,
                            loc.pose.position.y,
                        ]))
                        .shape(egui::plot::MarkerShape::Diamond)
                        .name("Current robot location")
                        .color(eframe::epaint::Color32::DARK_RED)
                        .radius(8.0),
                    );
                }

                // Waypoints
                if let Some(wps) = self.waypoints.as_ref() {
                    plot_ui.points(
                        egui::plot::Points::new(PlotPoints::from_iter(
                            wps.iter().map(|p| [p.pose.position.x, p.pose.position.y]),
                        ))
                        .name("POI")
                        .shape(egui::plot::MarkerShape::Plus)
                        .color(eframe::epaint::Color32::DARK_GREEN)
                        .radius(10.0),
                    );
                }

                // Charger
                if let Some(batt) = self.battery_constraint.as_ref() {
                    plot_ui.points(
                        egui::plot::Points::new(PlotPoints::from([
                            batt.charger_location.pose.position.x,
                            batt.charger_location.pose.position.y,
                        ]))
                        .shape(egui::plot::MarkerShape::Circle)
                        .name("Robot dock (charging)")
                        .color(eframe::epaint::Color32::LIGHT_RED)
                        .radius(8.0),
                    );
                }

                // // Dock
                // if let Some(dock) = app.dock.as_ref() {
                //     let loc = &dock.pose;
                //     plot_ui.points(
                //         egui::plot::Points::new(PlotPoints::from([loc.position.x, loc.position.y]))
                //             .shape(egui::plot::MarkerShape::Circle)
                //             .name("Robot dock (charging)")
                //             .color(eframe::epaint::Color32::GREEN)
                //             .radius(8.0),
                //     );
                // }

                // Plan arrows
                if let Some(status) = self.current_plan.as_ref() {
                    let mut prev_loc = None; //self.current_location.as_ref().map(|x| &x.pose);

                    for item in status.plan.iter() {
                        draw_arrow(plot_ui, prev_loc, &item.location.pose);
                        prev_loc = Some(&item.location.pose);

                        // // Arrow to dock
                        // if let Some(dock) = app.dock.as_ref() {
                        //     draw_arrow(plot_ui, prev_loc, &dock.pose);
                        // }
                        // prev_loc = app.dock.as_ref().map(|d| &d.pose);
                    }
                }
            });
        });
    }
}

impl eframe::App for PlanApp {
    fn update(&mut self, ctx: &eframe::egui::Context, _frame: &mut eframe::Frame) {
        self.get_messages();
        self.draw_gui(ctx);

        ctx.request_repaint_after(std::time::Duration::from_secs_f32(0.2));
    }
}

fn main() {
    let mqtt_opts = paho_mqtt::CreateOptionsBuilder::new()
        .server_uri("tcp://localhost:1883")
        .finalize();

    let mqtt_conn_opts = paho_mqtt::ConnectOptionsBuilder::new()
        .keep_alive_interval(std::time::Duration::from_secs(20))
        .clean_session(true)
        .finalize();

    let mqtt_client = paho_mqtt::Client::new(mqtt_opts).unwrap();
    let mqtt_receiver = mqtt_client.start_consuming();
    match mqtt_client.connect(mqtt_conn_opts) {
        Ok(_res) => {
            mqtt_client.subscribe("planner/status", 0).unwrap();
            mqtt_client.subscribe("planner/waypoints", 0).unwrap();

            mqtt_client
                .subscribe("planner/battery_constraint", 0)
                .unwrap();
            mqtt_client.subscribe("isar/william/pose", 0).unwrap();
            println!("Connected to MQTT.")
        }
        Err(_msg) => {
            panic!("Failed to connect to MQTT.")
        }
    }

    let app = PlanApp {
        _mqtt_client: mqtt_client,
        mqtt_receiver,
        battery_constraint: None,
        waypoints: None,
        current_plan: None,
        current_location: None,
    };

    eframe::run_native(
        "isar_inspectionroute interactive",
        eframe::NativeOptions::default(),
        Box::new(|ctx| {
            ctx.egui_ctx.set_visuals(Visuals::light());
            Box::new(app)
        }),
    );
}
