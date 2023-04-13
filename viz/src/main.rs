use std::time::Instant;

use eframe::{
    egui::{
        self,
        plot::{Line, PlotBounds, PlotPoints},
        Visuals,
    },
    epaint::Color32,
};
use paho_mqtt::Receiver;
mod model;
use model::*;

enum Event {
    WaypointSuccess(usize),
    WaypointFailure(usize),
}

struct PlanApp {
    _mqtt_client: paho_mqtt::Client,
    mqtt_receiver: Receiver<Option<paho_mqtt::Message>>,
    status: Option<ControllerStatus>,

    previous_plan: Option<PlanStatus>,

    battery_history: Vec<Vec<(Instant, f64)>>,
    events: Vec<Vec<(f64, Event)>>,

    enable_send_waypoint: bool,
    enable_planning: bool,
    num_points_sent: usize,
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
                "planner/enable_planning" => {
                    self.enable_planning = msg.payload_str() == "true";
                }
                "planner/status" => {
                    match serde_json::from_str::<ControllerStatus>(&msg.payload_str()) {
                        Ok(new_status) => {
                            let previous_status = self.status.take();

                            if self.previous_plan.is_none()
                                || self.previous_plan.as_ref().unwrap().plan_version
                                    < new_status.plan.plan_version
                            {
                                self.previous_plan = previous_status.map(|x| x.plan);
                            }

                            for (idx, new_robot_status) in new_status.robots.iter().enumerate() {
                                let time_now = Instant::now();

                                while !(idx < self.battery_history.len()) {
                                    self.battery_history.push(Default::default());
                                }

                                self.battery_history[idx].push((
                                    time_now,
                                    new_robot_status.battery_constraint.remaining_distance,
                                ));
                            }

                            println!("{:?}", self.battery_history);
                            self.status = Some(new_status);
                        }
                        Err(x) => {
                            println!("WARNING: could not parse status message {:?}", x);
                        }
                    }
                }
                _ => {
                    println!("Warning unknown topic");
                }
            }
        }
    }

    pub fn draw_gui(&mut self, ctx: &eframe::egui::Context) {
        egui::CentralPanel::default().show(ctx, |ui| {
            if self.status.is_none() {
                ui.heading("Waiting for first status message from controller.");
                return;
            }
            let status = self.status.as_ref().unwrap();
            let send_waypoint = &mut self.enable_send_waypoint;
            let enable_planning = &mut self.enable_planning;

            draw_sidebar(ui, send_waypoint, enable_planning, status, |p| {
                let msg = paho_mqtt::MessageBuilder::new()
                    .topic("planner/enable_planning")
                    .payload(if p { "true" } else { "false" })
                    .retained(true);
                self._mqtt_client.publish(msg.finalize()).unwrap();
            });

            draw_battery_histories(status, &self.battery_history, ui);
            draw_map(ui, send_waypoint, status, |x, y| {
                let value = serde_json::json!({
                    "name": format!("click{}", self.num_points_sent),
                    "x": x,
                    "y": y,
                    "z": 0.0
                });
                let string = serde_json::to_string(&value).unwrap();
                let msg = paho_mqtt::MessageBuilder::new()
                    .topic("planner/add_waypoint")
                    .payload(string);
                self._mqtt_client.publish(msg.finalize()).unwrap();
                self.num_points_sent += 1;
            });
        });
    }
}

fn draw_map(
    ui: &mut egui::Ui,
    send_waypoint: &mut bool,
    status: &ControllerStatus,
    mut send_pt: impl FnMut(f64, f64),
) {
    // Map
    ui.heading("Map");
    let plot = egui::plot::Plot::new("map_plot").data_aspect(1.0);

    plot.show(ui, |plot_ui| {
        if *send_waypoint && plot_ui.plot_clicked() {
            if let Some(pt) = plot_ui.pointer_coordinate() {
                send_pt(pt.x, pt.y)
            }
        }

        // Current location.
        for robot in status.robots.iter() {
            // Charger location
            if let Some(charger_location) = robot.battery_constraint.charger_location.as_ref() {
                let loc = &charger_location.pose;
                plot_ui.points(
                    egui::plot::Points::new(PlotPoints::from([loc.position.x, loc.position.y]))
                        .shape(egui::plot::MarkerShape::Circle)
                        .name("Robot dock (charging)")
                        .color(eframe::epaint::Color32::GREEN)
                        .radius(8.0),
                );
            }

            if let Some(loc) = robot.current_location.as_ref() {
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
        }

        // Waypoints
        for wp in status.waypoints.iter() {
            let (shape, color) = match (wp.is_charger, &wp.status) {
                (true, _) => (
                    egui::plot::MarkerShape::Circle,
                    eframe::epaint::Color32::DARK_GREEN,
                ),
                (false, WaypointStatus::Pending) => (
                    egui::plot::MarkerShape::Asterisk,
                    eframe::epaint::Color32::BLUE,
                ),
                (false, WaypointStatus::Success) => (
                    egui::plot::MarkerShape::Cross,
                    eframe::epaint::Color32::DARK_GREEN,
                ),
                (false, WaypointStatus::Failure) => (
                    egui::plot::MarkerShape::Cross,
                    eframe::epaint::Color32::DARK_RED,
                ),
            };

            plot_ui.points(
                egui::plot::Points::new(PlotPoints::from([
                    wp.location.pose.position.x,
                    wp.location.pose.position.y,
                ]))
                .name("POI")
                .shape(shape)
                .color(color)
                .radius(10.0),
            );
        }

        // Plan arrows
        for (robot_idx, robot_plan) in status.plan.robot_plans.iter().enumerate() {
            let mut prev_loc = status.robots[robot_idx]
                .current_location
                .as_ref()
                .map(|l| &l.pose);

            for item in robot_plan.iter() {
                draw_arrow(plot_ui, prev_loc, &item.location.pose);
                prev_loc = Some(&item.location.pose);
            }
        }
    });
}

fn draw_battery_histories(
    status: &ControllerStatus,
    histories: &Vec<Vec<(Instant, f64)>>,
    ui: &mut egui::Ui,
) {
    for ((robot_idx, robot), robplan) in status
        .robots
        .iter()
        .enumerate()
        .zip(status.plan.robot_plans.iter())
    {
        ui.heading(&format!("Robot #{}", robot_idx + 1));
        let plot = egui::plot::Plot::new(format!("robhist{}", robot_idx)).height(200.0);

        plot.show(ui, |plot_ui| {
            plot_ui.set_plot_bounds(PlotBounds::from_min_max([-100.0, 0.0], [120.0, 0.0]));
            // Plot future
            let max_batt = robot.battery_constraint.battery_distance;

            if let Some(history) = histories.get(robot_idx) {
                let historic: PlotPoints = history
                    .iter()
                    .map(|(t, f)| [-(t.elapsed().as_secs_f64()), *f / max_batt])
                    .chain(std::iter::once([
                        0.0,
                        robot.battery_constraint.remaining_distance / max_batt,
                    ]))
                    .collect();

                plot_ui.line(Line::new(historic));
            } else {
                println!("NO");
            }

            let batt_curve: PlotPoints =
                std::iter::once([0.0, robot.battery_constraint.remaining_distance / max_batt])
                    .chain(
                        robplan
                            .iter()
                            .map(|p| [p.time, p.remaining_battery / max_batt]),
                    )
                    .collect();
            plot_ui.line(Line::new(batt_curve));
        });
    }
}

fn draw_sidebar(
    ui: &mut egui::Ui,
    send_waypoint: &mut bool,
    enable_planning: &mut bool,
    status: &ControllerStatus,
    mut set_planning_enabled: impl FnMut(bool),
) {
    egui::SidePanel::left("left_panel")
        .min_width(500.0)
        .max_width(500.0)
        .show_inside(ui, |ui| {
            ui.heading("Planner");
            ui.label(format!("Status: {}", status.plan.status_msg));

            if ui.checkbox(enable_planning, "Enable planning.").changed() {
                set_planning_enabled(*enable_planning);
            }

            ui.checkbox(send_waypoint, "Click map to send waypoint.");

            for (idx, robot) in status.robots.iter().enumerate() {
                ui.heading(&format!("Robot #{}", idx + 1));

                let battery_fraction = robot.battery_constraint.remaining_distance
                    / robot.battery_constraint.battery_distance;

                ui.add(
                    egui::ProgressBar::new(battery_fraction as f32).text(format!(
                        "Battery level {:.2}/{:.2}",
                        robot.battery_constraint.remaining_distance,
                        robot.battery_constraint.battery_distance
                    )),
                );
                ui.end_row();

                ui.label(format!(
                    "Battery charger location {:?}",
                    robot.battery_constraint.charger_location
                ));

                if let Some(loc) = &robot.current_location {
                    ui.label(format!("Location: {:?}", loc));
                } else {
                    ui.label(format!("Location unknown"));
                }
            }

            ui.label(format!("Waypoints: {}", status.waypoints.len()));

            ui.label(format!("Solver: {}", &status.plan.solver));
            ui.label(format!("Plan cost: {:.2}", &status.plan.total_cost));
            for (i, robot_steps) in status.plan.robot_plans.iter().enumerate() {
                ui.heading(format!("Plan for robot {}", i + 1));
                for step in robot_steps.iter() {
                    ui.label(format!(
                        "Step {}: {}, batt:{:.2}",
                        i, step.location.name, step.remaining_battery
                    ));
                }
                // ui.label(format!("Solver: {}", &status.solver));
            }
        });
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
            mqtt_client.subscribe("planner/enable_planning", 0).unwrap();
            println!("Connected to MQTT.")
        }
        Err(_msg) => {
            panic!("Failed to connect to MQTT.")
        }
    }

    let app = PlanApp {
        _mqtt_client: mqtt_client,
        mqtt_receiver,
        status: None,
        battery_history: vec![],
        events: vec![],
        previous_plan: None,
        enable_send_waypoint: false,
        enable_planning: false,
        num_points_sent: 0,
    };

    eframe::run_native(
        "isar_inspectionroute interactive",
        eframe::NativeOptions::default(),
        Box::new(|ctx| {
            ctx.egui_ctx.set_visuals(Visuals::light());
            Box::new(app)
        }),
    )
    .unwrap();
}
