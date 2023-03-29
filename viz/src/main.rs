use eframe::{
    egui::{self, plot::PlotPoints, Visuals},
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

    battery_history: Vec<Vec<(f64, f64)>>,
    events: Vec<Vec<(f64, Event)>>,

    enable_send_waypoint: bool,
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
                "planner/status" => {
                    match serde_json::from_str::<ControllerStatus>(&msg.payload_str()) {
                        Ok(x) => {
                            let previous_status = self.status.take();

                            if self.previous_plan.is_none()
                                || self.previous_plan.as_ref().unwrap().plan_version
                                    < x.plan.plan_version
                            {
                                self.previous_plan = previous_status.map(|x| x.plan);
                            }

                            for (idx, robot) in x.robots.iter().enumerate() {
                                let time_now = todo!();
                                self.battery_history[idx].push((
                                    time_now,
                                    robot.battery_constraint.unwrap().battery_distance,
                                ));
                            }

                            self.status = Some(x);
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

            egui::SidePanel::left("left_panel")
                .min_width(500.0)
                .max_width(500.0)
                .show_inside(ui, |ui| {
                    ui.checkbox(
                        &mut self.enable_send_waypoint,
                        "Click map to send waypoint.",
                    );

                    for (idx, robot) in status.robots.iter().enumerate() {
                        ui.heading(&format!("Robot #{}", idx + 1));

                        if let Some(batt) = &robot.battery_constraint {
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

            // Map
            ui.heading("Map");

            for robot in status.robots.iter() {
                let plot = egui::plot::Plot::new("map_plot").data_aspect(1.0);
            }

            let plot = egui::plot::Plot::new("map_plot").data_aspect(1.0);
            // .custom_label_func(|name, value| {
            //     if !name.is_empty() {
            //         format!("{}: {:.*}%", name, 1, value.y)
            //     } else {
            //         "".to_string()
            //     }
            // })

            plot.show(ui, |plot_ui| {
                if self.enable_send_waypoint && plot_ui.plot_clicked() {

                    // TODO
                    // send_waypoint(plot_ui.pointer_coordinate());
                    // self._mqtt_client.publish(msg);
                }

                // Current location.

                for robot in status.robots.iter() {
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
                for (robot_idx, robot_plan) in status.plan.robot_plans.iter().enumerate() {
                    let mut prev_loc = status.robots[robot_idx]
                        .current_location
                        .as_ref()
                        .map(|l| &l.pose);

                    for item in robot_plan.iter() {
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
