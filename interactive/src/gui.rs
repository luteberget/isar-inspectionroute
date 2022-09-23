use eframe::{
    egui::{
        self,
        plot::{Arrows, HLine, LinkedAxisGroup, PlotPoint, PlotPoints, PlotUi, Text, VLine},
        TextEdit, Widget,
    },
    epaint::Color32,
};

use crate::{
    app::ParsePoiWindow,
    backend::{isar::IsarConnectionBuilder, Backend},
    data::Location,
};

const BATTERY_WARNING_LEVEL: f64 = 0.1;

pub fn draw_gui(app: &mut crate::app::WaypointsApp, ctx: &eframe::egui::Context) {
    egui::CentralPanel::default().show(ctx, |ui| {
        egui::SidePanel::left("left_panel")
            .default_width(500.0)
            .show_inside(ui, |ui| {
                backend_gui(ui, app);
                ui.separator();
                planner_gui(ui, app);
                ui.separator();
                poi_list_gui(ui, app);
            });

        timelines_gui(ui, app);
        map_gui(ui, app);
    });

    {
        let mut open = app.parse_poi_window.is_some();
        egui::Window::new("Add POI")
            .open(&mut open)
            .resizable(true)
            .default_width(800.0)
            .show(ctx, |ui| {
                let value = add_poi_window(ui, app.parse_poi_window.as_mut().unwrap());
                if let Some(pois) = value {
                    for poi in pois {
                        if poi.name.starts_with("dock") {
                            app.dock = Some(poi);
                        } else {
                            app.pending_pois.push(poi);
                        }
                    }
                    app.parse_poi_window = None;
                }
            });
        if !open {
            app.parse_poi_window = None;
        }
    }

    {
        let mut open = app.connect_to_backend_window.is_some();
        egui::Window::new("Connect to ISAR")
            .open(&mut open)
            .resizable(true)
            .default_width(800.0)
            .show(ctx, |ui| {
                let value =
                    connect_backend_window(app.connect_to_backend_window.as_mut().unwrap(), ui);
                if let Some(backend) = value {
                    app.backend = Some(backend);
                    app.connect_to_backend_window = None;
                }
            });
        if !open {
            app.connect_to_backend_window = None;
        }
    }
}

fn add_poi_window(
    ui: &mut eframe::egui::Ui,
    state: &mut ParsePoiWindow,
) -> Option<Vec<crate::data::Poi>> {
    let mut value = None;
    ui.heading("Input POI in ISAR format.");

    egui::SidePanel::right("rp")
        .default_width(500.0)
        .show_inside(ui, |ui| {
            ui.wrap_text();

            match &state.parse_result {
                Ok(poi) => {
                    if ui.button("Add POI").clicked() {
                        value = Some(poi.clone());
                    }
                    ui.label(format!("Parsed: {:?}", poi));
                }
                Err(msg) => {
                    ui.add_enabled(false, eframe::egui::Button::new("Add POI"));
                    ui.label(format!("Parse error: {:?}", msg));
                }
            }
        });

    let text = TextEdit::multiline(&mut state.input)
        .desired_rows(25)
        .desired_width(f32::INFINITY);
    if text.ui(ui).changed() {
        *state = ParsePoiWindow::from_string(std::mem::take(&mut state.input));
    }

    value
}

fn timelines_gui(ui: &mut eframe::egui::Ui, app: &mut crate::app::WaypointsApp) {
    ui.heading("Timelines");

    let plan_idx = app.planner.plan_counter;
    ui.label("Locations");
    let y0 = 0.0;
    let y1 = 1.0;

    let link_axis = LinkedAxisGroup::x();

    let plan = app.backend.as_ref().and_then(|b| {
        app.planner
            .get_temporal_plan(b.current_robot_state(), &app.dock)
    });

    egui::plot::Plot::new(format!("loc_tl{}", plan_idx))
        .height(50.0)
        .link_axis(link_axis.clone())
        .allow_boxed_zoom(false)
        .allow_scroll(false)
        .show_axes([true, false])
        .show(ui, |plot_ui| {
            if let Some(plan) = plan.as_ref() {
                for drive_box in plan.drive_activities.iter() {
                    plot_ui.polygon(
                        egui::plot::Polygon::new(PlotPoints::from_iter([
                            [drive_box.0.time, y0],
                            [drive_box.1.time, y0],
                            [drive_box.1.time, y1],
                            [drive_box.0.time, y1],
                        ]))
                        .color(Color32::BLUE),
                    );

                    let label_text = format!("-> {}", drive_box.1.poi.name);
                    let center = PlotPoint::new(
                        0.5 * (drive_box.0.time + drive_box.1.time),
                        0.5 * (y0 + y1),
                    );
                    plot_ui.text(Text::new(center, label_text))
                }
            }

            now_line(app, plot_ui);
        });

    ui.label("Inspections");
    egui::plot::Plot::new(format!("insp_tl{}", plan_idx))
        .height(50.0)
        .allow_boxed_zoom(false)
        .allow_scroll(false)
        .show_axes([true, false])
        .link_axis(link_axis.clone())
        .show(ui, |plot_ui| {
            if let Some(plan) = plan.as_ref() {
                for inspection in plan.inspection_activities.iter() {
                    plot_ui.polygon(
                        egui::plot::Polygon::new(PlotPoints::from_iter([
                            [inspection.0, y0],
                            [inspection.1, y0],
                            [inspection.1, y1],
                            [inspection.0, y1],
                        ]))
                        .color(Color32::RED),
                    );
                    let label_text = "Inspect";
                    let center =
                        PlotPoint::new(0.5 * (inspection.0 + inspection.1), 0.5 * (y0 + y1));
                    plot_ui.text(Text::new(center, label_text))
                }
            }

            now_line(app, plot_ui);
        });
    ui.label("Battery level");
    egui::plot::Plot::new(format!("batt_tl{}", plan_idx))
        .height(50.0)
        .allow_boxed_zoom(false)
        .allow_scroll(false)
        .show_axes([true, false])
        .link_axis(link_axis)
        .show(ui, |plot_ui| {
            if let Some(plan) = plan.as_ref() {
                plot_ui.line(egui::plot::Line::new(PlotPoints::from_iter(
                    plan.battery_profile.iter().map(|(x, y)| [*x, *y]),
                )));
            }
            now_line(app, plot_ui);
            plot_ui.hline(
                HLine::new(BATTERY_WARNING_LEVEL)
                    .highlight(true)
                    .color(Color32::GOLD),
            );
        });
}

fn now_line(app: &mut crate::app::WaypointsApp, plot_ui: &mut egui::plot::PlotUi) {
    if let Some(backend) = app.backend.as_ref() {
        plot_ui.vline(
            VLine::new(backend.current_robot_state().t)
                .highlight(true)
                .color(Color32::BLUE),
        );
    }
}

fn map_gui(ui: &mut eframe::egui::Ui, app: &mut crate::app::WaypointsApp) {
    ui.heading("Map");

    let plot = egui::plot::Plot::new("maze_plot").data_aspect(1.0);
    // .custom_label_func(|name, value| {
    //     if !name.is_empty() {
    //         format!("{}: {:.*}%", name, 1, value.y)
    //     } else {
    //         "".to_string()
    //     }
    // })

    plot.show(ui, |plot_ui| {
        // Current location.
        if let Some(backend) = app.backend.as_ref() {
            if let Some(loc) = backend.current_robot_state().location.as_ref() {
                plot_ui.points(
                    egui::plot::Points::new(PlotPoints::from([loc.pose.x, loc.pose.y]))
                        .shape(egui::plot::MarkerShape::Diamond)
                        .name("Current robot location")
                        .color(eframe::epaint::Color32::DARK_RED)
                        .radius(8.0),
                );
            }
        }

        // POIs
        plot_ui.points(
            egui::plot::Points::new(PlotPoints::from_iter(
                app.pending_pois
                    .iter()
                    .map(|p| [p.location.pose.x, p.location.pose.y]),
            ))
            .name("POI")
            .shape(egui::plot::MarkerShape::Plus)
            .color(eframe::epaint::Color32::DARK_GREEN)
            .radius(10.0),
        );

        // Dock
        if let Some(dock) = app.dock.as_ref() {
            let loc = &dock.location;
            plot_ui.points(
                egui::plot::Points::new(PlotPoints::from([loc.pose.x, loc.pose.y]))
                    .shape(egui::plot::MarkerShape::Circle)
                    .name("Robot dock (charging)")
                    .color(eframe::epaint::Color32::GREEN)
                    .radius(8.0),
            );
        }

        // Plan arrows
        if let (_, Some(poi_seqs)) = app.planner.get_plan_sequence() {
            let mut prev_loc = app
                .backend
                .as_ref()
                .and_then(|b| b.current_robot_state().location.as_ref());
            for seq in poi_seqs.iter() {
                for poi in seq.iter() {
                    draw_arrow(plot_ui, prev_loc, &poi.location);
                    prev_loc = Some(&poi.location);
                }

                // Arrow to dock
                if let Some(dock) = app.dock.as_ref() {
                    draw_arrow(plot_ui, prev_loc, &dock.location);
                }
                prev_loc = app.dock.as_ref().map(|d| &d.location);
            }
        }
    });
}

fn draw_arrow(ui: &mut PlotUi, from: Option<&Location>, to: &Location) {
    ui.arrows(
        Arrows::new(
            [
                from.map(|l| l.pose.x).unwrap_or(to.pose.x - 3.0),
                from.map(|l| l.pose.y).unwrap_or(to.pose.y - 3.0),
            ],
            [to.pose.x, to.pose.y],
        )
        .color(Color32::BLACK),
    );
}

fn planner_gui(ui: &mut eframe::egui::Ui, app: &mut crate::app::WaypointsApp) {
    ui.heading("Planner");
    ui.label(&format!(
        "Plan available: {} (plan #{})",
        if app.planner.get_plan_sequence().1.is_some() {
            "yes"
        } else {
            "no"
        },
        app.planner.get_plan_sequence().0
    ));
    ui.label(&format!(
        "Planner running: {}",
        if app.planner.is_planner_running() {
            "yes"
        } else {
            "no"
        }
    ));
}

fn poi_list_gui(ui: &mut eframe::egui::Ui, app: &mut crate::app::WaypointsApp) {
    ui.heading("POIs/inspections");
    for poi in app.pending_pois.iter_mut() {
        ui.label(format!(
            "POI at x={:.2} y={:.2}",
            poi.location.pose.x, poi.location.pose.y
        ));
    }

    if let Some(dock) = app.dock.as_ref() {
        ui.label(format!(
            "Dock at x={:.2} y={:.2}",
            dock.location.pose.x, dock.location.pose.y
        ));
    } else {
        ui.label("No dock is known!");
    }

    if ui
        .add_enabled(
            app.parse_poi_window.is_none(),
            eframe::egui::Button::new("☀ Add POI..."),
        )
        .clicked()
    {
        app.parse_poi_window = Some(ParsePoiWindow::from_string("".to_string()));
    }
}

fn backend_gui(ui: &mut eframe::egui::Ui, app: &mut crate::app::WaypointsApp) {
    match app.backend.as_mut() {
        Some(backend) => {
            ui.label(format!("Connected to {}", backend.backend_description()));
            if ui.button("⊗ Disconnect").clicked() {
                app.backend = None;
                return;
            }
            ui.label(format!("{:#?}", backend.current_robot_state()));

            // let robot_location = robot.location.as_ref().map(|l| format!("x={:.2},y={:.2}", l.pose.x, l.pose.y)).unwrap_or_else(|| "Unknown".to_string());
            // ui.label(format!("Robot loc.: {}", robot_location));

            // let max_battery = 1.0;
            // let progress = robot.battery / max_battery;
            // let progress_bar =
            //     egui::ProgressBar::new(progress).text(format!("Battery level {:.0}", progress * 100.0));
            // ui.add(progress_bar);
            // ui.end_row();
        }
        None => {
            if ui.button("🖧 Connect to ISAR").clicked() {
                app.connect_to_backend_window = Some(IsarConnectionBuilder::new());
            }
        }
    }
}

fn connect_backend_window(
    state: &mut IsarConnectionBuilder,
    ui: &mut eframe::egui::Ui,
) -> Option<Box<dyn Backend>> {
    ui.text_edit_singleline(state.edit_address());
    if ui.button("(->) Connect").clicked() {
        state.connect();
    }

    let (robots, status_msg) = state.get_status();
    ui.label(status_msg);
    if let Some((idx, robots)) = robots {
        egui::ComboBox::from_label("Select one!")
            .show_index(ui, idx, robots.len(), |i| robots[i].name.clone());

        if ui.button("Connect!").clicked() {
            return Some(Box::new(std::mem::take(state).into_backend()));
        }
    }

    None
}
