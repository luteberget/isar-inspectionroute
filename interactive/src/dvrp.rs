use crate::{
    data::{Location, Poi, RobotState},
    planner::{
        estimate_battery_usage, estimated_distance, estimated_travel_time, PlanJob, PoiTime,
        TemporalPlan,
    },
};

/// Heuristically solve a distance-constrained
/// vehicle routing problem with specified start
/// state that is not the same as the depot state.
pub fn dvrp(planjob: PlanJob) -> TemporalPlan {
    println!("DVRP solving {} pois", planjob.pois.len());
    // Temporarily, planner just does everything in one sequence.
    integrate_plan(vec![planjob.pois], planjob.robot_state, planjob.dock)
}

const INSPECTION_DURATION: f64 = 5.0;
const TOTAL_CHARGE_TIME: f64 = 10.0;

fn integrate_plan(
    sequence: Vec<Vec<Poi>>,
    robot_state: RobotState,
    dock: Option<Poi>,
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
        location: robot_state
            .location
            .unwrap_or_else(|| sequence[0][0].location.clone()),
    };

    let mut current_battery = robot_state.battery.0;
    battery_profile.push((current_time, current_battery));

    for vehicle in sequence.iter() {
        for poi in vehicle.iter() {
            // Go to the place
            let travel_time =
                estimated_travel_time(estimated_distance(&current_poi.location, &poi.location));
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
                estimated_travel_time(estimated_distance(&current_poi.location, &dock.location));
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
            let charge_time = (robot_state.battery.1 - current_battery) * TOTAL_CHARGE_TIME / robot_state.battery.1;
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
