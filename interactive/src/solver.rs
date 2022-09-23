use crate::{
    data::{Poi, Pose},
    planner::{estimated_distance, PlanJob, PoiSequence},
};

/// Heuristically solve a the routing problem.
/// It is similar to a distance-constrained
/// vehicle routing problem with specified start
/// state that is not the same as the depot state.
pub fn solve(planjob: PlanJob) -> PoiSequence {
    println!("DVRP solving {} pois", planjob.pois.len());

    if planjob.pois.is_empty() {
        return vec![];
    }

    let current_pose = planjob
        .robot_state
        .current_pose
        .clone()
        .unwrap_or_else(|| planjob.pois[0].pose.clone());

    // if let Some(dock) = planjob.dock {
    if false {
        let dock = planjob.dock.unwrap();
        struct Tour {
            start: Option<Pose>,
            end: Option<Pose>,
            intermediate: Vec<Poi>,
        }

        let mut tours = planjob
            .pois
            .iter()
            .map(|p| Tour {
                start: Some(dock.pose.clone()),
                end: Some(dock.pose.clone()),
                intermediate: vec![p.clone()],
            })
            .collect::<Vec<_>>();

        if !tours.is_empty() {
            tours[0].start = planjob.robot_state.current_pose.clone();
        }
    } else {
        // Not a VRP since we cannot charge.
        // Do the farthest insertion heuristic.

        let mut remaining_pois = planjob.pois.iter().collect::<Vec<_>>();
        let mut tour: Vec<Result<&Poi, Pose>> = vec![Err(current_pose)];
        while let Some((remaining_idx, tour_idx)) = farthest_insertion(&remaining_pois, &tour) {
            let poi = remaining_pois.remove(remaining_idx);
            if tour_idx == 0 {
                tour.insert(1, Ok(poi));
            } else {
                let cost_before =
                    estimated_distance(tour_item_pose(&tour[tour_idx - 1]), &poi.pose)
                        + estimated_distance(&poi.pose, tour_item_pose(&tour[tour_idx]))
                        - estimated_distance(
                            tour_item_pose(&tour[tour_idx - 1]),
                            tour_item_pose(&tour[tour_idx]),
                        );

                let mut cost_after = estimated_distance(tour_item_pose(&tour[tour_idx]), &poi.pose);
                if let Some(t) = tour.get(tour_idx + 1) {
                    cost_after += estimated_distance(&poi.pose, tour_item_pose(t))
                        - estimated_distance(tour_item_pose(&tour[tour_idx]), tour_item_pose(t));
                }

                if cost_before < cost_after {
                    tour.insert(tour_idx, Ok(poi));
                } else {
                    tour.insert(tour_idx + 1, Ok(poi));
                }
            }
        }

        return vec![tour.into_iter().filter_map(|p| p.ok().cloned()).collect()];
    }

    todo!()
}

fn farthest_insertion(
    remaining_pois: &[&Poi],
    tour: &[Result<&Poi, Pose>],
) -> Option<(usize, usize)> {
    remaining_pois
        .iter()
        .enumerate()
        .map(|(idx, add_poi)| {
            (
                idx,
                tour.iter()
                    .enumerate()
                    .map(|(tour_idx, tour_item)| {
                        (
                            tour_idx,
                            ordered_float::OrderedFloat(estimated_distance(
                                &add_poi.pose,
                                tour_item_pose(tour_item),
                            )),
                        )
                    })
                    .min_by_key(|(_, l)| *l)
                    .unwrap(),
            )
        })
        .max_by_key(|(_, (_l, l))| *l)
        .map(|(remaining_pois_idx, (tour_idx, _))| (remaining_pois_idx, tour_idx))
}

fn tour_item_pose<'a>(tour_item: &'a Result<&'a Poi, Pose>) -> &'a Pose {
    match tour_item {
        Ok(poi) => &poi.pose,
        Err(pose) => pose,
    }
}
