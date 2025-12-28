use crate::bvh::Bvh;
use crate::util::{MinHeapEntry, raycast};
use macroquad::prelude::*;

use std::collections::BinaryHeap;
use std::collections::HashMap;

pub fn naive_pathfind(start: Vec2, goal: Vec2, rects: &[Rect]) -> Option<Vec<Vec2>> {
    let mut points = vec![start, goal];
    let start_idx = 0;
    let goal_idx = 1;
    for rect in rects {
        // Add rect's corners to points with a bit of padding
        let eps = 0.01;
        points.push(rect.point() + Vec2::new(-eps, -eps));
        points.push(rect.point() + Vec2::new(rect.w + eps, -eps));
        points.push(rect.point() + Vec2::new(-eps, rect.h + eps));
        points.push(rect.point() + Vec2::new(rect.w + eps, rect.h + eps));
    }

    // Construct visibility graph
    let mut g = Vec::new();
    for i in 0..points.len() {
        g.push(Vec::new());
        for j in 0..i {
            if raycast(points[i], points[j], rects).is_none() {
                g[i].push(j);
                g[j].push(i);
            }
        }
    }

    // Perform A* search
    let mut queue = BinaryHeap::new();
    let mut dist = HashMap::new();
    let mut came_from = HashMap::new();
    queue.push(MinHeapEntry(start_idx, 0.0));
    dist.insert(start_idx, 0.0);

    while let Some(MinHeapEntry(current, _)) = queue.pop() {
        if current == goal_idx {
            // Goal found, reconstruct path
            let mut path = Vec::new();
            let mut current = goal_idx;
            while let Some(prev) = came_from.get(&current) {
                path.push(points[current]);
                current = *prev;
            }
            path.push(points[start_idx]);
            path.reverse();
            return Some(path);
        }
        // Add neighbors to the queue
        for &next in &g[current] {
            let new_dist = dist[&current] + (points[next] - points[current]).length();
            if !dist.contains_key(&next) || new_dist < *dist.get(&next).unwrap() {
                dist.insert(next, new_dist);
                came_from.insert(next, current);
                queue.push(MinHeapEntry(
                    next,
                    new_dist + (goal - points[next]).length(),
                ));
            }
        }
    }

    None
}

pub fn greedy_pathfind(start: Vec2, goal: Vec2, rects: &[Rect]) -> Option<Vec<Vec2>> {
    let mut points = vec![start, goal];
    let mut rect_corners = Vec::new();
    let start_idx = 0;
    let goal_idx = 1;
    for rect in rects {
        // Check if goal is impossible to reach
        if rect.contains(goal) {
            return None;
        }

        // Add rect's corners to points with a bit of padding
        let eps = 0.01;
        let idx = points.len();
        points.push(rect.point() + Vec2::new(-eps, -eps));
        points.push(rect.point() + Vec2::new(rect.w + eps, -eps));
        points.push(rect.point() + Vec2::new(-eps, rect.h + eps));
        points.push(rect.point() + Vec2::new(rect.w + eps, rect.h + eps));
        rect_corners.push([idx, idx + 1, idx + 2, idx + 3]);
    }

    let mut queue = BinaryHeap::new();
    let mut dist = HashMap::new();
    let mut came_from = HashMap::new();

    // Initialize start node
    dist.insert(start_idx, 0.0);

    // Raycast from start to goal
    let hit = raycast(start, goal, &rects);
    if let Some(hit) = hit {
        for corner in rect_corners[hit.rect_idx] {
            queue.push(MinHeapEntry(
                (corner, start_idx),
                (points[start_idx] - points[corner]).length() // dist
                    + (points[corner] - points[goal_idx]).length(), // A* heuristic
            ));
        }
    } else {
        // Goal directly reachable
        let path = vec![start, goal];
        return Some(path);
    }

    let mut nodes_expanded = 0;

    while let Some(MinHeapEntry((current, parent), _)) = queue.pop() {
        nodes_expanded += 1;

        let cur_dist = dist[&parent] + (points[parent] - points[current]).length();

        if dist.contains_key(&current) && dist[&current] <= cur_dist {
            continue;
        }

        if let Some(hit) = raycast(points[parent], points[current], &rects) {
            // Current node not visible from parent
            if !rect_corners[hit.rect_idx].contains(&current) {
                for corner in rect_corners[hit.rect_idx] {
                    // if corner == current {
                    //     continue;
                    // }
                    queue.push(MinHeapEntry(
                        (corner, parent),
                        dist[&parent]
                            + (points[parent] - points[corner]).length()
                            + (points[corner] - points[goal_idx]).length(),
                    ));
                }
            }
            continue;
        };

        dist.insert(current, cur_dist);
        came_from.insert(current, parent);

        // Check if path can be improved by walking directly from parent's parent to the current node
        if let Some(grandparent) = came_from.get(&parent) {
            if raycast(points[*grandparent], points[current], &rects).is_none() {
                // Direct path from grandparent to current node is visible, prune path
                dist.insert(
                    current,
                    dist[grandparent] + (points[current] - points[*grandparent]).length(),
                );
                came_from.insert(current, *grandparent);
            }
        }

        if current == goal_idx {
            // Goal reached
            let mut path = vec![goal];
            let mut current = goal_idx;
            while let Some(parent) = came_from.get(&current) {
                path.push(points[*parent]);
                current = *parent;
            }
            path.reverse();
            println!("Nodes expanded: {}", nodes_expanded);
            return Some(path);
        }

        // Raycast from current to goal
        if let Some(hit) = raycast(points[current], points[goal_idx], &rects) {
            // Current node not visible from goal
            for corner in rect_corners[hit.rect_idx] {
                if corner == current {
                    continue;
                }
                queue.push(MinHeapEntry(
                    (corner, current),
                    cur_dist
                        + (points[current] - points[corner]).length()
                        + (points[corner] - points[goal_idx]).length(),
                ));
            }
            continue;
        } else {
            // Goal is visible from current node
            queue.push(MinHeapEntry(
                (goal_idx, current),
                cur_dist + (points[current] - points[goal_idx]).length(),
            ));
        }
    }

    println!("Nodes expanded: {}", nodes_expanded);
    return None;
}

/// BVH accelerated pathfinding algorithm
/// Same as `greedy_pathfind` but with BVH for faster raycasts
/// Overall idea:
/// Perform A*, but instead of adding all visible neighbors, we lazily add them when needed
/// This is done in a couple of ways:
/// 1) Raycasting from current node to the goal, adding all 4 vertices of a single blocking obstacle to the open set (if there is one)
/// 2) Raycasting from the current node to its parent, adding all 4 vertices of a single blocking obstacle to the open set (if there is one)
/// Note that the raycast doesn't necessarily need to give the closest hit, the algorithm will still work correctly.
/// It's generally a lot faster if the raycast gives hits that aren't super far away though.
pub fn bvh_pathfind(start: Vec2, goal: Vec2, rects: &[Rect], bvh: &Bvh) -> Option<Vec<Vec2>> {
    // Start by assigning an index to all points
    // This could be moved outside the function and re-used between pathfinds (it's O(N)) but i'm lazy
    let mut points = vec![start, goal];
    let mut rect_corners = Vec::new();
    let start_idx = 0;
    let goal_idx = 1;
    for rect in rects {
        // Check if goal is impossible to reach
        if rect.contains(goal) {
            return None;
        }

        // Add rect's corners to points with a bit of padding
        let eps = 0.01;
        let idx = points.len();
        points.push(rect.point() + Vec2::new(-eps, -eps));
        points.push(rect.point() + Vec2::new(rect.w + eps, -eps));
        points.push(rect.point() + Vec2::new(-eps, rect.h + eps));
        points.push(rect.point() + Vec2::new(rect.w + eps, rect.h + eps));
        rect_corners.push([idx, idx + 1, idx + 2, idx + 3]);
    }

    // Initialize priority queue and distance map for A*
    let mut queue = BinaryHeap::new(); // open set
    let mut dist = HashMap::new(); // path length
    let mut came_from = HashMap::new(); // node parent map

    // Unlike regular A* we keep track of the parent node in the priority queue,
    // then write to the distance map and parent map later
    // This allows us to add invalid (node, parent) pairs to the queue and check them later lazily, but at the cost of extra queue operations

    // Initialize start node
    dist.insert(start_idx, 0.0);

    // Raycast from start to goal and add corners of a blocking obstacle
    if let Some(rect_idx) = bvh.raycast(start, goal) {
        for corner in rect_corners[rect_idx] {
            let next_dist = (points[start_idx] - points[corner]).length(); // Distance to parent
            let heuristic = (points[corner] - points[goal_idx]).length(); // A* heuristic
            queue.push(MinHeapEntry((corner, start_idx), next_dist + heuristic));
        }
    } else {
        // Goal directly reachable
        let path = vec![start, goal];
        return Some(path);
    }

    let mut nodes_expanded = 0;

    while let Some(MinHeapEntry((current, parent), _)) = queue.pop() {
        nodes_expanded += 1;

        let cur_dist = dist[&parent] + (points[parent] - points[current]).length();

        if dist.contains_key(&current) && dist[&current] <= cur_dist {
            // There's already a shorter path to this node
            continue;
        }

        // VALIDATE: Check if parent -> current is actually traversable
        if let Some(rect_idx) = bvh.raycast(points[parent], points[current]) {
            // Current node not visible from parent, something is blocking
            // We need to add the vertices of a blocking rectangle to the queue, otherwise we might not find a path
            // Note that if the raycast node is the current node, we don't need to add it to the queue
            // If we don't skip that case, we can end up adding the same nodes to the queue forever
            if !rect_corners[rect_idx].contains(&current) {
                for corner in rect_corners[rect_idx] {
                    let next_dist = dist[&parent] + (points[parent] - points[corner]).length();
                    let heuristic = (points[corner] - points[goal_idx]).length();
                    queue.push(MinHeapEntry((corner, parent), next_dist + heuristic));
                }
            }
            // Don't mark `current` as visited - it's not actually reachable from the parent
            continue;
        };

        // We now know this (node, parent) pair is valid, so we record its distance and parent
        dist.insert(current, cur_dist);
        came_from.insert(current, parent);

        // It might be possible to smoothen the path by removing unnecessary nodes from it.
        // Check if path can be improved by walking directly from parent's parent to the current node
        if let Some(grandparent) = came_from.get(&parent) {
            if bvh.raycast(points[*grandparent], points[current]).is_none() {
                // Direct path from grandparent to current node is visible, shorten path
                dist.insert(
                    current,
                    dist[grandparent] + (points[current] - points[*grandparent]).length(),
                );
                came_from.insert(current, *grandparent);
            }
        }

        if current == goal_idx {
            // Goal reached, reconstruct path
            let mut path = vec![goal];
            let mut current = goal_idx;
            while let Some(parent) = came_from.get(&current) {
                path.push(points[*parent]);
                current = *parent;
            }
            path.reverse();
            println!("Nodes expanded: {}", nodes_expanded);
            return Some(path);
        }

        // EXPAND: Find next obstacles toward goal
        // This is done greedily by raycasting directly to the goal and adding only the corners of a single blocking obstacle to the queue.
        // Note that these vertices may be blocked by an arbitrary number of other obstacles, but we add them to the queue anyway and check them later (lazily).
        // Raycast from current to goal
        if let Some(rect_idx) = bvh.raycast(points[current], points[goal_idx]) {
            // Current node not visible from goal
            for corner in rect_corners[rect_idx] {
                if corner == current {
                    continue;
                }
                let next_dist = cur_dist + (points[current] - points[corner]).length();
                let heuristic = (points[corner] - points[goal_idx]).length();
                queue.push(MinHeapEntry((corner, current), next_dist + heuristic));
            }
        } else {
            // Goal is visible from current node
            queue.push(MinHeapEntry(
                (goal_idx, current),
                cur_dist + (points[current] - points[goal_idx]).length(),
            ));
        }
    }

    println!("Nodes expanded: {}", nodes_expanded);
    return None;
}
