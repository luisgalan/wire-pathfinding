use macroquad::prelude::*;
use std::collections::BinaryHeap;
use std::collections::HashMap;

fn intersect_line(rect: &Rect, start: Vec2, end: Vec2) -> Option<Vec2> {
    // Get rectangle bounds
    let rect_left = rect.x;
    let rect_right = rect.x + rect.w;
    let rect_top = rect.y;
    let rect_bottom = rect.y + rect.h;

    // Line direction
    let dir = end - start;
    let length = dir.length();
    if length == 0.0 {
        return None;
    }

    let mut closest_t = f32::MAX;
    let mut found = false;

    // Check intersection with each edge of the rectangle
    // Left edge
    if dir.x != 0.0 {
        let t = (rect_left - start.x) / dir.x;
        if t >= 0.0 && t <= 1.0 {
            let y = start.y + t * dir.y;
            if y >= rect_top && y <= rect_bottom {
                if t < closest_t {
                    closest_t = t;
                    found = true;
                }
            }
        }
    }

    // Right edge
    if dir.x != 0.0 {
        let t = (rect_right - start.x) / dir.x;
        if t >= 0.0 && t <= 1.0 {
            let y = start.y + t * dir.y;
            if y >= rect_top && y <= rect_bottom {
                if t < closest_t {
                    closest_t = t;
                    found = true;
                }
            }
        }
    }

    // Top edge
    if dir.y != 0.0 {
        let t = (rect_top - start.y) / dir.y;
        if t >= 0.0 && t <= 1.0 {
            let x = start.x + t * dir.x;
            if x >= rect_left && x <= rect_right {
                if t < closest_t {
                    closest_t = t;
                    found = true;
                }
            }
        }
    }

    // Bottom edge
    if dir.y != 0.0 {
        let t = (rect_bottom - start.y) / dir.y;
        if t >= 0.0 && t <= 1.0 {
            let x = start.x + t * dir.x;
            if x >= rect_left && x <= rect_right {
                if t < closest_t {
                    closest_t = t;
                    found = true;
                }
            }
        }
    }

    if found {
        Some(start + dir * closest_t)
    } else {
        None
    }
}

struct RaycastHit {
    hit: Vec2,
    distance: f32,
    rect_idx: usize,
}

fn raycast(start: Vec2, end: Vec2, rects: &[Rect]) -> Option<RaycastHit> {
    let mut result = None;
    let mut closest_distance = f32::MAX;
    for (i, rect) in rects.iter().enumerate() {
        let intersection = intersect_line(rect, start, end);
        if let Some(hit) = intersection {
            let distance = (hit - start).length();
            if distance < closest_distance {
                result = Some(RaycastHit {
                    hit,
                    distance,
                    rect_idx: i,
                });
                closest_distance = distance;
            }
        }
    }
    result
}

struct MinHeapEntry<T>(T, f32);
impl<T> PartialEq for MinHeapEntry<T> {
    fn eq(&self, other: &Self) -> bool {
        self.1 == other.1
    }
}
impl<T> Eq for MinHeapEntry<T> {}
impl<T> PartialOrd for MinHeapEntry<T> {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        other.1.partial_cmp(&self.1)
    }
}
impl<T> Ord for MinHeapEntry<T> {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other
            .1
            .partial_cmp(&self.1)
            .unwrap_or(std::cmp::Ordering::Equal)
    }
}

fn naive_pathfind(start: Vec2, goal: Vec2, rects: &[Rect]) -> Option<Vec<Vec2>> {
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

fn distance(a: Vec2, b: Vec2) -> f32 {
    (a - b).length()
}

fn greedy_pathfind(start: Vec2, goal: Vec2, rects: &[Rect]) -> Option<Vec<Vec2>> {
    let mut points = vec![start, goal];
    let mut rect_corners = Vec::new();
    let start_idx = 0;
    let goal_idx = 1;
    for rect in rects {
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
                distance(points[start_idx], points[corner]) // dist
                    + distance(points[corner], points[goal_idx]), // A* heuristic
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

        let cur_dist = dist[&parent] + distance(points[parent], points[current]);
        // println!("Current: {}, Parent: {}, cost: {}", current, parent, cost);
        // println!("Queue size: {}", queue.len());
        if dist.contains_key(&current) && dist[&current] <= cur_dist {
            continue;
        }

        if let Some(hit) = raycast(points[parent], points[current], &rects) {
            // Current node not visible from parent
            for corner in rect_corners[hit.rect_idx] {
                if corner == current {
                    continue;
                }
                queue.push(MinHeapEntry(
                    (corner, parent),
                    dist[&parent]
                        + distance(points[parent], points[corner])
                        + distance(points[corner], points[goal_idx]),
                ));
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

#[macroquad::main("Wire pathfinding")]
async fn main() {
    // let rects = vec![
    //     Rect::new(100.0, 100.0, 100.0, 100.0),
    //     Rect::new(300.0, 300.0, 400.0, 100.0),
    //     Rect::new(200.0, 250.0, 50.0, 100.0),
    // ];

    // Evenly spaced rectangles
    let mut rects = Vec::new();
    let scale = 10.0;
    for i in 1..40 {
        for j in 1..40 {
            rects.push(Rect::new(
                i as f32 * scale + scale / 2.0,
                j as f32 * scale + scale / 2.0,
                scale / 2.0,
                scale / 2.0,
            ));
        }
    }

    // Randomly placed rectangles
    // let mut rects = Vec::new();
    // rand::srand(42);
    // let scale = 40.0;
    // for i in 1..10 {
    //     for j in 1..10 {
    //         rects.push(Rect::new(
    //             i as f32 * scale + rand::gen_range(-scale / 5.0, scale / 5.0),
    //             j as f32 * scale + rand::gen_range(-scale / 5.0, scale / 5.0),
    //             scale / 2.0,
    //             scale / 2.0,
    //         ));
    //     }
    // }

    loop {
        clear_background(WHITE);

        for rect in &rects {
            draw_rectangle(rect.x, rect.y, rect.w, rect.h, BLUE);
        }

        let goal_pos = Vec2::new(mouse_position().0, mouse_position().1);

        // if let Some(path) = naive_pathfind(Vec2::new(0.0, 0.0), goal_pos, &rects) {
        if let Some(path) = greedy_pathfind(Vec2::new(0.0, 0.0), goal_pos, &rects) {
            for i in 0..path.len() {
                draw_circle(path[i].x, path[i].y, 5.0, RED);
            }
            for i in 0..path.len() - 1 {
                draw_line(
                    path[i].x,
                    path[i].y,
                    path[i + 1].x,
                    path[i + 1].y,
                    2.0,
                    GRAY,
                );
            }
        }

        next_frame().await
    }
}
