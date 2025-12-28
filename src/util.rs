use macroquad::prelude::*;

pub fn line_rect_intersect(rect: &Rect, start: Vec2, end: Vec2) -> Option<Vec2> {
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

pub struct RaycastHit {
    pub hit: Vec2,
    pub distance: f32,
    pub rect_idx: usize,
}

pub fn raycast(start: Vec2, end: Vec2, rects: &[Rect]) -> Option<RaycastHit> {
    let mut result = None;
    let mut closest_distance = f32::MAX;
    for (i, rect) in rects.iter().enumerate() {
        let intersection = line_rect_intersect(rect, start, end);
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

pub struct MinHeapEntry<T>(pub T, pub f32);
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
