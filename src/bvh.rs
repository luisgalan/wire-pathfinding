/// Couldn't find a good BVH implementation online so I got claude to write this module
use macroquad::math::{Rect, Vec2};

trait RectExt {
    const EMPTY: Rect;
    fn union(self, other: Rect) -> Rect;
    fn centroid(self) -> Vec2;
    fn surface_area(self) -> f32;
    fn ray_intersect(self, origin: Vec2, inv_dir: Vec2) -> Option<f32>;
}

impl RectExt for Rect {
    const EMPTY: Rect = Rect {
        x: f32::INFINITY,
        y: f32::INFINITY,
        w: f32::NEG_INFINITY,
        h: f32::NEG_INFINITY,
    };

    #[inline]
    fn union(self, other: Rect) -> Rect {
        if self.w < 0.0 || self.h < 0.0 {
            return other;
        }
        if other.w < 0.0 || other.h < 0.0 {
            return self;
        }

        let min_x = self.x.min(other.x);
        let min_y = self.y.min(other.y);
        let max_x = (self.x + self.w).max(other.x + other.w);
        let max_y = (self.y + self.h).max(other.y + other.h);

        Rect::new(min_x, min_y, max_x - min_x, max_y - min_y)
    }

    #[inline]
    fn centroid(self) -> Vec2 {
        Vec2::new(self.x + self.w * 0.5, self.y + self.h * 0.5)
    }

    #[inline]
    fn surface_area(self) -> f32 {
        2.0 * (self.w + self.h)
    }

    #[inline]
    fn ray_intersect(self, origin: Vec2, inv_dir: Vec2) -> Option<f32> {
        let min = Vec2::new(self.x, self.y);
        let max = Vec2::new(self.x + self.w, self.y + self.h);

        let t1 = (min - origin) * inv_dir;
        let t2 = (max - origin) * inv_dir;

        let tmin = t1.min(t2);
        let tmax = t1.max(t2);

        let tmin = tmin.x.max(tmin.y);
        let tmax = tmax.x.min(tmax.y);

        if tmax >= tmin.max(0.0) {
            Some(tmin.max(0.0))
        } else {
            None
        }
    }
}

#[derive(Clone, Copy)]
struct BvhNode {
    bounds: Rect,
    left: u32,
    right_or_prim: i32,
}

impl BvhNode {
    #[inline]
    fn is_leaf(self) -> bool {
        self.right_or_prim < 0
    }

    #[inline]
    fn primitive_index(self) -> usize {
        debug_assert!(self.is_leaf());
        !self.right_or_prim as usize
    }

    #[inline]
    fn left_child(self) -> usize {
        self.left as usize
    }

    #[inline]
    fn right_child(self) -> usize {
        debug_assert!(!self.is_leaf());
        self.right_or_prim as usize
    }
}

pub struct Bvh {
    nodes: Vec<BvhNode>,
}

struct Primitive {
    bounds: Rect,
    centroid: Vec2,
    index: u32,
}

const SAH_TRAVERSAL_COST: f32 = 1.0;
const SAH_INTERSECT_COST: f32 = 1.0;
const SAH_BINS: usize = 16;

impl Bvh {
    pub fn build(rects: &[Rect]) -> Self {
        if rects.is_empty() {
            return Self { nodes: Vec::new() };
        }

        let mut primitives: Vec<Primitive> = rects
            .iter()
            .enumerate()
            .map(|(i, &bounds)| Primitive {
                bounds,
                centroid: bounds.centroid(),
                index: i as u32,
            })
            .collect();

        let mut nodes = Vec::with_capacity(2 * rects.len());
        Self::build_recursive(&mut primitives, &mut nodes);
        Self { nodes }
    }

    fn build_recursive(primitives: &mut [Primitive], nodes: &mut Vec<BvhNode>) -> u32 {
        let node_idx = nodes.len() as u32;

        let bounds = primitives
            .iter()
            .fold(Rect::EMPTY, |acc, p| acc.union(p.bounds));

        if primitives.len() == 1 {
            nodes.push(BvhNode {
                bounds,
                left: 0,
                right_or_prim: !(primitives[0].index as i32),
            });
            return node_idx;
        }

        let (best_axis, best_split_idx, best_cost) = Self::find_best_split(primitives, bounds);

        let split_idx = if best_cost < f32::INFINITY
            && best_split_idx > 0
            && best_split_idx < primitives.len()
        {
            primitives.sort_unstable_by(|a, b| {
                a.centroid[best_axis]
                    .partial_cmp(&b.centroid[best_axis])
                    .unwrap()
            });
            best_split_idx
        } else {
            let axis = if bounds.w > bounds.h { 0 } else { 1 };
            primitives
                .sort_unstable_by(|a, b| a.centroid[axis].partial_cmp(&b.centroid[axis]).unwrap());
            primitives.len() / 2
        };

        let (left_prims, right_prims) = primitives.split_at_mut(split_idx);

        nodes.push(BvhNode {
            bounds,
            left: 0,
            right_or_prim: 0,
        });

        let left_idx = Self::build_recursive(left_prims, nodes);
        let right_idx = Self::build_recursive(right_prims, nodes);

        nodes[node_idx as usize].left = left_idx;
        nodes[node_idx as usize].right_or_prim = right_idx as i32;

        node_idx
    }

    fn find_best_split(primitives: &[Primitive], bounds: Rect) -> (usize, usize, f32) {
        let n = primitives.len();
        if n <= 1 {
            return (0, 0, f32::INFINITY);
        }

        let centroid_min = primitives
            .iter()
            .fold(Vec2::splat(f32::INFINITY), |acc, p| acc.min(p.centroid));
        let centroid_max = primitives
            .iter()
            .fold(Vec2::splat(f32::NEG_INFINITY), |acc, p| acc.max(p.centroid));

        let parent_sa = bounds.surface_area();
        let mut best_axis = 0;
        let mut best_split = 0;
        let mut best_cost = f32::INFINITY;

        for axis in 0..2 {
            let extent = centroid_max[axis] - centroid_min[axis];
            if extent < 1e-6 {
                continue;
            }

            let mut bins = [(Rect::EMPTY, 0u32); SAH_BINS];

            let scale = SAH_BINS as f32 / extent;
            for p in primitives {
                let b = ((p.centroid[axis] - centroid_min[axis]) * scale) as usize;
                let b = b.min(SAH_BINS - 1);
                bins[b].0 = bins[b].0.union(p.bounds);
                bins[b].1 += 1;
            }

            let mut left_bounds = [Rect::EMPTY; SAH_BINS - 1];
            let mut left_counts = [0u32; SAH_BINS - 1];
            let mut running_bounds = Rect::EMPTY;
            let mut running_count = 0u32;

            for i in 0..SAH_BINS - 1 {
                running_bounds = running_bounds.union(bins[i].0);
                running_count += bins[i].1;
                left_bounds[i] = running_bounds;
                left_counts[i] = running_count;
            }

            running_bounds = Rect::EMPTY;
            running_count = 0;

            for i in (0..SAH_BINS - 1).rev() {
                running_bounds = running_bounds.union(bins[i + 1].0);
                running_count += bins[i + 1].1;

                if left_counts[i] == 0 || running_count == 0 {
                    continue;
                }

                let cost = SAH_TRAVERSAL_COST
                    + SAH_INTERSECT_COST
                        * (left_bounds[i].surface_area() * left_counts[i] as f32
                            + running_bounds.surface_area() * running_count as f32)
                        / parent_sa;

                if cost < best_cost {
                    best_cost = cost;
                    best_axis = axis;
                    best_split = left_counts[i] as usize;
                }
            }
        }

        (best_axis, best_split, best_cost)
    }

    /// Raycast from `from` to `to`. Returns Some(primitive_index) on hit, None otherwise.
    /// Traverses closer children first, so the returned hit tends to be close to the actual closest.
    pub fn raycast(&self, from: Vec2, to: Vec2) -> Option<usize> {
        if self.nodes.is_empty() {
            return None;
        }

        let dir = to - from;
        let max_t = dir.length();
        if max_t < 1e-9 {
            return None;
        }
        let dir = dir / max_t;

        let inv_dir = Vec2::new(
            if dir.x.abs() < 1e-9 {
                f32::INFINITY.copysign(dir.x)
            } else {
                1.0 / dir.x
            },
            if dir.y.abs() < 1e-9 {
                f32::INFINITY.copysign(dir.y)
            } else {
                1.0 / dir.y
            },
        );

        let mut stack = [0u32; 64];
        let mut stack_ptr = 1;
        stack[0] = 0;

        while stack_ptr > 0 {
            stack_ptr -= 1;
            let node = self.nodes[stack[stack_ptr] as usize];

            if let Some(t) = node.bounds.ray_intersect(from, inv_dir) {
                if t > max_t {
                    continue;
                }

                if node.is_leaf() {
                    return Some(node.primitive_index());
                } else {
                    let left_idx = node.left_child();
                    let right_idx = node.right_child();
                    let left = &self.nodes[left_idx];
                    let right = &self.nodes[right_idx];

                    let t_left = left.bounds.ray_intersect(from, inv_dir);
                    let t_right = right.bounds.ray_intersect(from, inv_dir);

                    // Push further child first so closer child is popped first
                    match (t_left, t_right) {
                        (Some(tl), Some(tr)) => {
                            if tl < tr {
                                stack[stack_ptr] = right_idx as u32;
                                stack_ptr += 1;
                                stack[stack_ptr] = left_idx as u32;
                                stack_ptr += 1;
                            } else {
                                stack[stack_ptr] = left_idx as u32;
                                stack_ptr += 1;
                                stack[stack_ptr] = right_idx as u32;
                                stack_ptr += 1;
                            }
                        }
                        (Some(_), None) => {
                            stack[stack_ptr] = left_idx as u32;
                            stack_ptr += 1;
                        }
                        (None, Some(_)) => {
                            stack[stack_ptr] = right_idx as u32;
                            stack_ptr += 1;
                        }
                        (None, None) => {}
                    }
                }
            }
        }

        None
    }

    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }
}
