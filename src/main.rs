mod algorithms;
mod bvh;
mod util;

use macroquad::prelude::*;

#[macroquad::main("Wire pathfinding")]
async fn main() {
    // Test scenes, uncomment to try

    // let rects = vec![
    //     Rect::new(100.0, 100.0, 100.0, 100.0),
    //     Rect::new(300.0, 300.0, 400.0, 100.0),
    //     Rect::new(200.0, 250.0, 50.0, 100.0),
    // ];

    // // Evenly spaced rectangles
    // let mut rects = Vec::new();
    // let scale = 8.0;
    // for i in 1..50 {
    //     for j in 1..50 {
    //         rects.push(Rect::new(
    //             i as f32 * scale + scale / 2.0,
    //             j as f32 * scale + scale / 2.0,
    //             scale / 2.0,
    //             scale / 2.0,
    //         ));
    //     }
    // }

    // Randomly placed rectangles
    let mut rects = Vec::new();
    rand::srand(42); // Set seed
    let scale = 10.0;
    for i in 1..500 {
        for j in 1..500 {
            rects.push(Rect::new(
                i as f32 * scale + rand::gen_range(-scale / 5.0, scale / 5.0),
                j as f32 * scale + rand::gen_range(-scale / 5.0, scale / 5.0),
                scale / 2.0,
                scale / 2.0,
            ));
        }
    }

    // Randomly placed/sized rectangles
    // let mut rects = Vec::new();
    // rand::srand(42); // Set seed
    // let scale = 5.0;
    // for i in 1..150 {
    //     for j in 1..150 {
    //         let new_rect = Rect::new(
    //             i as f32 * scale + rand::gen_range(-scale / 5.0, scale / 5.0),
    //             j as f32 * scale + rand::gen_range(-scale / 5.0, scale / 5.0),
    //             rand::gen_range(0.5, 10.0) * scale / 2.0,
    //             rand::gen_range(0.5, 10.0) * scale / 2.0,
    //         );
    //         let eps = 0.1;
    //         let padded_rect = Rect::new(
    //             new_rect.x - eps,
    //             new_rect.y - eps,
    //             new_rect.w + 2.0 * eps,
    //             new_rect.h + 2.0 * eps,
    //         );
    //         let mut overlap = false;
    //         for rect in &rects {
    //             overlap |= padded_rect.overlaps(&rect);
    //         }
    //         if !overlap {
    //             rects.push(new_rect);
    //         }
    //     }
    // }

    // Make BVH
    println!("Building BVH...");
    let bvh = bvh::Bvh::build(&rects);

    println!("Starting main loop...");
    loop {
        clear_background(WHITE);

        for rect in &rects {
            draw_rectangle(rect.x, rect.y, rect.w, rect.h, BLUE);
        }

        let goal_pos = Vec2::new(mouse_position().0, mouse_position().1);

        // if let Some(path) = naive_pathfind(Vec2::new(0.0, 0.0), goal_pos, &rects) {
        // if let Some(path) = algorithms::greedy_pathfind(Vec2::new(0.0, 0.0), goal_pos, &rects) {
        if let Some(path) = algorithms::bvh_pathfind(Vec2::new(0.0, 0.0), goal_pos, &rects, &bvh) {
            for point in &path {
                draw_circle(point.x, point.y, 5.0, RED);
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
