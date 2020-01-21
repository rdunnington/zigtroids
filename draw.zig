const std = @import("std");
const math = @import("math.zig");

const Vector2 = math.Vector2;
const Vector3 = math.Vector3;
const Vector4 = math.Vector4;
const Mat4x4 = math.Mat4x4;

const sdl = @cImport({
    @cInclude("SDL2/SDL.h");
});

// =======================================================
// helpers

const SHIP_POINTS = &[_]Vector3{
    Vector3{ .x = 1, .y = 0.0, .z = 0.0 },
    Vector3{ .x = -0.7, .y = -1, .z = 0.0 },
    Vector3{ .x = 0.0, .y = 0.0, .z = 0.0 },
    Vector3{ .x = -0.7, .y = 1, .z = 0.0 },
    Vector3{ .x = 1, .y = 0.0, .z = 0.0 },
};

const ASTEROID_POINTS = &[_]Vector3{
    Vector3{ .x = 0.0, .y = 0.8, .z = 0.0 },
    Vector3{ .x = 0.8, .y = 0.4, .z = 0.0 },
    Vector3{ .x = 0.6, .y = -0.2, .z = 0.0 },
    Vector3{ .x = 0.2, .y = -0.6, .z = 0.0 },
    Vector3{ .x = -0.2, .y = -0.4, .z = 0.0 },
    Vector3{ .x = -0.4, .y = -0.1, .z = 0.0 },
    Vector3{ .x = -0.6, .y = 0.2, .z = 0.0 },
    Vector3{ .x = -0.4, .y = 0.6, .z = 0.0 },
    Vector3{ .x = 0.0, .y = 0.8, .z = 0.0 },
};

const BULLET_POINTS = &[_]Vector3{
    Vector3{ .x = 1.0, .y = -1.0 },
    Vector3{ .x = 1.0, .y = 1.0 },
    Vector3{ .x = -1.0, .y = 1.0 },
    Vector3{ .x = -1.0, .y = -1.0 },
    Vector3{ .x = 1.0, .y = -1.0 },
};

const CIRCLE_POINTS = generate_circle();

const SdlPointsList = std.ArrayList(sdl.SDL_Point);

fn vector3_to_sdl(v: Vector3) sdl.SDL_Point {
    return sdl.SDL_Point{
        .x = @floatToInt(c_int, v.x),
        .y = @floatToInt(c_int, v.y),
    };
}

fn generate_circle() []Vector3 {
    var points: [32]Vector3 = undefined;
    for (points) |*point, i| {
        var rads = std.math.pi * 2.0 * (@intToFloat(f32, i) / @intToFloat(f32, points.len - 1));
        const x: f32 = std.math.cos(rads);
        const y: f32 = std.math.sin(rads);
        point.* = Vector3{ .x = x, .y = y, .z = 0 };
    }
    return points;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// public interface

pub const MoverType = enum {
    PlayerShip,
    Asteroid,
    Bullet,
    Fighter,
};

pub const Camera = struct {
    pos: Vector3 = Vector3{},
};

pub const DrawInfo = struct {
    renderer: *sdl.SDL_Renderer,
    camera: Camera,
    viewport_offset: Vector3,
    world_bounds: Vector2,
};

pub fn draw_points(draw_info: DrawInfo, color: Color, points: []const sdl.SDL_Point) void {
    if (sdl.SDL_SetRenderDrawColor(draw_info.renderer, color.r, color.g, color.b, color.a) < 0) {
        log_sdl_error();
    }

    var cPoints: [*c]const sdl.SDL_Point = &points[0];
    if (sdl.SDL_RenderDrawPoints(draw_info.renderer, cPoints, @intCast(c_int, points.len)) < 0) {
        log_sdl_error();
    }
}

pub fn draw_line_strip(draw_info: DrawInfo, color: Color, points: []const sdl.SDL_Point) void {
    if (sdl.SDL_SetRenderDrawColor(draw_info.renderer, color.r, color.g, color.b, color.a) < 0) {
        log_sdl_error();
    }

    var cPoints: [*c]const sdl.SDL_Point = &points[0];
    if (sdl.SDL_RenderDrawLines(draw_info.renderer, cPoints, @intCast(c_int, points.len)) < 0) {
        log_sdl_error();
    }
}

pub fn draw_bounding_circle(draw_info: DrawInfo, mover: *const Mover, is_colliding: bool) void {
    const color = if (is_colliding) COLOR_RED else COLOR_GRAY;
    const sdl_points = transform_points_with_mover(draw_info, CIRCLE_POINTS, mover);

    draw_line_strip(draw_info, color, sdl_points.toSliceConst());
    sdl_points.deinit();
}

pub fn draw_object(draw_info: DrawInfo, mover: *const Mover, object_type: MoverType, is_colliding: bool) void {
    const points: []Vector3 = switch (object_type) {
        MoverType.PlayerShip => SHIP_POINTS,
        MoverType.Asteroid => ASTEROID_POINTS,
        MoverType.Bullet => BULLET_POINTS,
        MoverType.Fighter => SHIP_POINTS,
    };
    const color = switch (object_type) {
        MoverType.PlayerShip => COLOR_GRAY,
        MoverType.Asteroid => COLOR_GRAY,
        MoverType.Bullet => COLOR_GRAY,
        MoverType.Fighter => COLOR_GREEN,
    };

    var sdl_points = transform_points_with_mover(draw_info, points, mover);

    draw_line_strip(draw_info, color, sdl_points.toSliceConst());
    sdl_points.deinit();

    // draw_bounding_circle(draw_info, mover, is_colliding);

    // wrapped in worldspace
    // {
    //     const to_edge_left: f32 = mover.pos.x;
    //     const to_edge_right: f32 = draw_info.world_bounds.x - mover.pos.x;
    //     const to_edge_top: f32 = mover.pos.y;
    //     const to_edge_bot: f32 = draw_info.world_bounds.y - mover.pos.y;

    //     const mirror_mover_x = Mover{

    //     };

    //     const mirror_x = if (to_edge_left < to_edge_right) draw_info.world_bounds.x + mover.pos.x else -to_edge_right;
    //     const mirror_y = if (to_edge_top < to_edge_bot) draw_info.world_bounds.y + mover.pos.y else -to_edge_bot;
    //     const mirror_mover = Mover{
    //         .pos = Vector3{ .x = mirror_x, .y = mirror_y, .z = mover.pos.z },
    //         .scale = mover.scale,
    //         .rot = mover.rot,
    //     };

    //     sdl_points = transform_points_with_mover(draw_info, points, &mirror_mover);
    //     draw_line_strip(draw_info, COLOR_BLUE, sdl_points.toSliceConst());
    //     draw_bounding_circle(draw_info, &mirror_mover, is_colliding);
    // }
}

pub fn draw_stars(draw_info: DrawInfo, points: []const Vector3) void {
    var sdl_points = transform_points_with_positions(draw_info, points);
    draw_points(draw_info, COLOR_GRAY, sdl_points.toSliceConst());
    sdl_points.deinit();
}

pub fn transform_point_to_screen_space(point: Vector3, draw_info: *const DrawInfo) Vector3 {
    return point.sub(draw_info.camera.pos).add(draw_info.viewport_offset);
}

pub fn transform_points_with_positions(draw_info: DrawInfo, points: []const Vector3) SdlPointsList {
    var sdl_points = SdlPointsList.init(std.heap.c_allocator);
    sdl_points.resize(points.len) catch |err| std.debug.warn("Failed to resize sdl_points: {}", .{err});

    for (points) |point, i| {
        var p = transform_point_to_screen_space(point, &draw_info);
        sdl_points.set(i, vector3_to_sdl(p));
    }

    return sdl_points;
}

pub fn transform_points_with_mover(draw_info: DrawInfo, points: []const Vector3, mover: *const Mover) SdlPointsList {
    var sdl_points = SdlPointsList.init(std.heap.c_allocator);
    sdl_points.resize(points.len) catch |err| std.debug.warn("Failed to resize sdl_points: {}", .{err});

    for (points) |point, i| {
        var p = point.scale(mover.scale);
        p = p.rotateZ(mover.rot);
        p = p.add(mover.pos);
        p = transform_point_to_screen_space(p, &draw_info);
        sdl_points.set(i, vector3_to_sdl(p));
    }

    return sdl_points;
}
