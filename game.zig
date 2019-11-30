const std = @import("std");
const math = @import("math.zig");

const Vector2 = math.Vector2;
const Vector3 = math.Vector3;
const Vector4 = math.Vector4;
const Mat4x4 = math.Mat4x4;

const sdl = @cImport({
    @cInclude("SDL2/SDL.h");
});

const Color = struct {
    r: u8 = 0,
    g: u8 = 0,
    b: u8 = 0,
    a: u8 = 0,
};

const COLOR_WHITE = Color{ .r = 255, .g = 255, .b = 255, .a = 255 };
const COLOR_GRAY = Color{ .r = 128, .g = 128, .b = 128, .a = 255 };
const COLOR_RED = Color{ .r = 255, .a = 255 };
const COLOR_GREEN = Color{ .g = 255, .a = 255 };
const COLOR_BLUE = Color{ .b = 255, .a = 255 };

const Time = struct {
    elapsed_from_start: f64,
    dt: f32,
};

const InputState = struct {
    left: bool = false,
    right: bool = false,
    forward: bool = false,
    back: bool = false,
};

const Fixed = struct {
    pos: Vector3 = Vector3{},
};

const Mover = struct {
    velocity: Vector3 = Vector3{},
    pos: Vector3 = Vector3{},
    scale: f32 = 1,
    rot: f32 = 0,
};

const Camera = struct {
    pos: Vector3 = Vector3{},
};

const DrawInfo = struct {
    renderer: *sdl.SDL_Renderer,
    camera: Camera,
    viewport_offset: Vector3,
};

const PositionList = std.ArrayList(Vector3);
const MoverList = std.ArrayList(Mover);

const GameState = struct {
    renderer: *sdl.SDL_Renderer,
    window: *sdl.SDL_Window,
    quit: bool,
    previous_time: Time,
    rng: std.rand.DefaultPrng,

    camera: Camera,
    world_bounds: Vector2,
    stars: PositionList,
    movers: MoverList,
    input: InputState,

    debug_camera: bool,
};

const MoverType = enum {
    PlayerShip,
    Asteroid,
};

fn get_window_size(window: *sdl.SDL_Window) Vector2 {
    var width: c_int = 0;
    var height: c_int = 0;
    sdl.SDL_GetWindowSize(window, &width, &height);

    return Vector2{
        .x = @intToFloat(f32, width),
        .y = @intToFloat(f32, height),
    };
}

pub fn main() u8 {
    if (sdl.SDL_Init(sdl.SDL_INIT_VIDEO | sdl.SDL_INIT_AUDIO) != 0) {
        std.debug.panic("sdl.SDL_Init failed: {c}\n", sdl.SDL_GetError());
    }
    defer sdl.SDL_Quit();

    // TODO sdl.SDL_WINDOW_RESIZABLE - game exits with error code when we resize??
    var window: *sdl.SDL_Window = sdl.SDL_CreateWindow(
        c"Zigtroids",
        200,
        200,
        @intCast(c_int, 640),
        @intCast(c_int, 400),
        sdl.SDL_WINDOW_OPENGL | sdl.SDL_WINDOW_ALLOW_HIGHDPI,
    ).?;
    defer sdl.SDL_DestroyWindow(window);

    var renderer: *sdl.SDL_Renderer = sdl.SDL_CreateRenderer(window, -1, sdl.SDL_RENDERER_ACCELERATED).?;
    defer sdl.SDL_DestroyRenderer(renderer);

    const world_bounds = math.Vector2{ .x = 1600, .y = 1000 };

    var gamestate = GameState{
        .renderer = renderer,
        .window = window,
        .quit = false,
        .previous_time = Time{
            .elapsed_from_start = 0,
            .dt = 0,
        },
        .rng = std.rand.DefaultPrng.init(0),
        .camera = Camera{},
        .world_bounds = world_bounds,
        .input = InputState{},
        .stars = PositionList.init(std.heap.c_allocator),
        .movers = MoverList.init(std.heap.c_allocator),

        .debug_camera = false,
    };

    // stars
    gamestate.stars.resize(2000) catch |err| std.debug.warn("failed to alloc ships: {}", err);
    for (gamestate.stars.toSlice()) |*pos| {
        const x = gamestate.rng.random.float(f32) * gamestate.world_bounds.x;
        const y = gamestate.rng.random.float(f32) * gamestate.world_bounds.y;
        pos.x = x;
        pos.y = y;
    }

    // ship and asteroids
    gamestate.movers.resize(5) catch |err| std.debug.warn("failed to alloc ships: {}", err);
    {
        var rng = &gamestate.rng.random;

        for (gamestate.movers.toSlice()) |*mover, index| {
            if (index == 0) {
                mover.pos = Vector3{ .x = 200, .y = 200, .z = 0 };
                mover.scale = 20;
            } else {
                const x = rng.float(f32) * gamestate.world_bounds.x;
                const y = rng.float(f32) * gamestate.world_bounds.y;
                const scale = 40 + rng.float(f32) * 40.0;
                const speed = 0.005 + rng.float(f32) * 0.035;
                const xdir = rng.float(f32) * 2.0 - 1.0;
                const ydir = rng.float(f32) * 2.0 - 1.0;

                mover.pos = Vector3{ .x = x, .y = y, .z = 0 };
                mover.scale = scale;
                mover.velocity = (Vector3{ .x = xdir, .y = ydir, .z = 0 }).normalize().scale(speed);
            }
        }
    }

    while (!gamestate.quit) {
        const time: Time = get_time(gamestate.previous_time);
        game_tick(&gamestate, time);
        gamestate.previous_time = time;

        var event: sdl.SDL_Event = undefined;
        while (sdl.SDL_PollEvent(&event) == 1) {
            if (event.type == sdl.SDL_WINDOWEVENT) {
                if (event.window.event == sdl.SDL_WINDOWEVENT_CLOSE) {
                    gamestate.quit = true;
                }
            } else if (event.type == sdl.SDL_KEYDOWN or event.type == sdl.SDL_KEYUP) {
                const value: bool = if (event.key.type == sdl.SDL_KEYDOWN) true else false;
                if (event.key.keysym.sym == sdl.SDLK_LEFT) {
                    gamestate.input.left = value;
                } else if (event.key.keysym.sym == sdl.SDLK_RIGHT) {
                    gamestate.input.right = value;
                } else if (event.key.keysym.sym == sdl.SDLK_UP) {
                    gamestate.input.forward = value;
                } else if (event.key.keysym.sym == sdl.SDLK_DOWN) {
                    gamestate.input.back = value;
                } else if (event.key.keysym.sym == sdl.SDLK_SPACE and event.type == sdl.SDL_KEYUP) {
                    std.debug.warn("toggled\n");
                    gamestate.debug_camera = !gamestate.debug_camera;
                }
            }
        }
    }

    return 0;
}

// fn bit_set(byte: u8, bit: u8, value: u8) u8 {
//     return byte ^ ((-%value ^ byte) & @shlExact(1, bit));
// }

// fn bit_clear(byte: u8, bit: u8) u8 {
//     return bit_set(byte, bit, 0);
// }

fn log_sdl_error() void {
    var c_err = sdl.SDL_GetError() orelse return;
    var err = std.mem.toSliceConst(u8, c_err);
    std.debug.warn("Error in SDL: {}\n", err);
}

fn get_time(prev: Time) Time {
    var counter: u64 = sdl.SDL_GetPerformanceCounter();
    var frequency: u64 = sdl.SDL_GetPerformanceFrequency();

    var current: f64 = @intToFloat(f64, counter) / @intToFloat(f64, frequency);

    return Time{
        .elapsed_from_start = current,
        .dt = @floatCast(f32, current - prev.elapsed_from_start),
    };
}

fn game_tick(state: *GameState, time: Time) void {
    var draw_info = DrawInfo{
        .renderer = state.renderer,
        .camera = state.camera,
        .viewport_offset = get_window_size(state.window).toVector3().scale(0.5),
    };

    if (sdl.SDL_SetRenderDrawColor(state.renderer, 0, 0, 0, 255) < 0) {
        log_sdl_error();
    }
    if (sdl.SDL_RenderClear(state.renderer) < 0) {
        log_sdl_error();
    }

    // draw grid lines
    if (false) {
        if (sdl.SDL_SetRenderDrawColor(state.renderer, 45, 45, 45, 255) < 0) {
            log_sdl_error();
        }

        const step: i32 = 10;
        var x: c_int = 0;
        var y: c_int = 0;
        while (x < state.world_bounds.x) {
            x += step;
            if (sdl.SDL_RenderDrawLine(state.renderer, x, 0, x, state.world_bounds.y) < 0) {
                log_sdl_error();
            }
        }

        while (y < state.world_bounds.y) {
            y += step;
            if (sdl.SDL_RenderDrawLine(state.renderer, 0, y, state.world_bounds.x, y) < 0) {
                log_sdl_error();
            }
        }
    }

    // draw starfield
    {
        draw_stars(draw_info, state.stars.toSliceConst());
    }

    // draw world border
    {
        const border_points = [_]Vector3{
            Vector3{ .x = 0, .y = 0 },
            Vector3{
                .x = state.world_bounds.x,
                .y = 0,
            },
            Vector3{
                .x = state.world_bounds.x,
                .y = state.world_bounds.y,
            },
            Vector3{
                .x = 0,
                .y = state.world_bounds.y,
            },
            Vector3{ .x = 0, .y = 0 },
        };

        var points = transform_points_with_positions(draw_info, border_points);
        draw_line_strip(draw_info, COLOR_GREEN, points.toSlice());
    }

    // debug camera or ship steering
    if (state.debug_camera) {
        var x: f32 = 0.0;
        var y: f32 = 0.0;

        var step: f32 = 700.0 * time.dt;

        x += if (state.input.right) step else 0.0;
        x += if (state.input.left) -step else 0.0;
        y += if (state.input.back) step else 0.0;
        y += if (state.input.forward) -step else 0.0;

        state.camera.pos = state.camera.pos.add(Vector3{ .x = x, .y = y });
    } else {
        const playerIndex = 0;
        var mover = &state.movers.toSlice()[playerIndex];

        var rot: f32 = 0.0;
        rot += if (state.input.left) @as(f32, 5.0) else 0.0;
        rot -= if (state.input.right) @as(f32, 5.0) else 0.0;

        const period = std.math.pi * 2.0;
        mover.rot = @mod(mover.rot + rot * time.dt + period, period);

        if (state.input.forward) {
            const MAX_ACCELERATION = 0.06;
            const MAX_VELOCITY_MAGNITUDE = 0.5;
            const MAX_VELOCITY_MAGNITUDE_SQ = MAX_VELOCITY_MAGNITUDE * MAX_VELOCITY_MAGNITUDE;

            const forward = Vector3{ .x = 0, .y = 1, .z = 0 };
            const playerForward = forward.rotateZ(mover.rot).normalize().scale(MAX_ACCELERATION * time.dt);
            mover.velocity = mover.velocity.add(playerForward);
            if (mover.velocity.lengthSq() > MAX_VELOCITY_MAGNITUDE_SQ) {
                mover.velocity = mover.velocity.normalize().scale(MAX_VELOCITY_MAGNITUDE);
            }
        }
    }

    std.debug.warn("camera pos: ({d:.2}, {d:.2})\n", state.camera.pos.x, state.camera.pos.y);
    std.debug.warn("player pos: ({d:.2}, {d:.2})\n", state.movers.at(0).pos.x, state.movers.at(0).pos.y);

    var collidingState = std.ArrayList(bool).init(std.heap.c_allocator);
    collidingState.resize(state.movers.count()) catch |err| std.debug.warn("failed to resize collidingState");
    for (state.movers.toSlice()) |*mover1, i| {
        for (state.movers.toSlice()) |mover2, j| {
            if (i == j) {
                continue;
            }
            var collisionLength = mover1.scale + mover2.scale;
            var distanceSq = mover1.pos.sub(mover2.pos).lengthSq();
            var isColliding = distanceSq <= collisionLength * collisionLength;

            collidingState.set(i, collidingState.at(i) or isColliding);
            collidingState.set(j, collidingState.at(j) or isColliding);

            // if (isColliding and i == 0) {
            //     var length = mover1.velocity.length();
            //     var bounceDir = mover1.velocity.normalize();
            //     var bounceVel = bounceDir.scale(-length * 0.8);
            //     std.debug.warn("{} {}\n", bounceVel.x, bounceVel.y);
            //     mover1.velocity = bounceVel;
            //     mover1.pos = mover1.pos.add(bounceDir.scale(-std.math.sqrt(distanceSq)));
            // }
        }
    }

    if (!state.debug_camera) {
        state.camera.pos = state.movers.at(0).pos;
    }

    // std.debug.warn("{} {}\n", collidingState.at(0), collidingState.at(1));

    for (state.movers.toSlice()) |*mover, index| {
        mover.pos = mover.pos.add(mover.velocity);
        mover.pos.x = @mod(mover.pos.x + state.world_bounds.x, state.world_bounds.x);
        mover.pos.y = @mod(mover.pos.y + state.world_bounds.y, state.world_bounds.y);

        const objectType = switch (index) {
            0 => MoverType.PlayerShip,
            else => MoverType.Asteroid,
        };
        draw_object(draw_info, mover, objectType, collidingState.at(index));
    }

    sdl.SDL_RenderPresent(state.renderer);
}

fn vector3_to_sdl(v: Vector3) sdl.SDL_Point {
    return sdl.SDL_Point{
        .x = @floatToInt(c_int, v.x),
        .y = @floatToInt(c_int, v.y),
    };
}

// fn vector3_to_sdl(v: Vector3) sdl.SDL_Point {
//     return sdl.SDL_Point{
//         .x = @floatToInt(c_int, v.x),
//         .y = @floatToInt(c_int, v.y),
//     };
// }

const SHIP_POINTS = [_]Vector3{
    Vector3{ .x = 0.0, .y = 0.5, .z = 0.0 },
    Vector3{ .x = 0.5, .y = -0.35, .z = 0.0 },
    Vector3{ .x = 0.0, .y = 0.0, .z = 0.0 },
    Vector3{ .x = -0.5, .y = -0.35, .z = 0.0 },
    Vector3{ .x = 0.0, .y = 0.5, .z = 0.0 },
};

const ASTEROID_POINTS = [_]Vector3{
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

const CIRCLE_POINTS = generate_circle();

const SdlPointsList = std.ArrayList(sdl.SDL_Point);

fn generate_circle() [32]Vector3 {
    var points: [32]Vector3 = undefined;
    for (points) |*point, i| {
        var rads = std.math.pi * 2.0 * (@intToFloat(f32, i) / @intToFloat(f32, points.len - 1));
        const x: f32 = std.math.cos(rads);
        const y: f32 = std.math.sin(rads);
        point.* = Vector3{ .x = x, .y = y, .z = 0 };
    }
    return points;
}

fn draw_bounding_circle(draw_info: DrawInfo, mover: *const Mover, isColliding: bool) void {
    const color = if (isColliding) COLOR_RED else COLOR_GRAY;
    const sdlPoints = transform_points_with_mover(draw_info, CIRCLE_POINTS, mover);

    draw_line_strip(draw_info, color, sdlPoints.toSliceConst());
}

fn draw_object(draw_info: DrawInfo, mover: *const Mover, objectType: MoverType, isColliding: bool) void {
    const points = switch (objectType) {
        MoverType.PlayerShip => SHIP_POINTS,
        MoverType.Asteroid => ASTEROID_POINTS,
    };

    var sdlPoints = transform_points_with_mover(draw_info, points, mover);

    draw_line_strip(draw_info, COLOR_GRAY, sdlPoints.toSliceConst());

    draw_bounding_circle(draw_info, mover, isColliding);
}

fn draw_stars(draw_info: DrawInfo, points: []const Vector3) void {
    var sdlPoints = transform_points_with_positions(draw_info, points);
    draw_points(draw_info, COLOR_GRAY, sdlPoints.toSliceConst());
}

fn transform_point_to_screen_space(point: Vector3, draw_info: *const DrawInfo) Vector3 {
    return point.sub(draw_info.camera.pos).add(draw_info.viewport_offset);
}

fn transform_points_with_positions(draw_info: DrawInfo, points: []const Vector3) SdlPointsList {
    var sdlPoints = SdlPointsList.init(std.heap.c_allocator);
    sdlPoints.resize(points.len) catch |err| std.debug.warn("Failed to resize sdlPoints: {}", err);

    for (points) |point, i| {
        var p = transform_point_to_screen_space(point, &draw_info);
        sdlPoints.set(i, vector3_to_sdl(p));
    }

    return sdlPoints;
}

fn transform_points_with_mover(draw_info: DrawInfo, points: []const Vector3, mover: *const Mover) SdlPointsList {
    var sdlPoints = SdlPointsList.init(std.heap.c_allocator);
    sdlPoints.resize(points.len) catch |err| std.debug.warn("Failed to resize sdlPoints: {}", err);

    for (points) |point, i| {
        var p = point.scale(mover.scale);
        p = p.rotateZ(mover.rot);
        p = p.add(mover.pos);
        p = transform_point_to_screen_space(p, &draw_info);
        sdlPoints.set(i, vector3_to_sdl(p));
    }

    return sdlPoints;
}

fn draw_line_strip(draw_info: DrawInfo, color: Color, points: []const sdl.SDL_Point) void {
    if (sdl.SDL_SetRenderDrawColor(draw_info.renderer, color.r, color.g, color.b, color.a) < 0) {
        log_sdl_error();
    }

    var cPoints: [*c]const sdl.SDL_Point = &points[0];
    if (sdl.SDL_RenderDrawLines(draw_info.renderer, cPoints, @intCast(c_int, points.len)) < 0) {
        log_sdl_error();
    }
}

fn draw_points(draw_info: DrawInfo, color: Color, points: []const sdl.SDL_Point) void {
    if (sdl.SDL_SetRenderDrawColor(draw_info.renderer, color.r, color.g, color.b, color.a) < 0) {
        log_sdl_error();
    }

    var cPoints: [*c]const sdl.SDL_Point = &points[0];
    if (sdl.SDL_RenderDrawPoints(draw_info.renderer, cPoints, @intCast(c_int, points.len)) < 0) {
        log_sdl_error();
    }
}
