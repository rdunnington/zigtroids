const std = @import("std");
const math = @import("math.zig");

const Vector3 = math.Vector3;

const sdl = @cImport({
    @cInclude("SDL2/SDL.h");
});

const Time = struct {
    elapsed_from_start: f64,
    dt: f32,
};

const InputState = struct {
    left: bool = false,
    right: bool = false,
    forward: bool = false,
};

// comptime std.debug.assert(@sizeof(InputState) <= 1);

const Mover = struct {
    pos: math.Vector3 = math.Vector3{},
    scale: f32 = 1,
    rot: f32 = 0,
    velocity: Vector3,
};

const MoverList = std.ArrayList(Mover);
// const TransformList = std.ArrayList(math.Mat4x4);

const GameState = struct {
    renderer: *sdl.SDL_Renderer,
    window: *sdl.SDL_Window,
    quit: bool,
    previous_time: Time,
    rng: std.rand.DefaultPrng,

    movers: MoverList,
    input: InputState,
};

const MoverType = enum {
    PlayerShip,
    Asteroid,
};

const WindowSize = struct {
    width: f32,
    height: f32,
};

fn get_window_size(window: *sdl.SDL_Window) WindowSize {
    var width: c_int = 0;
    var height: c_int = 0;
    sdl.SDL_GetWindowSize(window, &width, &height);

    return WindowSize{
        .width = @intToFloat(f32, width),
        .height = @intToFloat(f32, height),
    };
}

pub fn main() u8 {
    if (sdl.SDL_Init(sdl.SDL_INIT_VIDEO | sdl.SDL_INIT_AUDIO) != 0) {
        std.debug.panic("sdl.SDL_Init failed: {c}\n", sdl.SDL_GetError());
    }
    defer sdl.SDL_Quit();

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

    var gamestate = GameState{
        .renderer = renderer,
        .window = window,
        .quit = false,
        .previous_time = Time{
            .elapsed_from_start = 0,
            .dt = 0,
        },
        .rng = std.rand.DefaultPrng.init(0),
        .input = InputState{},
        .movers = MoverList.init(std.heap.c_allocator),
        // .transforms = TransformList.init(std.heap.c_allocator),
    };

    gamestate.movers.resize(2) catch |err| std.debug.warn("failed to alloc ships: {}", err);
    {
        const WINDOWSIZE = get_window_size(gamestate.window);

        var rng = &gamestate.rng.random;

        for (gamestate.movers.toSlice()) |*mover, index| {
            if (index == 0) {
                mover.pos = Vector3{ .x = 200, .y = 200, .z = 0 };
                mover.scale = 20;
            } else {
                const x = rng.float(f32) * WINDOWSIZE.width;
                const y = rng.float(f32) * WINDOWSIZE.height;
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
    if (sdl.SDL_SetRenderDrawColor(state.renderer, 0, 0, 0, 255) < 0) {
        log_sdl_error();
    }
    if (sdl.SDL_RenderClear(state.renderer) < 0) {
        log_sdl_error();
    }

    // player input affects its mover
    {
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

    var collidingState = std.ArrayList(bool).init(std.heap.c_allocator);
    collidingState.resize(state.movers.count()) catch |err| std.debug.warn("failed to resize collidingState");
    for (state.movers.toSlice()) |mover1, i| {
        for (state.movers.toSlice()) |mover2, j| {
            if (i == j) {
                continue;
            }
            var lengthSq1 = mover1.scale * mover1.scale;
            var lengthSq2 = mover2.scale * mover2.scale;
            var distanceSq = mover1.pos.sub(mover2.pos).lengthSq();
            var isColliding = distanceSq <= lengthSq1 + lengthSq2;

            // if (i == 0 and j == 1) {
            // std.debug.warn("{} {} {} {}\n", lengthSq1, lengthSq2, distanceSq, isColliding);
            // }

            collidingState.set(i, collidingState.at(i) or isColliding);
            collidingState.set(j, collidingState.at(j) or isColliding);

            // std.debug.warn("({}, {}) {} {}\n", i, j, collidingState.at(i), collidingState.at(j));
        }
    }

    // std.debug.warn("{} {}\n", collidingState.at(0), collidingState.at(1));

    const WINDOWSIZE = get_window_size(state.window);
    for (state.movers.toSlice()) |*mover, index| {
        mover.pos = mover.pos.add(mover.velocity);
        mover.pos.x = @mod(mover.pos.x + WINDOWSIZE.width, WINDOWSIZE.width);
        mover.pos.y = @mod(mover.pos.y + WINDOWSIZE.height, WINDOWSIZE.height);

        const objectType = switch (index) {
            0 => MoverType.PlayerShip,
            else => MoverType.Asteroid,
        };
        draw_object(state.renderer, mover, objectType, collidingState.at(index));
        // draw_object(state.renderer, mover, objectType, false);
    }

    sdl.SDL_RenderPresent(state.renderer);
}

fn vector3_to_sdl(v: math.Vector3) sdl.SDL_Point {
    return sdl.SDL_Point{
        .x = @floatToInt(c_int, v.x),
        .y = @floatToInt(c_int, v.y),
    };
}

const SHIP_POINTS = [_]math.Vector3{
    math.Vector3{ .x = 0.0, .y = 0.5, .z = 0.0 },
    math.Vector3{ .x = 0.5, .y = -0.35, .z = 0.0 },
    math.Vector3{ .x = 0.0, .y = 0.0, .z = 0.0 },
    math.Vector3{ .x = -0.5, .y = -0.35, .z = 0.0 },
    math.Vector3{ .x = 0.0, .y = 0.5, .z = 0.0 },
};

const ASTEROID_POINTS = [_]math.Vector3{
    math.Vector3{ .x = 0.0, .y = 0.8, .z = 0.0 },
    math.Vector3{ .x = 0.8, .y = 0.4, .z = 0.0 },
    math.Vector3{ .x = 0.6, .y = -0.2, .z = 0.0 },
    math.Vector3{ .x = 0.2, .y = -0.6, .z = 0.0 },
    math.Vector3{ .x = -0.2, .y = -0.4, .z = 0.0 },
    math.Vector3{ .x = -0.4, .y = -0.1, .z = 0.0 },
    math.Vector3{ .x = -0.6, .y = 0.2, .z = 0.0 },
    math.Vector3{ .x = -0.4, .y = 0.6, .z = 0.0 },
    math.Vector3{ .x = 0.0, .y = 0.8, .z = 0.0 },
};

const CIRCLE_POINTS = generate_circle();

fn generate_circle() [32]math.Vector3 {
    var points: [32]math.Vector3 = undefined;
    for (points) |*point, i| {
        var rads = std.math.pi * 2.0 * (@intToFloat(f32, i) / @intToFloat(f32, points.len - 1));
        const x: f32 = std.math.cos(rads);
        const y: f32 = std.math.sin(rads);
        point.* = math.Vector3{ .x = x, .y = y, .z = 0 };
    }
    return points;
}

fn draw_bounding_circle(renderer: *sdl.SDL_Renderer, mover: *const Mover, isColliding: bool) void {
    const points = CIRCLE_POINTS;
    var sdlPoints = std.ArrayList(sdl.SDL_Point).init(std.heap.c_allocator);
    sdlPoints.resize(points.len) catch |err| std.debug.warn("Failed to resize sdlPoints: {}", err);

    for (points) |point, i| {
        var p = point.scale(mover.scale);
        p = p.rotateZ(mover.rot);
        p = p.add(mover.pos);

        sdlPoints.set(i, vector3_to_sdl(p));
    }

    const cPoints: [*c]sdl.SDL_Point = &sdlPoints.items[0];

    var r: u8 = 128;
    var g: u8 = 128;
    var b: u8 = 128;
    if (isColliding) {
        g = 0;
        b = 0;
    }

    if (sdl.SDL_SetRenderDrawColor(renderer, r, g, b, 255) < 0) {
        log_sdl_error();
    }
    if (sdl.SDL_RenderDrawLines(renderer, cPoints, @intCast(c_int, points.len)) < 0) {
        log_sdl_error();
    }
}

fn draw_object(renderer: *sdl.SDL_Renderer, mover: *const Mover, objectType: MoverType, isColliding: bool) void {
    const points = switch (objectType) {
        MoverType.PlayerShip => SHIP_POINTS,
        MoverType.Asteroid => ASTEROID_POINTS,
    };

    var sdlPoints = std.ArrayList(sdl.SDL_Point).init(std.heap.c_allocator);
    sdlPoints.resize(points.len) catch |err| std.debug.warn("Failed to resize sdlPoints: {}", err);

    for (points) |point, i| {
        var p = point.scale(mover.scale);
        p = p.rotateZ(mover.rot);
        p = p.add(mover.pos);

        sdlPoints.set(i, vector3_to_sdl(p));
    }

    var cPoints: [*c]sdl.SDL_Point = &sdlPoints.items[0];

    // const sdlPoints: [*]sdl.SDL_Point = &points;

    if (sdl.SDL_SetRenderDrawColor(renderer, 128, 128, 128, 255) < 0) {
        log_sdl_error();
    }
    if (sdl.SDL_RenderDrawLines(renderer, cPoints, @intCast(c_int, points.len)) < 0) {
        log_sdl_error();
    }

    draw_bounding_circle(renderer, mover, isColliding);
}
