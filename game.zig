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
    // transforms: TransformList,
    input: InputState,
};

const MoverType = enum {
    PlayerShip,
    Asteroid,
};

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

    gamestate.movers.resize(4) catch |err| std.debug.warn("failed to alloc ships: {}", err);
    {
        var width: c_int = 0;
        var height: c_int = 0;
        sdl.SDL_GetWindowSize(gamestate.window, &width, &height);

        const MAX_WIDTH = @intToFloat(f32, width);
        const MAX_HEIGHT = @intToFloat(f32, height);

        var rng = &gamestate.rng.random;

        for (gamestate.movers.toSlice()) |*mover, index| {
            if (index == 0) {
                mover.pos = Vector3{ .x = 200, .y = 200, .z = 0 };
                mover.scale = 20;
            } else {
                const x = rng.float(f32) * MAX_WIDTH;
                const y = rng.float(f32) * MAX_HEIGHT;
                const scale = 4 + rng.float(f32) * 20.0;
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

    var width: c_int = 0;
    var height: c_int = 0;
    sdl.SDL_GetWindowSize(state.window, &width, &height);

    const MAX_WIDTH = @intToFloat(f32, width);
    const MAX_HEIGHT = @intToFloat(f32, height);

    for (state.movers.toSlice()) |*mover, index| {
        mover.pos = mover.pos.add(mover.velocity);
        mover.pos.x = @mod(mover.pos.x + MAX_WIDTH, MAX_WIDTH);
        mover.pos.y = @mod(mover.pos.y + MAX_HEIGHT, MAX_HEIGHT);

        const objectType = switch (index) {
            0 => MoverType.PlayerShip,
            else => MoverType.Asteroid,
        };
        draw_object(state.renderer, mover, objectType);
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

fn draw_object(renderer: *sdl.SDL_Renderer, mover: *const Mover, objectType: MoverType) void {
    const points = switch (objectType) {
        MoverType.PlayerShip => SHIP_POINTS,
        MoverType.Asteroid => ASTEROID_POINTS,
    };

    // comptime {
    //     std.debug.warn("points type: {}", @typeOf(points));
    // }

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
}
