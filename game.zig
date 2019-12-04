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
    shoot: bool = false,
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
    world_bounds: Vector2,
};

const Asteroid = struct {
    scale_index: u8 = 0, // indexes into ASTEROID_SCALES
};

const PositionList = std.ArrayList(Vector3);
const MoverList = std.ArrayList(Mover);
const MoverTypeList = std.ArrayList(MoverType);
const AsteroidList = std.ArrayList(Asteroid);
const IndexList = std.ArrayList(usize);

const GameState = struct {
    renderer: *sdl.SDL_Renderer,
    window: *sdl.SDL_Window,
    quit: bool,
    previous_time: Time,
    rng: std.rand.DefaultPrng,

    camera: Camera,
    world_bounds: Vector2,

    stars: PositionList,
    input: InputState,

    movers: MoverList,
    mover_types: MoverTypeList,
    asteroids: AsteroidList,

    delete_list: IndexList,

    debug_camera: bool,
};

const MoverType = enum {
    PlayerShip,
    Asteroid,
    Bullet,
};

const COLLISION_MASKS = [_]u8{
    0b010,
    0b111,
    0b010,
};

comptime {
    std.debug.assert(@memberCount(MoverType) == COLLISION_MASKS.len);
}

const ASTEROID_SCALES = [_]f32{ 15.0, 20.0, 45.0, 100.0 };

fn get_window_size(window: *sdl.SDL_Window) Vector2 {
    var width: c_int = 0;
    var height: c_int = 0;
    sdl.SDL_GetWindowSize(window, &width, &height);

    return Vector2{
        .x = @intToFloat(f32, width),
        .y = @intToFloat(f32, height),
    };
}

pub fn main() !u8 {
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
    // const world_bounds = get_window_size(window);
    // const world_bounds = Vector2{ .x = 400, .y = 300 };

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
        .mover_types = MoverTypeList.init(std.heap.c_allocator),
        .asteroids = AsteroidList.init(std.heap.c_allocator),

        .delete_list = IndexList.init(std.heap.c_allocator),

        .debug_camera = false,
    };

    // stars
    try gamestate.stars.resize(2000);
    for (gamestate.stars.toSlice()) |*pos| {
        const x = gamestate.rng.random.float(f32) * gamestate.world_bounds.x;
        const y = gamestate.rng.random.float(f32) * gamestate.world_bounds.y;
        pos.x = x;
        pos.y = y;
    }

    // player
    const min_capacity = 256;
    try gamestate.movers.ensureCapacity(min_capacity);
    try gamestate.mover_types.ensureCapacity(min_capacity);
    try gamestate.asteroids.ensureCapacity(min_capacity);

    {
        const mover = Mover{
            .pos = Vector3{ .x = 200, .y = 200 },
            .scale = 20,
        };
        try gamestate.movers.append(mover);
        try gamestate.mover_types.append(MoverType.PlayerShip);
        try gamestate.asteroids.append(Asteroid{});
    }

    // asteroids
    {
        var rng = &gamestate.rng.random;

        var countAsteroids: i32 = 0;
        while (countAsteroids < 12) {
            countAsteroids = countAsteroids + 1;

            const asteroid = Asteroid{
                .scale_index = ASTEROID_SCALES.len - 1,
            };

            const x = rng.float(f32) * gamestate.world_bounds.x;
            const y = rng.float(f32) * gamestate.world_bounds.y;
            const scale = ASTEROID_SCALES[asteroid.scale_index];
            const speed = 0.005 + rng.float(f32) * 0.035;
            const xdir = rng.float(f32) * 2.0 - 1.0;
            const ydir = rng.float(f32) * 2.0 - 1.0;

            const mover = Mover{
                .pos = Vector3{ .x = x, .y = y, .z = 0 },
                .scale = scale,
                .velocity = (Vector3{ .x = xdir, .y = ydir }).normalize().scale(speed),
            };

            try gamestate.movers.append(mover);
            try gamestate.mover_types.append(MoverType.Asteroid);
            try gamestate.asteroids.append(asteroid);
        }
    }

    // game loop
    while (!gamestate.quit) {
        const time: Time = get_time(gamestate.previous_time);
        try game_tick(&gamestate, time);
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
                }

                if (event.type == sdl.SDL_KEYUP) {
                    if (event.key.keysym.sym == sdl.SDLK_SPACE) {
                        gamestate.input.shoot = true;
                    } else if (event.key.keysym.sym == sdl.SDLK_BACKQUOTE) {
                        gamestate.debug_camera = !gamestate.debug_camera;
                    }
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

fn addMover(state: *GameState, mover_type: MoverType, mover: Mover) !void {
    try state.movers.append(mover);
    try state.mover_types.append(mover_type);
}

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

fn game_tick(state: *GameState, time: Time) !void {
    var draw_info = DrawInfo{
        .renderer = state.renderer,
        .camera = state.camera,
        .viewport_offset = get_window_size(state.window).toVector3().scale(0.5),
        .world_bounds = state.world_bounds,
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
        var playerMover = &state.movers.toSlice()[playerIndex];

        var rot: f32 = 0.0;
        rot += if (state.input.left) @as(f32, 5.0) else 0.0;
        rot -= if (state.input.right) @as(f32, 5.0) else 0.0;

        const period = std.math.pi * 2.0;
        playerMover.rot = @mod(playerMover.rot + rot * time.dt + period, period);

        const forward = Vector3{ .x = 0, .y = 1, .z = 0 };

        if (state.input.forward and !state.input.left and !state.input.right) {
            const MAX_ACCELERATION = 0.5;
            const MAX_VELOCITY_MAGNITUDE = 0.7;
            const MAX_VELOCITY_MAGNITUDE_SQ = MAX_VELOCITY_MAGNITUDE * MAX_VELOCITY_MAGNITUDE;

            const playerForward = forward.rotateZ(playerMover.rot).normalize().scale(MAX_ACCELERATION * time.dt);
            playerMover.velocity = playerMover.velocity.add(playerForward);
            if (playerMover.velocity.lengthSq() > MAX_VELOCITY_MAGNITUDE_SQ) {
                playerMover.velocity = playerMover.velocity.normalize().scale(MAX_VELOCITY_MAGNITUDE);
            }
        }

        if (state.input.shoot) {
            state.input.shoot = false;
            const BULLET_SPEED = 0.7;

            const player_speed = playerMover.velocity.length();
            const player_dir = forward.rotateZ(playerMover.rot).normalize();

            const pos = playerMover.pos.add(player_dir.scale(playerMover.scale));
            const vel = playerMover.velocity.add(player_dir.scale(BULLET_SPEED));

            const mover = Mover{
                .pos = pos,
                .velocity = vel,
                .scale = 2,
            };

            try state.movers.append(mover);
            try state.mover_types.append(MoverType.Bullet);
            try state.asteroids.append(Asteroid{});
        }
    }

    // std.debug.warn("camera pos: ({d:.2}, {d:.2})\n", state.camera.pos.x, state.camera.pos.y);
    // std.debug.warn("player pos: ({d:.2}, {d:.2})\n", state.movers.at(0).pos.x, state.movers.at(0).pos.y);

    var collidingState = std.ArrayList(bool).init(std.heap.c_allocator);
    try collidingState.resize(state.movers.count());
    for (state.movers.toSlice()) |*mover1, i| {
        for (state.movers.toSlice()) |mover2, j| {
            if (i == j) {
                continue;
            }

            // const MyEnum = enum {
            //     V1,
            //     V2,
            //     V3,
            // };
            // var e = MyEnum.V2;
            // var v = 1 << @intCast(u32, @enumToInt(e));

            const type1: u3 = @enumToInt(state.mover_types.at(i));
            const type2: u3 = @enumToInt(state.mover_types.at(j));

            const collision_mask1 = COLLISION_MASKS[type1];
            const collision_mask2 = COLLISION_MASKS[type2];

            const collision_bit1: u32 = @shlExact(@as(u8, 1), type1);
            const collision_bit2: u32 = @shlExact(@as(u8, 1), type2);

            var collisionLength = mover1.scale + mover2.scale;
            var distanceSq = mover1.pos.sub(mover2.pos).lengthSq();
            var isColliding = distanceSq <= collisionLength * collisionLength;

            if (collision_mask1 & collision_bit2 != 0) {
                collidingState.set(i, collidingState.at(i) or isColliding);
            }

            if (collision_mask2 & collision_bit1 != 0) {
                collidingState.set(j, collidingState.at(j) or isColliding);
            }

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

        const mover_type = state.mover_types.at(index);
        const is_colliding = collidingState.at(index);
        draw_object(draw_info, mover, mover_type, is_colliding);

        if (is_colliding and mover_type == MoverType.Bullet) {
            try state.delete_list.append(index);
        }
    }

    sdl.SDL_RenderPresent(state.renderer);

    // delayed delete in reverse index order
    std.sort.sort(usize, state.delete_list.toSlice(), delete_list_sorter);
    // std.sort.sort(usize, state.delete_list.toSlice(), fn (a: usize, b: usize) bool {
    //     return a > b;
    // });

    for (state.delete_list.toSlice()) |index| {
        _ = state.movers.swapRemove(index);
        _ = state.mover_types.swapRemove(index);
        _ = state.asteroids.swapRemove(index);
    }
    try state.delete_list.resize(0);
}

fn delete_list_sorter(a: usize, b: usize) bool {
    return a > b;
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

const BULLET_POINTS = [_]Vector3{
    Vector3{ .x = 0.5, .y = -0.5 },
    Vector3{ .x = 0.5, .y = 0.5 },
    Vector3{ .x = -0.5, .y = 0.5 },
    Vector3{ .x = -0.5, .y = -0.5 },
    Vector3{ .x = 0.5, .y = -0.5 },
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
        MoverType.Bullet => BULLET_POINTS,
    };

    var sdlPoints = transform_points_with_mover(draw_info, points, mover);

    draw_line_strip(draw_info, COLOR_GRAY, sdlPoints.toSliceConst());

    draw_bounding_circle(draw_info, mover, isColliding);

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

    //     sdlPoints = transform_points_with_mover(draw_info, points, &mirror_mover);
    //     draw_line_strip(draw_info, COLOR_BLUE, sdlPoints.toSliceConst());
    //     draw_bounding_circle(draw_info, &mirror_mover, isColliding);
    // }
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
