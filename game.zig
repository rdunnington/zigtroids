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
    total_elapsed: f64,
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

const None = struct {};

const FighterStateMove = struct {
    // shoot_cooldown_secs: f32 = 0.0,
};

const FighterStateShoot = struct {
    last_shoot_time: f64 = 0.0,
};

const Fighter = union(enum) {
    None: None,
    Moving: FighterStateMove,
    Shooting: FighterStateShoot,
};

const EnemySpawner = struct {
    last_spawn_time: f64 = 0.0,
};

const PositionList = std.ArrayList(Vector3);
const MoverList = std.ArrayList(Mover);
const MoverTypeList = std.ArrayList(MoverType);
const AsteroidList = std.ArrayList(Asteroid);
const FighterList = std.ArrayList(Fighter);
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

    mover_types: MoverTypeList,
    movers: MoverList,
    asteroids: AsteroidList,
    fighters: FighterList,

    enemy_spawner: EnemySpawner,

    delete_list: IndexList,

    debug_camera: bool,
};

const MoverType = enum {
    PlayerShip,
    Asteroid,
    Bullet,
    Fighter,
};

const COLLISION_MASKS = [_]u8{
    0b0010,
    0b0100,
    0b1111,
    0b0100,
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
        .previous_time = get_time(Time{
            .total_elapsed = 0,
            .dt = 0,
        }),
        .rng = std.rand.DefaultPrng.init(0),
        .camera = Camera{},
        .world_bounds = world_bounds,
        .input = InputState{},
        .stars = PositionList.init(std.heap.c_allocator),
        .mover_types = MoverTypeList.init(std.heap.c_allocator),
        .movers = MoverList.init(std.heap.c_allocator),
        .asteroids = AsteroidList.init(std.heap.c_allocator),
        .fighters = FighterList.init(std.heap.c_allocator),

        .enemy_spawner = EnemySpawner{
            .last_spawn_time = 0.0,
        },

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
    try gamestate.mover_types.ensureCapacity(min_capacity);
    try gamestate.movers.ensureCapacity(min_capacity);
    try gamestate.asteroids.ensureCapacity(min_capacity);
    try gamestate.fighters.ensureCapacity(min_capacity);

    {
        const mover = Mover{
            .pos = Vector3{ .x = 200, .y = 200 },
            .scale = 10,
        };

        try add_object(&gamestate, MoverType.PlayerShip, &mover, null, null);
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

            try add_object(&gamestate, MoverType.Asteroid, &mover, &asteroid, null);
        }
    }

    // game loop
    while (!gamestate.quit) {
        const time: Time = get_time(gamestate.previous_time);

        // std.debug.warn("elapsed: {d:.4}, dt: {d:.4}\n", time.total_elapsed, time.dt);

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

fn add_object(state: *GameState, mover_type: MoverType, mover: *const Mover, optional_asteroid: ?*const Asteroid, optional_fighter: ?*const Fighter) !void {
    const default_asteroid = Asteroid{};
    const default_fighter = Fighter{ .None = None{} };

    const safe_asteroid = if (optional_asteroid) |asteroid| asteroid else &default_asteroid;
    const safe_fighter = if (optional_fighter) |fighter| fighter else &default_fighter;

    try state.mover_types.append(mover_type);
    try state.movers.append(mover.*);
    try state.asteroids.append(safe_asteroid.*);
    try state.fighters.append(safe_fighter.*);

    std.debug.assert(state.fighters.count() == state.movers.count());
}

fn shoot_bullet(state: *GameState, from_mover: *const Mover, direction: Vector3) !void {
    const BULLET_SPEED = 0.7;
    const BULLET_SIZE = 2.0;

    const mover_speed = from_mover.velocity.length();
    // const player_dir = forward.rotateZ(from_mover.rot).normalize();

    const pos = from_mover.pos.add(direction.scale(from_mover.scale + BULLET_SIZE + std.math.f32_epsilon));
    const vel = from_mover.velocity.add(direction.scale(BULLET_SPEED));

    const mover = Mover{
        .pos = pos,
        .velocity = vel,
        .scale = BULLET_SIZE,
    };

    try add_object(state, MoverType.Bullet, &mover, null, null);
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
        .total_elapsed = current,
        .dt = @floatCast(f32, current - prev.total_elapsed),
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

    // draw debug grid lines
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

    const forward = Vector3{ .x = 1, .y = 0, .z = 0 };

    const playerIndex = 0;
    var player_mover = &state.movers.toSlice()[playerIndex];

    // debug camera or player steering
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
        var rot: f32 = 0.0;
        rot += if (state.input.left) @as(f32, 5.0) else 0.0;
        rot -= if (state.input.right) @as(f32, 5.0) else 0.0;

        const period = std.math.pi * 2.0;
        player_mover.rot = @mod(player_mover.rot + rot * time.dt + period, period);

        if (state.input.forward and !state.input.left and !state.input.right) {
            const MAX_ACCELERATION = 0.5 * time.dt;
            const MAX_VELOCITY_MAGNITUDE = 0.7;

            player_mover.velocity = calc_velocity(player_mover.rot, player_mover.velocity, MAX_ACCELERATION, MAX_VELOCITY_MAGNITUDE);

            // const playerForward = forward.rotateZ(player_mover.rot).normalize().scale(MAX_ACCELERATION * time.dt);
            // player_mover.velocity = player_mover.velocity.add(playerForward);
            // if (player_mover.velocity.lengthSq() > MAX_VELOCITY_MAGNITUDE_SQ) {
            //     player_mover.velocity = player_mover.velocity.normalize().scale(MAX_VELOCITY_MAGNITUDE);
            // }
        }

        if (state.input.shoot) {
            state.input.shoot = false;

            const player_dir = forward.rotateZ(player_mover.rot).normalize();
            try shoot_bullet(state, player_mover, player_dir);
            // const BULLET_SPEED = 0.7;

            // const player_speed = player_mover.velocity.length();
            // const player_dir = forward.rotateZ(player_mover.rot).normalize();

            // const pos = player_mover.pos.add(player_dir.scale(player_mover.scale));
            // const vel = player_mover.velocity.add(player_dir.scale(BULLET_SPEED));

            // const mover = Mover{
            //     .pos = pos,
            //     .velocity = vel,
            //     .scale = 2,
            // };

            // try add_object(state, MoverType.Bullet, &mover, null, null);
        }
    }

    // Enemies
    {
        // spawn new enemies
        const ENEMY_SPAWN_COOLDOWN = 10.0;

        if (time.total_elapsed - state.enemy_spawner.last_spawn_time >= ENEMY_SPAWN_COOLDOWN) {
            state.enemy_spawner.last_spawn_time = time.total_elapsed;

            var x: f32 = 100.0;
            var y: f32 = 100.0;

            const mover = Mover{
                .pos = Vector3{ .x = x, .y = y },
                .scale = 10,
            };
            const fighter = Fighter{
                .Shooting = FighterStateShoot{},
            };

            try add_object(state, MoverType.Fighter, &mover, null, &fighter);
        }

        // AI tick for existing enemies
        for (state.fighters.toSlice()) |*fighter, index| {
            const MAX_TURN_SPEED = 5.0 * time.dt;
            const MAX_SHOOT_RANGE = 200.0;
            const MAX_ACCELERATION = 0.12 * time.dt;
            const MAX_SPEED = 0.3;

            var mover = &state.movers.toSlice()[index];
            const to_player_mover = player_mover.pos.sub(mover.pos);
            const to_player_length = to_player_mover.length();
            const to_player_mover_dir = to_player_mover.scale(1.0 / to_player_length);

            switch (fighter.*) {
                Fighter.Moving => |*move_state| {
                    // turn towards the player and accelerate toward them
                    const goal_rot = calc_goal_rot(to_player_mover_dir);
                    const rot_delta = calc_rot_delta(goal_rot, mover.rot, MAX_TURN_SPEED);
                    mover.rot += rot_delta;

                    mover.velocity = calc_velocity(mover.rot, mover.velocity, MAX_ACCELERATION, MAX_SPEED);

                    // get a bit within max shooting range so we don't immediately switch out
                    if (to_player_length < MAX_SHOOT_RANGE * 0.9) {
                        fighter.* = Fighter{
                            .Shooting = FighterStateShoot{
                                .last_shoot_time = 0,
                            },
                        };
                    }

                    // debug draw target
                    // {
                    //     const transformed1 = [_]sdl.SDL_Point{
                    //         vector3_to_sdl(transform_point_to_screen_space(mover.pos, &draw_info)),
                    //         vector3_to_sdl(transform_point_to_screen_space(mover.pos.add(to_player_mover), &draw_info)),
                    //     };

                    //     draw_line_strip(draw_info, COLOR_WHITE, transformed1);
                    // }
                },
                Fighter.Shooting => |*shoot_state| {
                    // turn towards player
                    const goal_rot = calc_goal_rot(to_player_mover_dir);
                    const rot_delta = calc_rot_delta(goal_rot, mover.rot, MAX_TURN_SPEED);
                    mover.rot += rot_delta;

                    const SHOOT_COOLDOWN = 1.0;
                    const is_aiming_at_player = std.math.fabs(rot_delta) <= MAX_TURN_SPEED;
                    const is_cooldown_finished = SHOOT_COOLDOWN <= time.total_elapsed - shoot_state.last_shoot_time;

                    if (is_cooldown_finished and is_aiming_at_player) {
                        shoot_state.last_shoot_time = time.total_elapsed;
                        const shoot_direction = forward.rotateZ(mover.rot);
                        try shoot_bullet(state, mover, shoot_direction);
                    }

                    if (to_player_length > MAX_SHOOT_RANGE) {
                        fighter.* = Fighter{
                            .Moving = FighterStateMove{},
                        };
                    }

                    // debug draw target
                    // {
                    //     const target1 = to_player_mover;
                    //     const transformed1 = [_]sdl.SDL_Point{
                    //         vector3_to_sdl(transform_point_to_screen_space(mover.pos, &draw_info)),
                    //         vector3_to_sdl(transform_point_to_screen_space(mover.pos.add(target1), &draw_info)),
                    //     };

                    //     const target2 = forward.rotateZ(mover.rot).scale(200);
                    //     const transformed2 = [_]sdl.SDL_Point{
                    //         vector3_to_sdl(transform_point_to_screen_space(mover.pos, &draw_info)),
                    //         vector3_to_sdl(transform_point_to_screen_space(mover.pos.add(target2), &draw_info)),
                    //     };

                    //     // const transformed = transform_points_with_mover(draw_info, points, mover);
                    //     // draw_line_strip(draw_info, COLOR_WHITE, transformed.toSliceConst());
                    //     draw_line_strip(draw_info, COLOR_WHITE, transformed1);
                    //     draw_line_strip(draw_info, COLOR_GREEN, transformed2);
                    // }
                },
                else => {},
            }
        }
    }

    // std.debug.warn("camera pos: ({d:.2}, {d:.2})\n", state.camera.pos.x, state.camera.pos.y);
    // std.debug.warn("player pos: ({d:.2}, {d:.2})\n", state.movers.at(0).pos.x, state.movers.at(0).pos.y);

    if (!state.debug_camera) {
        for (state.mover_types.toSliceConst()) |mover_type, index| {
            if (mover_type == MoverType.PlayerShip) {
                state.camera.pos = state.movers.at(index).pos;
                break;
            }
        }
    }

    // std.debug.warn("{} {}\n", colliding_state.at(0), colliding_state.at(1));

    var colliding_state = std.ArrayList(bool).init(std.heap.c_allocator);
    try colliding_state.resize(state.movers.count());

    for (state.movers.toSlice()) |*mover1, i| {
        for (state.movers.toSlice()) |mover2, j| {
            if (i == j) {
                continue;
            }

            const type1: u3 = @enumToInt(state.mover_types.at(i));
            const type2: u3 = @enumToInt(state.mover_types.at(j));

            const collision_mask1 = COLLISION_MASKS[type1];
            const collision_mask2 = COLLISION_MASKS[type2];

            const collision_bit1: u32 = @shlExact(@as(u8, 1), type1);
            const collision_bit2: u32 = @shlExact(@as(u8, 1), type2);

            var collisionLength = mover1.scale * 0.95 + mover2.scale * 0.95;
            var distanceSq = mover1.pos.sub(mover2.pos).lengthSq();
            var is_colliding = distanceSq <= collisionLength * collisionLength;

            if (collision_mask1 & collision_bit2 != 0) {
                colliding_state.set(i, colliding_state.at(i) or is_colliding);
            }

            if (collision_mask2 & collision_bit1 != 0) {
                colliding_state.set(j, colliding_state.at(j) or is_colliding);
            }

            // if (is_colliding and i == 0) {
            //     var length = mover1.velocity.length();
            //     var bounceDir = mover1.velocity.normalize();
            //     var bounceVel = bounceDir.scale(-length * 0.8);
            //     std.debug.warn("{} {}\n", bounceVel.x, bounceVel.y);
            //     mover1.velocity = bounceVel;
            //     mover1.pos = mover1.pos.add(bounceDir.scale(-std.math.sqrt(distanceSq)));
            // }
        }
    }

    for (state.movers.toSlice()) |*mover, index| {
        mover.pos = mover.pos.add(mover.velocity);
        mover.pos.x = @mod(mover.pos.x + state.world_bounds.x, state.world_bounds.x);
        mover.pos.y = @mod(mover.pos.y + state.world_bounds.y, state.world_bounds.y);

        const mover_type = state.mover_types.at(index);
        const is_colliding = colliding_state.at(index);
        draw_object(draw_info, mover, mover_type, is_colliding);

        if (is_colliding and mover_type == MoverType.Bullet) {
            try state.delete_list.append(index);
        }

        if (is_colliding and mover_type == MoverType.Asteroid) {
            var asteroid: *Asteroid = &state.asteroids.toSlice()[index];

            // scale this guy down
            if (asteroid.scale_index == 0) {
                try state.delete_list.append(index);
            } else {
                const old_speed = mover.velocity.length();

                // 10-25% speed increase
                const new_speed1 = old_speed * (1.1 + state.rng.random.float(f32) * 0.15);
                const new_speed2 = old_speed * (1.1 + state.rng.random.float(f32) * 0.15);

                const new_dir1 = state.rng.random.float(f32) * std.math.pi * 2.0;
                const new_dir2 = state.rng.random.float(f32) * std.math.pi * 2.0;

                const vel1 = forward.rotateZ(new_dir1).scale(new_speed1);
                const vel2 = forward.rotateZ(new_dir2).scale(new_speed2);

                asteroid.scale_index = asteroid.scale_index - 1;

                mover.scale = ASTEROID_SCALES[asteroid.scale_index];
                mover.velocity = vel1;

                var mover_clone: Mover = mover.*;
                mover_clone.velocity = vel2;

                // offset positions so they look like individual asteroids
                mover.pos = mover.pos.add(mover.velocity.scale(100.0));
                mover_clone.pos = mover_clone.pos.add(mover_clone.velocity.scale(100.0));

                try add_object(state, MoverType.Asteroid, &mover_clone, asteroid, null);
            }
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
        _ = state.fighters.swapRemove(index);
    }
    try state.delete_list.resize(0);
}

fn calc_goal_rot(desired_forward: Vector3) f32 {
    const sign = std.math.copysign(f32, 1.0, -Vector3.right.cross(desired_forward).z);
    const goal_rot = std.math.acos(Vector3.right.dot(desired_forward)) * sign + std.math.pi * 2.0;
    return goal_rot;
}

fn calc_rot_delta(goal_rot: f32, current_rot: f32, max_rot_delta: f32) f32 {
    const full_rot = goal_rot - current_rot;
    const rot_sign = std.math.copysign(f32, 1.0, full_rot);
    const max_rot = rot_sign * max_rot_delta;
    const rot_delta = std.math.min(std.math.fabs(full_rot), std.math.fabs(max_rot));
    return rot_delta * rot_sign;
}

fn calc_velocity(rot: f32, old_velocity: Vector3, max_acceleration: f32, max_velocity_length: f32) Vector3 {
    const max_velocity_length_sq = max_velocity_length * max_velocity_length;
    const mover_forward = Vector3.right.rotateZ(rot).normalize().scale(max_acceleration);
    var new_velocity = old_velocity.add(mover_forward);
    if (new_velocity.lengthSq() > max_velocity_length_sq) {
        new_velocity = new_velocity.normalize().scale(max_velocity_length);
    }
    return new_velocity;
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
    Vector3{ .x = 1, .y = 0.0, .z = 0.0 },
    Vector3{ .x = -0.7, .y = -1, .z = 0.0 },
    Vector3{ .x = 0.0, .y = 0.0, .z = 0.0 },
    Vector3{ .x = -0.7, .y = 1, .z = 0.0 },
    Vector3{ .x = 1, .y = 0.0, .z = 0.0 },
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

fn draw_bounding_circle(draw_info: DrawInfo, mover: *const Mover, is_colliding: bool) void {
    const color = if (is_colliding) COLOR_RED else COLOR_GRAY;
    const sdlPoints = transform_points_with_mover(draw_info, CIRCLE_POINTS, mover);

    draw_line_strip(draw_info, color, sdlPoints.toSliceConst());
}

fn draw_object(draw_info: DrawInfo, mover: *const Mover, object_type: MoverType, is_colliding: bool) void {
    const points = switch (object_type) {
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

    var sdlPoints = transform_points_with_mover(draw_info, points, mover);

    draw_line_strip(draw_info, color, sdlPoints.toSliceConst());

    draw_bounding_circle(draw_info, mover, is_colliding);

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
    //     draw_bounding_circle(draw_info, &mirror_mover, is_colliding);
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
