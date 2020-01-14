const std = @import("std");
const math = @import("math.zig");
const slotmap = @import("lib/zig-slotmap/slotmap.zig");

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
    total_elapsed: f64 = 0.0,
    dt: f32 = 0.0,
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

const EntityId = struct {
    id: u32,
};

const NewEntityDesc = struct {
    id: EntityId = ENTITY_ID_INVALID,
    mover_type: MoverType,
    mover: Mover,
    asteroid: ?Asteroid = null,
    fighter: ?Fighter = null,
};

fn MakeSystem(comptime ComponentType: type) type {
    return struct {
        const Self = @This();
        const EC = struct {
            component: ComponentType,
            entity: EntityId,
        };
        const ECSlotmap = slotmap.Dense(EC);
        const EntityMapType = std.AutoHashMap(EntityId, ECSlotmap.Slot);

        components: ECSlotmap,
        entity_lookup: EntityMapType,

        pub fn init(allocator: *std.mem.Allocator) Self {
            return Self{
                .components = ECSlotmap.init(allocator),
                .entity_lookup = EntityMapType.init(allocator),
            };
        }

        pub fn deinit(system: *Self) void {
            system.components.deinit();
            system.entity_lookup.deinit();
        }

        pub fn add(system: *Self, component: ComponentType, entity: EntityId) !void {
            const ec = EC{
                .component = component,
                .entity = entity,
            };
            var slot = try system.components.insert(ec);
            try system.entity_lookup.putNoClobber(entity, slot);
        }

        pub fn remove(system: *Self, entity: EntityId) !void {
            var removed = system.entity_lookup.remove(entity) orelse return error.NOT_FOUND;
            _ = try system.components.remove(removed.value);
        }

        pub fn removeSafe(system: *Self, entity: EntityId) void {
            system.remove(entity) catch {};
        }

        pub fn get(system: *Self, entity: EntityId) !*ComponentType {
            var index = system.entity_lookup.getValue(entity) orelse return error.NOT_FOUND;
            var ec = try system.components.getPtr(index);
            return &ec.component;
        }

        pub fn toSlice(system: *Self) []EC {
            return system.components.toSlice();
        }

        pub fn toSliceConst(system: *Self) []const EC {
            return system.components.toSliceConst();
        }

        pub fn count(system: *Self) usize {
            return system.components.len;
        }
    };
}

const MoverSystem = MakeSystem(Mover);
const MoverTypeSystem = MakeSystem(MoverType);
const AsteroidSystem = MakeSystem(Asteroid);
const FighterSystem = MakeSystem(Fighter);

const EntityList = std.ArrayList(EntityId);
const PositionList = std.ArrayList(Vector3);
const NewEntityDescList = std.ArrayList(NewEntityDesc);

const ENTITY_ID_INVALID = EntityId{ .id = 0 };

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

    entities: EntityList,
    mover_types: MoverTypeSystem,
    movers: MoverSystem,
    asteroids: AsteroidSystem,
    fighters: FighterSystem,

    enemy_spawner: EnemySpawner,

    delete_list: EntityList,
    new_entities: NewEntityDescList,
    next_entity_id: u32 = 0,
    player_entity_id: EntityId = ENTITY_ID_INVALID,

    debug_camera: bool,
};

const MoverType = enum {
    PlayerShip,
    Asteroid,
    Bullet,
    Fighter,
};

// const CollisionType = enum {
//     PlayerShip,
//     Asteroid,
//     Bullet,
//     Fighter,
// };

// const CollisionComponent = struct {
//     type: CollisionType,
//     mask: u32 = 0,
// };

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
        std.debug.panic("sdl.SDL_Init failed: {c}\n", .{sdl.SDL_GetError()});
    }
    defer sdl.SDL_Quit();

    // TODO sdl.SDL_WINDOW_RESIZABLE - game exits with error code when we resize??
    var window: *sdl.SDL_Window = sdl.SDL_CreateWindow(
        "Zigtroids",
        200,
        200,
        @intCast(c_int, 640),
        @intCast(c_int, 480),
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

        .entities = EntityList.init(std.heap.c_allocator),
        .mover_types = MoverTypeSystem.init(std.heap.c_allocator),
        .movers = MoverSystem.init(std.heap.c_allocator),
        .asteroids = AsteroidSystem.init(std.heap.c_allocator),
        .fighters = FighterSystem.init(std.heap.c_allocator),

        .enemy_spawner = EnemySpawner{
            .last_spawn_time = 0.0,
        },

        .delete_list = EntityList.init(std.heap.c_allocator),
        .new_entities = NewEntityDescList.init(std.heap.c_allocator),

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
    {
        const mover = Mover{
            .pos = Vector3{ .x = 200, .y = 200 },
            .scale = 10,
        };

        const desc = NewEntityDesc{
            .mover = mover,
            .mover_type = MoverType.PlayerShip,
        };

        gamestate.player_entity_id = try new_entity_delayed(&gamestate, desc);
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

            const desc = NewEntityDesc{
                .mover = mover,
                .mover_type = MoverType.Asteroid,
                .asteroid = asteroid,
            };

            _ = try new_entity_delayed(&gamestate, desc);
        }
    }

    // make sure entities are there for the first frame of the loop
    try pump_entity_queues(&gamestate);

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

fn new_entity_delayed(state: *GameState, desc: NewEntityDesc) !EntityId {
    state.next_entity_id = state.next_entity_id + 1;
    const id = EntityId{
        .id = state.next_entity_id,
    };

    var desc_with_id = desc;
    desc_with_id.id = id;

    try state.new_entities.append(desc_with_id);

    return id;
}

fn pump_entity_queues(state: *GameState) !void {
    {
        const start_time = get_time(null);
        defer log_scope_time_on_overage("\tdelayed delete", start_time, 12.0);

        for (state.delete_list.toSlice()) |entity| {
            state.movers.removeSafe(entity);
            state.mover_types.removeSafe(entity);
            state.asteroids.removeSafe(entity);
            state.fighters.removeSafe(entity);
        }
        try state.delete_list.resize(0);
    }

    for (state.new_entities.toSliceConst()) |desc| {
        try state.entities.append(desc.id);
        try state.mover_types.add(desc.mover_type, desc.id);
        try state.movers.add(desc.mover, desc.id);

        std.debug.warn("[entity]\n\ttype: {}\n\tpos: {d:.2} {d:.2}\n\tvelocity: {d:.2} {d:.2}\n", .{
            @tagName(desc.mover_type),
            desc.mover.pos.x,
            desc.mover.pos.y,
            desc.mover.velocity.x,
            desc.mover.velocity.y,
        });

        if (desc.asteroid) |asteroid| try state.asteroids.add(asteroid, desc.id);
        if (desc.fighter) |fighter| try state.fighters.add(fighter, desc.id);
    }
    try state.new_entities.resize(0);
}

fn shoot_bullet(state: *GameState, from_mover: *const Mover, direction: Vector3) !void {
    const BULLET_SPEED = 0.7;
    const BULLET_SIZE = 2.0;

    const mover_speed = from_mover.velocity.length();
    const pos = from_mover.pos.add(direction.scale(from_mover.scale + BULLET_SIZE + std.math.f32_epsilon));
    const vel = from_mover.velocity.add(direction.scale(BULLET_SPEED));

    const mover = Mover{
        .pos = pos,
        .velocity = vel,
        .scale = BULLET_SIZE,
    };

    const desc = NewEntityDesc{
        .mover = mover,
        .mover_type = MoverType.Bullet,
    };

    _ = try new_entity_delayed(state, desc);
}

fn log_sdl_error() void {
    var c_err = sdl.SDL_GetError() orelse return;
    var err = std.mem.toSliceConst(u8, c_err);
    std.debug.warn("Error in SDL: {}\n", .{err});
}

fn get_time(optional_prev: ?Time) Time {
    const prev = if (optional_prev) |time| time else Time{};

    var counter: u64 = sdl.SDL_GetPerformanceCounter();
    var frequency: u64 = sdl.SDL_GetPerformanceFrequency();

    var current: f64 = @intToFloat(f64, counter) / @intToFloat(f64, frequency);

    return Time{
        .total_elapsed = current,
        .dt = @floatCast(f32, current - prev.total_elapsed),
    };
}

fn log_scope_time(name: []const u8, prev_time: Time) void {
    const now = get_time(prev_time);
    std.debug.warn("{}: {d:.2}ms\n", .{ name, now.dt * 1000.0 });
}

fn log_scope_time_on_overage(name: []const u8, prev_time: Time, overage_threshold_ms: f32) void {
    const now = get_time(prev_time);
    if (now.dt * 1000.0 > overage_threshold_ms) {
        std.debug.warn("{}: {d:.2}ms\n", .{ name, now.dt * 1000.0 });
    }
}

fn game_tick(state: *GameState, time: Time) !void {
    defer log_scope_time_on_overage("game_tick", time, 12.0);

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
    // if (false) {
    //     if (sdl.SDL_SetRenderDrawColor(state.renderer, 45, 45, 45, 255) < 0) {
    //         log_sdl_error();
    //     }

    //     const step: i32 = 10;
    //     var x: c_int = 0;
    //     var y: c_int = 0;
    //     while (x < state.world_bounds.x) {
    //         x += step;
    //         if (sdl.SDL_RenderDrawLine(state.renderer, x, 0, x, state.world_bounds.y) < 0) {
    //             log_sdl_error();
    //         }
    //     }

    //     while (y < state.world_bounds.y) {
    //         y += step;
    //         if (sdl.SDL_RenderDrawLine(state.renderer, 0, y, state.world_bounds.x, y) < 0) {
    //             log_sdl_error();
    //         }
    //     }
    // }

    {
        const start_time = get_time(time);
        defer log_scope_time_on_overage("\tdraw_stars", start_time, 12.0);
        draw_stars(draw_info, state.stars.toSliceConst());
    }

    // draw world border
    {
        const border_points = &[_]Vector3{
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
        points.deinit();
    }

    const forward = Vector3{ .x = 1, .y = 0, .z = 0 };

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
        var player_mover: *Mover = try state.movers.get(state.player_entity_id);
        player_mover.rot = @mod(player_mover.rot + rot * time.dt + period, period);

        if (state.input.forward and !state.input.left and !state.input.right) {
            const MAX_ACCELERATION = 0.5 * time.dt;
            const MAX_SPEED = 0.7;

            player_mover.velocity = calc_velocity(player_mover.rot, player_mover.velocity, MAX_ACCELERATION, MAX_SPEED);
        }

        if (state.input.shoot) {
            state.input.shoot = false;

            const player_dir = forward.rotateZ(player_mover.rot).normalize();
            try shoot_bullet(state, player_mover, player_dir);
        }
    }

    // Enemies
    {
        var player_mover: *Mover = try state.movers.get(state.player_entity_id);

        const start_time = get_time(time);
        defer log_scope_time_on_overage("\tenemy_logic", start_time, 12.0);

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

            const desc = NewEntityDesc{
                .mover = mover,
                .mover_type = MoverType.Fighter,
                .fighter = fighter,
            };

            _ = try new_entity_delayed(state, desc);
        }

        // AI tick for existing enemies
        for (state.fighters.toSlice()) |*ec| {
            var fighter = &ec.component;

            const MAX_TURN_SPEED = 5.0 * time.dt;
            const MAX_SHOOT_RANGE = 200.0;
            const MAX_ACCELERATION = 0.12 * time.dt;
            const MAX_SPEED = 0.3;

            var mover = try state.movers.get(ec.entity);

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

    if (!state.debug_camera) {
        var player_mover: *Mover = try state.movers.get(state.player_entity_id);
        state.camera.pos = player_mover.pos;
    }

    {
        var colliding_state = std.ArrayList(bool).init(std.heap.c_allocator);
        try colliding_state.resize(state.movers.count());

        {
            const start_time = get_time(time);
            defer log_scope_time_on_overage("\tcollision_detection", start_time, 12.0);
            for (state.movers.toSlice()) |*ec1, i| {
                for (state.movers.toSlice()) |*ec2, k| {
                    if (ec1.entity.id == ec2.entity.id) {
                        continue;
                    }

                    const type1: u3 = @enumToInt((try state.mover_types.get(ec1.entity)).*);
                    const type2: u3 = @enumToInt((try state.mover_types.get(ec2.entity)).*);

                    const mover1 = ec1.component;
                    const mover2 = ec2.component;

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
                        colliding_state.set(k, colliding_state.at(k) or is_colliding);
                    }
                }
            }
        }

        {
            const start_time = get_time(time);
            defer log_scope_time_on_overage("\tmover_update", start_time, 12.0);
            for (state.movers.toSlice()) |*ec, index| {
                var mover: *Mover = &ec.component;

                mover.pos = mover.pos.add(mover.velocity);
                mover.pos.x = @mod(mover.pos.x + state.world_bounds.x, state.world_bounds.x);
                mover.pos.y = @mod(mover.pos.y + state.world_bounds.y, state.world_bounds.y);

                const mover_type = (try state.mover_types.get(ec.entity)).*;
                const is_colliding = colliding_state.at(index);

                {
                    const start_time2 = get_time(time);
                    defer log_scope_time_on_overage("\t\tdraw_object", start_time2, 12.0);
                    draw_object(draw_info, mover, mover_type, is_colliding);
                }

                if (is_colliding and mover_type == MoverType.Bullet) {
                    try state.delete_list.append(ec.entity);
                }

                if (is_colliding and mover_type == MoverType.Asteroid) {
                    var asteroid: *Asteroid = try state.asteroids.get(ec.entity);

                    // scale this guy down
                    std.debug.warn("{}\n", .{asteroid.scale_index});
                    if (asteroid.scale_index == 0) {
                        try state.delete_list.append(ec.entity);
                    } else {
                        const old_speed = mover.velocity.length();

                        // 10-25% speed increase
                        const new_speed1 = old_speed * (1.1 + state.rng.random.float(f32) * 0.15);
                        const new_speed2 = old_speed * (1.1 + state.rng.random.float(f32) * 0.15);

                        const new_dir1 = state.rng.random.float(f32) * std.math.pi * 2.0;
                        const new_dir2 = state.rng.random.float(f32) * std.math.pi * 2.0;

                        const vel1 = forward.rotateZ(new_dir1).scale(new_speed1);
                        const vel2 = forward.rotateZ(new_dir2).scale(new_speed2);

                        std.debug.warn("\t{}\n", .{asteroid.scale_index});
                        asteroid.scale_index = asteroid.scale_index - 1;

                        mover.scale = ASTEROID_SCALES[asteroid.scale_index];
                        mover.velocity = vel1;

                        var mover_clone: Mover = mover.*;
                        mover_clone.velocity = vel2;

                        // offset positions so they look like individual asteroids
                        mover.pos = mover.pos.add(mover.velocity.scale(100.0));
                        mover_clone.pos = mover_clone.pos.add(mover_clone.velocity.scale(100.0));

                        const desc = NewEntityDesc{
                            .mover = mover_clone,
                            .mover_type = MoverType.Asteroid,
                            .asteroid = asteroid.*,
                        };

                        _ = try new_entity_delayed(state, desc);
                    }
                }
            }
        }
        colliding_state.deinit();
    }

    {
        const start_time = get_time(time);
        defer log_scope_time_on_overage("\tpresent", start_time, 12.0);
        sdl.SDL_RenderPresent(state.renderer);
    }

    // update these after all deletes and new entities have been submitted
    try pump_entity_queues(state);
}

// =======================================================
// gameplay helpers

fn delete_list_sorter(a: usize, b: usize) bool {
    return a > b;
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

// =======================================================
// drawing and geo

fn vector3_to_sdl(v: Vector3) sdl.SDL_Point {
    return sdl.SDL_Point{
        .x = @floatToInt(c_int, v.x),
        .y = @floatToInt(c_int, v.y),
    };
}

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

fn draw_bounding_circle(draw_info: DrawInfo, mover: *const Mover, is_colliding: bool) void {
    const color = if (is_colliding) COLOR_RED else COLOR_GRAY;
    const sdl_points = transform_points_with_mover(draw_info, CIRCLE_POINTS, mover);

    draw_line_strip(draw_info, color, sdl_points.toSliceConst());
    sdl_points.deinit();
}

fn draw_object(draw_info: DrawInfo, mover: *const Mover, object_type: MoverType, is_colliding: bool) void {
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

fn draw_stars(draw_info: DrawInfo, points: []const Vector3) void {
    var sdl_points = transform_points_with_positions(draw_info, points);
    draw_points(draw_info, COLOR_GRAY, sdl_points.toSliceConst());
    sdl_points.deinit();
}

fn transform_point_to_screen_space(point: Vector3, draw_info: *const DrawInfo) Vector3 {
    return point.sub(draw_info.camera.pos).add(draw_info.viewport_offset);
}

fn transform_points_with_positions(draw_info: DrawInfo, points: []const Vector3) SdlPointsList {
    var sdl_points = SdlPointsList.init(std.heap.c_allocator);
    sdl_points.resize(points.len) catch |err| std.debug.warn("Failed to resize sdl_points: {}", .{err});

    for (points) |point, i| {
        var p = transform_point_to_screen_space(point, &draw_info);
        sdl_points.set(i, vector3_to_sdl(p));
    }

    return sdl_points;
}

fn transform_points_with_mover(draw_info: DrawInfo, points: []const Vector3, mover: *const Mover) SdlPointsList {
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
