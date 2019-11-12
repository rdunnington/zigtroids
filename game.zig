const std = @import("std");

const sdl = @cImport({
    @cInclude("SDL2/SDL.h");
});

const Time = struct {
    elapsed_from_start: f64,
    dt: f32,
};

const GameState = struct {
    renderer: *sdl.SDL_Renderer,
    quit: bool,
    previous_time: Time,
};

pub fn main() u8 {
    std.debug.warn("Hello world 3");

    if (sdl.SDL_Init(sdl.SDL_INIT_VIDEO | sdl.SDL_INIT_AUDIO) != 0) {
        std.debug.panic("sdl.SDL_Init failed: {c}\n", sdl.SDL_GetError());
    }
    defer sdl.SDL_Quit();

    var window: ?*sdl.SDL_Window = sdl.SDL_CreateWindow(
        c"Zigtroids",
        200,
        200,
        @intCast(c_int, 640),
        @intCast(c_int, 427),
        sdl.SDL_WINDOW_OPENGL | sdl.SDL_WINDOW_ALLOW_HIGHDPI,
    );
    defer sdl.SDL_DestroyWindow(window);

    var renderer: *sdl.SDL_Renderer = sdl.SDL_CreateRenderer(window, -1, sdl.SDL_RENDERER_ACCELERATED).?;
    defer sdl.SDL_DestroyRenderer(renderer);

    var gamestate = GameState{
        .renderer = renderer,
        .quit = false,
        .previous_time = Time{
            .elapsed_from_start = 0,
            .dt = 0,
        },
    };
    while (!gamestate.quit) {
        const time: Time = get_time(gamestate.previous_time);
        game_tick(&gamestate);
        gamestate.previous_time = time;

        var event: sdl.SDL_Event = undefined;
        while (sdl.SDL_PollEvent(&event) == 1) {
            if (event.type == sdl.SDL_WINDOWEVENT) {
                if (event.window.event == sdl.SDL_WINDOWEVENT_CLOSE) {
                    gamestate.quit = true;
                }
            }
        }
    }

    return 0;
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

fn game_tick(state: *GameState) void {
    if (sdl.SDL_SetRenderDrawColor(state.renderer, 0, 0, 0, 255) < 0) {
        log_sdl_error();
    }
    if (sdl.SDL_RenderClear(state.renderer) < 0) {
        log_sdl_error();
    }
    sdl.SDL_RenderPresent(state.renderer);
}
