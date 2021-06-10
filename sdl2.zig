pub usingnamespace @cImport({
    // @cInclude("lib/SDL2-2.0.14/include/SDL.h");
    @cInclude("SDL.h");
});

const std = @import("std");

pub fn log_sdl_error() void {
    var err = SDL_GetError() orelse return;
    std.debug.warn("Error in SDL: {s}\n", .{err});
}
