pub usingnamespace @cImport({
    @cInclude("SDL.h");
});

const std = @import("std");

pub fn log_sdl_error() void {
    var err = SDL_GetError() orelse return;
    std.debug.warn("Error in SDL: {s}\n", .{err});
}
