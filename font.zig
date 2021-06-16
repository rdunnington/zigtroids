const std = @import("std");
const sdl = @import("sdl2.zig");
const stb = @import("stb.zig");
// const lodepng = @import("lodepng.zig");

const bmfont = @import("lib/zig-bmfont/bmfont.zig");

pub const CharDrawInfo = struct {
    rect: sdl.SDL_Rect,
    texture_index: u32, // lookup into textures
    char_index: u32, // lookup into font.chars
};

pub const Font = struct {
    allocator: *std.mem.Allocator,
    font: bmfont.FontInfo,
    textures: []*sdl.SDL_Texture,
    char_lookup: std.AutoHashMap(u32, CharDrawInfo),

    pub fn deinit() void {
        font.deinit();
        for (textures) |texture| {
            sdl.SDL_DestroyTexture(texture);
        }
        std.allocator.free(textures);
        charDrawInfo.deinit();
    }
};

pub const LoadError = error{
    UnsupportedTextureType,
    FailedLoadingTexturePage,
    FailedCreatingTexture,
};

pub fn loadFont(filepath: []const u8, allocator: *std.mem.Allocator, renderer: *sdl.SDL_Renderer) !Font {
    var font: bmfont.FontInfo = try bmfont.loadBinaryFromPath(filepath, allocator);
    errdefer font.deinit();
    var textures: []*sdl.SDL_Texture = try allocator.alloc(*sdl.SDL_Texture, font.pages.len);
    errdefer allocator.free(textures);

    var dirname = std.fs.path.dirname(filepath).?;

    for (font.pages) |page, i| {
        var extension: []const u8 = std.fs.path.extension(page);

        if (!std.mem.eql(u8, extension, ".tga")) {
            return error.UnsupportedTextureType;
        }

        var path_parts = [_][]const u8{ dirname, page };
        var page_path: []const u8 = try std.fs.path.join(allocator, &path_parts);
        defer allocator.free(page_path);

        std.debug.print("path: {s}\n", .{page_path});

        var width: c_int = 0;
        var height: c_int = 0;
        var original_format: c_int = 0;
        // var channels_per_pixel = stb.STBI_rgb_alpha;
        var channels_per_pixel: c_int = 0;
        var pixels_c: [*c]u8 = stb.stbi_load(&page_path[0], &width, &height, &original_format, channels_per_pixel);
        channels_per_pixel = original_format;
        if (pixels_c == null) {
            return error.FailedLoadingTexturePage;
        }
        defer stb.stbi_image_free(pixels_c);

        // var file = try std.fs.cwd().openFile(page_path, .{ .read = true });
        // defer file.close();

        // var pixels_encoded = try file.readToEndAlloc(allocator, 32 * 1024 * 1024); // max size 32 MB
        // defer allocator.free(pixels_encoded);

        // var width: c_uint = 0;
        // var height: c_uint = 0;
        // var pixels_decoded: [*c]u8 = null;
        // var decode_error:c_uint = lodepng.lodepng_decode32(&pixels_decoded, &width, &height, &pixels_encoded[0], pixels_encoded.len);
        // if (decode_error != 0)
        // {
        //     std.debug.warn("Failed to decode image {s}. Error: {s}", .{page_path, lodepng.lodepng_error_text(decode_error)});
        //     return error.FailedLoadingTexturePage;
        // }
        // defer std.heap.c_allocator.free(pixels_decoded[0..(width * height * 4)]);

        const bit_depth: c_int = 32;
        const pitch: c_int = channels_per_pixel * width;
        // const pitch: c_int = 4 * @intCast(c_int, width); // lodepng_decode32 always outputs 32 bit, RGBA color
        const rmask: c_uint = 0xff000000;
        const gmask: c_uint = 0x00ff0000;
        const bmask: c_uint = 0x0000ff00;
        const amask: c_uint = 0x000000ff;
        // var surface: ?*sdl.SDL_Surface = sdl.SDL_CreateRGBSurfaceFrom(pixels_decoded, @intCast(c_int, width), @intCast(c_int, height), bit_depth, pitch, rmask, gmask, bmask, amask);
        var surface: ?*sdl.SDL_Surface = sdl.SDL_CreateRGBSurfaceFrom(pixels_c, width, height, bit_depth, pitch, rmask, gmask, bmask, amask);
        if (surface == null) {
            return error.FailedCreatingSurface;
        }
        defer sdl.SDL_FreeSurface(surface.?);

        var texture: ?*sdl.SDL_Texture = sdl.SDL_CreateTextureFromSurface(renderer, surface.?);
        if (texture == null) {
            return error.FailedCreatingTexture;
        } else {
            textures[i] = texture.?;
        }
    }

    var char_lookup = std.AutoHashMap(u32, CharDrawInfo).init(allocator);
    for (font.chars) |char, i| {
        var info = CharDrawInfo{
            .rect = sdl.SDL_Rect{
                .x = char.x,
                .y = char.y,
                .w = char.width,
                .h = char.height,
            },
            .texture_index = char.page,
            .char_index = @intCast(u32, i),
        };
        try char_lookup.putNoClobber(char.id, info);
    }

    return Font{
        .allocator = allocator,
        .font = font,
        .textures = textures,
        .char_lookup = char_lookup,
    };
}
