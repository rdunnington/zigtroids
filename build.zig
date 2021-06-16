const std = @import("std");

pub fn build(b: *std.build.Builder) void {
    const target = b.standardTargetOptions(.{ .default_target = .{ .abi = .gnu } });
    const mode = b.standardReleaseOptions();

    const exe = b.addExecutable("game", "game.zig");
    exe.setTarget(target);
    exe.setBuildMode(mode);

    const sdl_path = "lib\\SDL2-2.0.14\\";
    exe.addIncludeDir(sdl_path ++ "include");
    exe.addLibPath(sdl_path ++ "lib\\x64");
    b.installBinFile(sdl_path ++ "lib\\x64\\SDL2.dll", "SDL2.dll");
    exe.linkSystemLibrary("sdl2");

    exe.addIncludeDir("lib\\stb");
    exe.addIncludeDir("lib\\lodepng");

    exe.linkLibC();
    exe.install();

    const run_command = exe.run();
    run_command.step.dependOn(b.getInstallStep());

    const play_step = b.step("play", "Play the game");
    play_step.dependOn(&run_command.step);
}
