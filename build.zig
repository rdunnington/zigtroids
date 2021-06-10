// const std = @import("std");
// const Builder = std.build.Builder;
// const builtin = @import("builtin");

// pub fn build(b: *Builder) void {
//     const LibExeObjStep = std.build.LibExeObjStep;

//     var main: *LibExeObjStep = b.addExecutable("game", "game.zig");
//     main.setBuildMode(b.standardReleaseOptions());

//     main.addIncludeDir("include");
//     main.addIncludeDir("include/SDL2");
//     // main.addLibPath("D:\\Dev\\zig_projects\\asteroid\\lib\\SDL-win");
//     main.addLibPath("D:\\Dev\\zig_projects\\asteroid\\lib\\SDL-super\\x64");
//     b.installBinFile("D:\\Dev\\zig_projects\\asteroid\\lib\\SDL-super\\x64\\SDL2.dll", "SDL2.dll");
//     main.linkSystemLibrary("SDL2");
//     main.linkSystemLibrary("c");
//     // main.linkSystemLibrary("user32");
//     // main.linkSystemLibrary("ole32");
//     // main.linkSystemLibrary("winmm");
//     // main.linkSystemLibrary("gdi32");
//     // main.linkSystemLibrary("setupapi");
//     // main.linkSystemLibrary("imm32");
//     // main.linkSystemLibrary("version");
//     // main.linkSystemLibrary("oleaut32");

//     b.default_step.dependOn(&main.step);
//     b.installArtifact(main);

//     const play = b.step("play", "Play the game");
//     const run = main.run();
//     play.dependOn(&run.step);
// }

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
    exe.linkLibC();
    exe.install();

    const run_command = exe.run();
    run_command.step.dependOn(b.getInstallStep());

    const play_step = b.step("play", "Play the game");
    play_step.dependOn(&run_command.step);
}
