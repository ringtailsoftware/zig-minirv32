const std = @import("std");

const rv32ima_flags = [_][]const u8{
    "-Wall",
};

const rv32ima_sources = [_][]const u8{
    "mini-rv32ima/mini-rv32ima.c",
};

fn createrv32ima(b: *std.build.Builder) *std.build.LibExeObjStep {
    const lib = b.addStaticLibrary("rv32ima", null);
    lib.setBuildMode(.ReleaseSafe);
    lib.addCSourceFiles(&rv32ima_sources, &rv32ima_flags);
    lib.addIncludePath("mini-rv32ima");
    lib.linkSystemLibraryName("c");
    return lib;
}

pub fn build(b: *std.build.Builder) void {
    // Standard target options allows the person running `zig build` to choose
    // what target to build for. Here we do not override the defaults, which
    // means any target is allowed, and the default is native. Other options
    // for restricting supported target set are available.
    const target = b.standardTargetOptions(.{});

    // Standard release options allow the person running `zig build` to select
    // between Debug, ReleaseSafe, ReleaseFast, and ReleaseSmall.
    const mode = b.standardReleaseOptions();

    const exe = b.addExecutable("zigrv32ima", "main.zig");
    exe.setTarget(target);
    exe.setBuildMode(mode);

    const rv32ima = createrv32ima(b);
    exe.linkLibrary(rv32ima);
    exe.addIncludePath("mini-rv32ima");

    exe.install();

    const run_cmd = exe.run();
    run_cmd.step.dependOn(b.getInstallStep());
    if (b.args) |args| {
        run_cmd.addArgs(args);
    }

    const run_step = b.step("run", "Run the app");
    run_step.dependOn(&run_cmd.step);

    const exe_tests = b.addTest("main.zig");
    exe_tests.setTarget(target);
    exe_tests.setBuildMode(mode);

    const test_step = b.step("test", "Run unit tests");
    test_step.dependOn(&exe_tests.step);
}
