const std = @import("std");
const CrossTarget = @import("std").zig.CrossTarget;
const Target = @import("std").Target;
const Feature = @import("std").Target.Cpu.Feature;

pub fn build(b: *std.build.Builder) void {
    const features = Target.riscv.Feature;
    var disabled_features = Feature.Set.empty;
    var enabled_features = Feature.Set.empty;

    // disable all CPU extensions
    disabled_features.addFeature(@enumToInt(features.a));
    disabled_features.addFeature(@enumToInt(features.c));
    disabled_features.addFeature(@enumToInt(features.d));
    disabled_features.addFeature(@enumToInt(features.e));
    disabled_features.addFeature(@enumToInt(features.f));
    // except multiply
    enabled_features.addFeature(@enumToInt(features.m));

    const target = CrossTarget{
        .cpu_arch = Target.Cpu.Arch.riscv32,
        .os_tag = Target.Os.Tag.freestanding,
        .abi = Target.Abi.none,
        .cpu_model = .{ .explicit = &std.Target.riscv.cpu.generic_rv32},
        .cpu_features_sub = disabled_features,
        .cpu_features_add = enabled_features
    };

    const exe = b.addExecutable("mandelbrot", "src/main.zig");

    exe.setBuildMode(.ReleaseSmall);
    exe.setLinkerScriptPath(.{ .path = "src/linker.ld" });
    exe.strip = false;
    _ = exe.installRaw("mandelbrot.bin", .{});
    exe.setTarget(target);
    exe.install();
}
