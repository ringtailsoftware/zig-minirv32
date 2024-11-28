const std = @import("std");

const shell = @import("shell.zig");
const term = @import("term.zig");

const SYSCON_REG_ADDR:usize = 0x11100000;
const syscon = @volatileCast(@as(*u32, @ptrFromInt(SYSCON_REG_ADDR)));

export fn _start() callconv(.Naked) noreturn {
    asm volatile ("la sp, _sstack");    // set stack pointer
    asm volatile ("add s0, sp, zero");
    @call(.auto, kmain, .{});
    syscon.* = 0x5555;
    while (true) {}
}

fn kmain() void {
    try shell.init();
    while (true) {
        try shell.loop();
    }
}
