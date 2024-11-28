const std = @import("std");

const shell = @import("shell.zig");
const term = @import("term.zig");

export fn kmain() noreturn {
    try shell.init();
    while (true) {
        try shell.loop();
    }
}
