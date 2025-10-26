const std = @import("std");

const shell = @import("shell.zig");
const term = @import("term.zig");

export fn kmain() noreturn {
    _ = term.uart_write("BOOT") catch 0;

    _ = term.getWriter().print("Hello", .{}) catch 0;
    _ = term.getWriter().flush() catch 0;

    _ = shell.init() catch 0;

    while (true) {
        _ = shell.loop() catch 0;
        _ = term.getWriter().flush() catch 0;
    }
}
