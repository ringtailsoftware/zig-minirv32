const std = @import("std");
const c = @cImport({
    @cInclude("stdlib.h");
    @cInclude("termios.h");
    @cInclude("unistd.h");
    @cInclude("sys/ioctl.h");
});

// implementation of getch and write for linux terminal

var originalTermios: c.struct_termios = undefined;

pub fn init() void {
    if (c.tcgetattr(std.os.linux.STDIN_FILENO, &originalTermios) < 0) {
        std.debug.print("could not get terminal settings\n", .{});
        std.os.exit(1);
    }

    var raw: c.struct_termios = originalTermios;

    // raw mode
    raw.c_iflag &= ~@as(c_uint, c.BRKINT | c.ICRNL | c.INPCK | c.ISTRIP | c.IXON);
    raw.c_lflag &= ~@as(c_uint, c.ECHO | c.ICANON | c.IEXTEN | c.ISIG);

    // non-blocking reads
    raw.c_cc[c.VMIN] = 0;
    raw.c_cc[c.VTIME] = 0; //  // 0.1s timeout

    if (c.tcsetattr(std.os.linux.STDIN_FILENO, c.TCSANOW, &raw) < 0) {
        std.debug.print("could not set new terminal settings\n", .{});
        std.os.exit(1);
    }

    _ = c.atexit(cleanup_terminal);
}

fn cleanup_terminal() callconv(.C) void {
    _ = c.tcsetattr(std.os.linux.STDIN_FILENO, c.TCSANOW, &originalTermios);
    std.debug.print("\nFinished\n", .{});
}

pub fn getch() ?u8 {
    var b: u8 = undefined;
    const count = c.read(std.os.linux.STDIN_FILENO, &b, 1);
    if (count == 0) {
        return null;
    }

    // hack to get out
    if (b == std.ascii.control_code.etx) {
        std.os.exit(1);
    }

    return b;
}

pub fn write(buf: []const u8) !void {
    const count = c.write(std.os.linux.STDOUT_FILENO, @ptrCast(*const anyopaque, buf.ptr), buf.len);
    if (count < buf.len) {
        std.debug.print("\nTBD, implement write retries\n", .{});
    }
}
