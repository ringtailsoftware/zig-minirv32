const std = @import("std");

const SYSCON_REG_ADDR:usize = 0x11100000;
const UART_BUF_REG_ADDR:usize = 0x10000000;
const UART_STATE_REG_ADDR:usize = 0x10000005;

// https://github.com/ziglang/zig/issues/21033
const PeripheralTypeU8 = struct {
    raw: extern struct {
        value: u8
    },
};
const PeripheralTypeU32 = struct {
    raw: extern struct {
        value: u32
    },
};

var uartreg: *volatile PeripheralTypeU8 = @ptrFromInt(UART_BUF_REG_ADDR);
var uartstatereg: *volatile PeripheralTypeU8 = @ptrFromInt(UART_STATE_REG_ADDR);
var sysconreg: *volatile PeripheralTypeU32 = @ptrFromInt(SYSCON_REG_ADDR);

var tw = TermWriter{};

pub fn getch() ?u8 {
    if (uartstatereg.raw.value & ~@as(u8, 0x60) > 0) {
        return uartreg.raw.value;
    } else {
        return null;
    }
}

fn uart_write(buf:[]const u8) !void {
    for (buf) |c| uartreg.raw.value = c;
}

// Implement a std.io.Writer backed by uart_write()
const TermWriter = struct {
    const Writer = std.io.Writer(
        *TermWriter,
        error{},
        write,
    );

    fn write(
        self: *TermWriter,
        data: []const u8,
    ) error{}!usize {
        _ = self;
        try uart_write(data);
        return data.len;
    }

    pub fn writer(self: *TermWriter) Writer {
        return .{ .context = self };
    }
};

pub fn getWriter() *TermWriter {
    return &tw;
}
