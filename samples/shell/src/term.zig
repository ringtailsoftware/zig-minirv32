const std = @import("std");

const UART_BUF_REG_ADDR:usize = 0x10000000;
const UART_STATE_REG_ADDR:usize = 0x10000005;
const SYSCON_REG_ADDR: usize = 0x11100000;

const syscon = @as(*volatile u8, @ptrFromInt(SYSCON_REG_ADDR));
const uart_buf_reg = @as(*volatile u8, @ptrFromInt(UART_BUF_REG_ADDR));
const uart_state_reg = @as(*volatile u8, @ptrFromInt(UART_STATE_REG_ADDR));

var tw = TermWriter{};

pub fn getch() ?u8 {
    if (uart_state_reg.* & ~@as(u8, 0x60) > 0) {
        return uart_buf_reg.*;
    } else {
        return null;
    }
}

fn uart_write(buf:[]const u8) !void {
    var i:usize = 0;
    while(i < buf.len) : (i+=1) {
        uart_buf_reg.* = buf[i];
    }
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
