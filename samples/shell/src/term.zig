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

pub fn uart_write(buf:[]const u8) !void {
    for (buf) |c| uartreg.raw.value = c;
}


var wbuf:[4096]u8 = undefined;
var cw = TermWriter.init(&wbuf);

pub const WriteError = error{ Unsupported, NotConnected };

pub const TermWriter = struct {
    interface: std.Io.Writer,
    err: ?WriteError = null,

    fn drain(w: *std.Io.Writer, data: []const []const u8, splat: usize) std.Io.Writer.Error!usize {
        var ret: usize = 0;

        const b = w.buffered();
        _ = uart_write(b) catch 0;
        _ = w.consume(b.len);

        for (data) |d| {
            _ = uart_write(d) catch 0;
            ret += d.len;
        }

        const pattern = data[data.len - 1];
        for (0..splat) |_| {
            _ = uart_write(pattern) catch 0;
            ret += pattern.len;
        }

        return ret;
    }

    pub fn init(buf: []u8) TermWriter {
        return TermWriter{
            .interface = .{
                .buffer = buf,
                .vtable = &.{
                    .drain = drain,
                },
            },
        };
    }
};

pub fn getWriter() *std.Io.Writer {
    return &cw.interface;
}

