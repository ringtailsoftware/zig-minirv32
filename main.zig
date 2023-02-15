const std = @import("std");
const term = @import("term.zig");

const rv32 = @cImport({
    @cInclude("mini-rv32ima.h");
    @cInclude("default64mbdtc.h");
});

var console_scratch: [8192]u8 = undefined;
var console_fifo = std.fifo.LinearFifo(u8, .Slice).init(console_scratch[0..]);

const ramSize = 16*1024*1024;

export fn HandleControlStore(addr:u32, val:u32) callconv(.C) u32 {
    if (addr == 0x10000000 ) { //UART 8250 / 16550 Data Buffer
        std.debug.print("{c}", .{@intCast(u8, val)});
        // FIXME term.write
    }
    return 0;
}

export fn HandleControlLoad(addr:u32) callconv(.C) u32 {
	// Emulating a 8250 / 16550 UART
	if (addr == 0x10000005) {
        if (console_fifo.count > 0) {
            return 0x60 | 1;//@intCast(u32, console_fifo.count);
        } else {
            return 0x60;
        }
    } else if (addr == 0x10000000 and console_fifo.count > 0) {
        var buf:[1]u8 = undefined;
        _ = console_fifo.read(&buf);
		return @intCast(u32, buf[0]);
    }
	return 0;
}

export fn HandleException(ir:u32, code:u32) callconv(.C) u32 {
    //std.log.info("PROCESSOR EXCEPTION ir:{x} code:{x}", .{ir, code});
    _ = ir;
	return code;
}

export fn HandleOtherCSRWrite(image:[*]u8, csrno:u16, value:u32) callconv(.C) void {
    _ = image;
	if (csrno == 0x136 ) {
        std.log.info("CSR 0x136: {d}", .{value});
	}
	if (csrno == 0x137) {
        std.log.info("CSR 0x137: {x}", .{value});
	}
	if (csrno == 0x138) {
        std.log.info("CSR 0x138: TBD dump data at {x}", .{value - rv32.MINIRV32_RAM_IMAGE_OFFSET});
	}
}

pub fn main() !void {
    const memory = try std.heap.page_allocator.alignedAlloc(u8, 4, ramSize);
    @memset(memory.ptr, 0x00, ramSize);
    defer std.heap.page_allocator.free(memory);

    // Get the path
    var path_buffer: [std.fs.MAX_PATH_BYTES]u8 = undefined;
    const path = try std.fs.realpath("Image", &path_buffer);

    // Open the file
    const file = try std.fs.openFileAbsolute(path, .{});
    defer file.close();

    // Read the contents
    _ = try file.readAll(memory);

    // use default dtb
	const dtb_off = ramSize - rv32.default64mbdtb.len - @sizeOf(rv32.MiniRV32IMAState);
	@memcpy(memory.ptr + dtb_off, &rv32.default64mbdtb, rv32.default64mbdtb.len);

	// The core lives at the end of RAM.
    var core:*rv32.MiniRV32IMAState = @ptrCast(*rv32.MiniRV32IMAState, memory.ptr + (ramSize - @sizeOf(rv32.MiniRV32IMAState)));

	core.pc = rv32.MINIRV32_RAM_IMAGE_OFFSET;
	core.regs[10] = 0x00; // hart ID
	core.regs[11] = dtb_off + rv32.MINIRV32_RAM_IMAGE_OFFSET;
	core.extraflags |= 3; // Machine-mode.

    // Update system ram size in DTB (but if and only if we're using the default DTB)
    // Warning - this will need to be updated if the skeleton DTB is ever modified.
    var dtb:[*]u32 = @ptrCast([*]u32, memory.ptr + dtb_off);
    if (dtb[0x13c/4] == 0x00c0ff03) {
        const validram:u32 = dtb_off;
        dtb[0x13c/4] = (validram>>24) | ((( validram >> 16 ) & 0xff) << 8 ) | (((validram>>8) & 0xff ) << 16 ) | ( ( validram & 0xff) << 24 );
	}

	// Image is loaded.

    term.init();

    const startTime = std.time.microTimestamp();

    const instrs_per_flip:u32 = 1024;
    var lastTime:u32 = 0;
    while(true) {
        if (term.getch()) |b| {
            const sl:[1]u8 = .{b};
            _ = console_fifo.write(&sl) catch null;
        }

        var this_ccount = core.cyclel;
        var elapsedUs:u32 = 0;
        const fixed_update = false;
        const time_divisor:u32 = 1;
        if (fixed_update) {
            elapsedUs = this_ccount / time_divisor - lastTime;
        } else {
            elapsedUs = @intCast(u32, @divFloor(std.time.microTimestamp() - startTime, time_divisor)) - lastTime;
        }

        lastTime += elapsedUs;
        const ret = rv32.MiniRV32IMAStep(core, memory.ptr, 0, elapsedUs, instrs_per_flip, ramSize);
        switch(ret) {
            0 => {},
            1 => {
                std.time.sleep(std.time.ns_per_us * 5000);
                core.cyclel += instrs_per_flip;
            },
            else => {
                std.log.info("Unknown failure ret code {d}", .{ret});
            }
        }
    }
}
