const SYSCON_REG_ADDR: usize = 0x11100000;
const UART_BUF_REG_ADDR: usize = 0x10000000;

const syscon = @volatileCast(@as(*u32, @ptrFromInt(SYSCON_REG_ADDR)));
const uart_buf_reg = @volatileCast(@as(*u32, @ptrFromInt(UART_BUF_REG_ADDR)));

export fn _start() callconv(.Naked) noreturn {
    asm volatile ("la sp, _sstack"); // set stack pointer
    asm volatile ("add s0, sp, zero"); // set frame pointer to stack pointer
    mandel();
    syscon.* = 0x5555; // send powerdown
    while (true) {}
}

// https://rosettacode.org/wiki/Mandelbrot_set
fn mandel() void {
    var xmin: i32 = -8601;
    var xmax: i32 = 2867;
    var ymin: i32 = -4915;
    var ymax: i32 = 4915;

    const maxiter: usize = 32;

    var dx: i32 = @divTrunc((xmax - xmin), 79);
    var dy: i32 = @divTrunc((ymax - ymin), 24);

    var cy = ymin;
    while (cy <= ymax) {
        var cx = xmin;
        while (cx <= xmax) {
            var x: i32 = 0;
            var y: i32 = 0;
            var x2: i32 = 0;
            var y2: i32 = 0;
            var iter: usize = 0;
            while (iter < maxiter) : (iter += 1) {
                if (x2 + y2 > 16384) break;
                y = ((x * y) >> 11) + cy;
                x = x2 - y2 + cx;
                x2 = (x * x) >> 12;
                y2 = (y * y) >> 12;
            }
            uart_buf_reg.* = ' ' + @intCast(u8, iter);

            cx += dx;
        }
        uart_buf_reg.* = '\r';
        uart_buf_reg.* = '\n';
        cy += dy;
    }
}
