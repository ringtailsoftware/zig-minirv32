# zig-minirv32ima

Toby Jaffey https://mastodon.me.uk/@tobyjaffey

A zig wrapper for https://github.com/cnlohr/mini-rv32ima

    zig build && ./zig-out/bin/zigrv32ima

Tested with `zig 0.11.0-dev.1507+6f13a725a` on linux/aarch64.

Fixes were made to left and right shift operations so they don't rely on undefined behaviour. Left shift needs to be arithmetic (`sarll()`) and right shift needs to handle shifting by over 32 bits.

libc is linked for access to the raw terminal. To remove, comment out `lib.linkSystemLibraryName("c");` in `build.zig`.
The `term` struct could be replaced by this minimal stub:

    const term = struct {
        pub fn init() void {
        }
        pub fn getch() ?u8 {
            return null;
        }
    };

