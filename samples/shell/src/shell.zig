const std = @import("std");
const term = @import("term.zig");
const cmds = @import("cmds.zig");
const cmdTable = cmds.cmdTable;
const ascii = std.ascii.control_code;

const MAXARGS = 16;
const CMDBUF_SIZE_BYTES = 128;

var got_line:bool = false;
var cmdbuf:[CMDBUF_SIZE_BYTES+1]u8 = .{0} ** (CMDBUF_SIZE_BYTES+1);
var cmdbuf_len:usize = 0;

var tw = term.getWriter().writer();

// type definitions
pub const ArgList = [][]const u8;
pub const CmdHandler = *const fn(args:ArgList) CmdErr!void; // function pointer

// comptime helper to contruct handler table
pub fn makeCmd(comptime name:[] const u8, comptime handler: CmdHandler) Cmd {
    return Cmd{
        .name = name,
        .handler = handler,
    };
}

pub const CmdErr = error{
    BadArgs,
    Fail
};

pub const Cmd = struct{
    name: [] const u8,
    handler: CmdHandler,
};

pub fn banner() !void {
    _ = try tw.print("zig-embshell: type help or press tab!\n", .{});
}

pub fn prompt() !void {
    _ = try tw.print("> ", .{});
}

pub fn init() !void {
    got_line = false;
    cmdbuf_len = 0;
    cmdbuf[cmdbuf_len] = 0;

    try banner();
    try prompt();
}

// given an ArgList, take action
fn runcmd(args:ArgList) !void {
    if (args.len > 0) {
        for (cmdTable) |cmd| {
            if (std.mem.eql(u8, cmd.name, args[0])) {
                if (cmd.handler(args)) {    // execute command handler
                    _ = try tw.print("OK\n", .{});
                    return;
                } else |err| switch(err) {  // report error from command handler
                    CmdErr.BadArgs => _ = try tw.print("\nBad arguments\n", .{}),
                    CmdErr.Fail => _ = try tw.print("\nFailed\n", .{}),
                }
            }
        }
        // special case for help, generate it automatically from cmdTable
        if (std.mem.eql(u8, args[0], "help")) {
            for (cmdTable) |cmd| {
                _ = try tw.print("{s}\n", .{cmd.name});
            }
        }
    }
}

// execute a command line
fn execline(line:[]const u8) !void {
    // tokenize, returns iterator to slices
    var tokens = std.mem.tokenize(u8, line, " ");
    // setup argv array to hold tokens
    var argv:[MAXARGS] []const u8 = .{undefined} ** MAXARGS;
    var argc:u8 = 0;

    while (tokens.next()) |chunk| : (argc += 1) {
        if (argc >= MAXARGS) {
            break;
        }
        argv[argc] = chunk;
    }
    try runcmd(argv[0..argc]);
}

pub fn loop() !void {
    // try to get a key from terminal
    const key:?u8 = term.getch();
    if (key != null) {
        if (got_line) {
            // buffer is already full
            return;
        }
        switch(key.?) {
            ascii.etx => {  // ctrl-c
                //std.os.exit(0); // FIXME, should return error from loop?
            },
            ascii.cr, ascii.lf => {
                got_line = true;
                _ = try tw.print("\n", .{});
            },
            ascii.del, ascii.bs => {
                if (cmdbuf_len > 0) {
                    cmdbuf_len = cmdbuf_len-1;
                    cmdbuf[cmdbuf_len] = 0;
                    _ = try tw.print("{c} {c}", .{ascii.bs, ascii.bs});
                }
            },
            ascii.ht => { // Tab
                var matches:[cmdTable.len] usize = .{undefined} ** (cmdTable.len);  // indices of matching commands
                var numMatches:usize = 0;
                // look for matches
                for (cmdTable, 0..) |cmd, index| {
                    if (std.mem.startsWith(u8, cmd.name, cmdbuf[0..cmdbuf_len])) {
                        matches[numMatches] = index;
                        numMatches += 1;
                    }
                }
                if (numMatches > 0) {
                    switch(numMatches) {
                        1 => {  // exactly one match
                            const cmd = cmdTable[matches[0]];
                            _ = try tw.print("{s}", .{cmd.name[cmdbuf_len..]});
                            @memcpy(&cmdbuf, cmd.name);
                            cmdbuf_len = cmd.name.len;
                            cmdbuf[cmdbuf_len] = 0;
                        },
                        else => { // multiple matches
                            _ = try tw.print("\n", .{});
                            for (matches) |match| {
                                const cmd = cmdTable[match];
                                _ = try tw.print("{s}\n", .{cmd.name});
                            }
                            try prompt();
                            _ = try tw.print("{s}", .{cmdbuf[0..cmdbuf_len]});
                        }
                    }
                }
            },
            else => {
                // echo
                if (cmdbuf_len < CMDBUF_SIZE_BYTES) {
                    cmdbuf[cmdbuf_len] = key.?;
                    cmdbuf_len = cmdbuf_len + 1;
                    cmdbuf[cmdbuf_len] = 0;
                    _ = try tw.print("{c}", .{key.?});
                } else {
                    _ = try tw.print("{c}", .{ascii.bel});
                }
            }
        }
    }
    if (got_line) {
        if (cmdbuf_len > 0) {
            try execline(cmdbuf[0..cmdbuf_len]);
        }
        cmdbuf_len = 0;
        cmdbuf[cmdbuf_len] = 0;
        try prompt();
        got_line = false;
    }
}

