const std = @import("std");
const term = @import("term.zig");

// As a note: We quouple-ify these, because in HLSL, we will be operating with
// uint4's.  We are going to uint4 data to/from system RAM.
//
// We're going to try to keep the full processor state to 12 x uint4.
const MiniRV32IMAState = struct {
	regs:[32]u32,

	pc:u32,
	mstatus:u32,
	cyclel:u32,
	cycleh:u32,

	timerl:u32,
	timerh:u32,
	timermatchl:u32,
	timermatchh:u32,

	mscratch:u32,
	mtvec:u32,
	mie:u32,
	mip:u32,

	mepc:u32,
	mtval:u32,
	mcause:u32,
	
	// Note: only a few bits are used.  (Machine = 3, User = 0)
	// Bits 0..1 = privilege.
	// Bit 2 = WFI (Wait for interrupt)
	// Bit 3 = Load/Store has a reservation.
	extraflags:u32,
};
const MINIRV32_RAM_IMAGE_OFFSET:u32 = 0x80000000;


const dtbData = @embedFile("sixtyfourmb.dtb");
const imageData = @embedFile("Image");

var console_scratch: [8192]u8 = undefined;
var console_fifo = std.fifo.LinearFifo(u8, .Slice).init(console_scratch[0..]);

const memSize = 64 * 1024 * 1024;

const stdout = std.io.getStdOut();


fn HandleControlStore(addr: u32, val: u32) u32 {
    if (addr == 0x10000000) { //UART 8250 / 16550 Data Buffer
        std.debug.print("{c}", .{@intCast(u8, val)});
        // FIXME term.write
    }
    return 0;
}

fn HandleControlLoad(addr: u32) u32 {
    // Emulating a 8250 / 16550 UART
    if (addr == 0x10000005) {
        if (console_fifo.count > 0) {
            return 0x60 | 1; //@intCast(u32, console_fifo.count);
        } else {
            return 0x60;
        }
    } else if (addr == 0x10000000 and console_fifo.count > 0) {
        var buf: [1]u8 = undefined;
        _ = console_fifo.read(&buf);
        return @intCast(u32, buf[0]);
    }
    return 0;
}

fn HandleException(ir: u32, code: u32) u32 {
    //std.log.info("PROCESSOR EXCEPTION ir:{x} code:{x}", .{ir, code});
    _ = ir;
    return code;
}

fn HandleOtherCSRWrite(image: [*]u8, csrno: u16, value: u32) void {
    _ = image;
    if (csrno == 0x136) {
        std.log.info("CSR 0x136: {d}", .{value});
    }
    if (csrno == 0x137) {
        std.log.info("CSR 0x137: {x}", .{value});
    }
    if (csrno == 0x138) {
        std.log.info("CSR 0x138: TBD dump data at {x}", .{value - MINIRV32_RAM_IMAGE_OFFSET});
    }
}

pub fn main() !void {
    const memory = try std.heap.page_allocator.alignedAlloc(u8, 4, memSize);
    @memset(memory.ptr, 0x00, memSize);
    defer std.heap.page_allocator.free(memory);

    // load image into ram
    @memcpy(memory.ptr, imageData, imageData.len);

    // load DTB into ram
    const dtb_off = memSize - dtbData.len - @sizeOf(MiniRV32IMAState);
    @memcpy(memory.ptr + dtb_off, dtbData, dtbData.len);

    // The core lives at the end of RAM.
    var core: *MiniRV32IMAState = @ptrCast(*MiniRV32IMAState, memory.ptr + (memSize - @sizeOf(MiniRV32IMAState)));

    core.pc = MINIRV32_RAM_IMAGE_OFFSET;
    core.regs[10] = 0x00; // hart ID
    core.regs[11] = dtb_off + MINIRV32_RAM_IMAGE_OFFSET;
    core.extraflags |= 3; // Machine-mode.

    // Update system ram size in DTB (but if and only if we're using the default DTB)
    // Warning - this will need to be updated if the skeleton DTB is ever modified.
    var dtb: [*]u32 = @ptrCast([*]u32, memory.ptr + dtb_off);
    if (dtb[0x13c / 4] == 0x00c0ff03) {
        const validram: u32 = dtb_off;
        dtb[0x13c / 4] = (validram >> 24) | (((validram >> 16) & 0xff) << 8) | (((validram >> 8) & 0xff) << 16) | ((validram & 0xff) << 24);
    }

    // Image is loaded.

    term.init();

    const startTime = std.time.microTimestamp();

    const instrs_per_flip: u32 = 1024;
    var lastTime: u32 = 0;
//var icount:usize = 0;
//const maxinstrs = 300000;
//    while (icount <= maxinstrs) : (icount+=1) {
    while(true) {
        if (term.getch()) |b| {
            const sl: [1]u8 = .{b};
            _ = console_fifo.write(&sl) catch null;
        }

        var this_ccount = core.cyclel;
        var elapsedUs: u32 = 0;
        const fixed_update = false;
        const time_divisor: u32 = 1;
        if (fixed_update) {
            elapsedUs = this_ccount / time_divisor - lastTime;
        } else {
            elapsedUs = @intCast(u32, @divFloor(std.time.microTimestamp() - startTime, time_divisor)) -% lastTime;
        }

        lastTime +%= elapsedUs;
        const ret = MiniRV32IMAStep_zig(core, memory, elapsedUs, instrs_per_flip);
        switch (ret) {
            0 => {},
            1 => {
                std.time.sleep(std.time.ns_per_us * 5000);
                core.cyclel += instrs_per_flip;
            },
            else => {
                std.log.info("Unknown failure ret code {d}", .{ret});
                std.os.exit(0);
            },
        }
    }
}

fn MiniRV32IMAStep_zig(state:*MiniRV32IMAState, image1:[] align(4) u8, elapsedUs:u32, count:usize) i32 {
    const fail_on_all_faults = false;
    const ramSize:u32 = @intCast(u32, image1.len);

    const new_timer = state.timerl +% elapsedUs;
    if (new_timer < state.timerl) {
        state.timerh +%= 1;
    }
    state.timerl = new_timer;

    // u16 and u32 access
    var image2 = std.mem.bytesAsSlice(u16, image1);
    var image4 = std.mem.bytesAsSlice(u32, image1);

	// Handle Timer interrupt.
	if( ( state.timerh > state.timermatchh or ( state.timerh == state.timermatchh and state.timerl > state.timermatchl ) ) and ( state.timermatchh != 0 or state.timermatchl != 0 ) ) {
		state.extraflags &= ~@as(u32, 4); // Clear WFI
		state.mip |= 1<<7; //MTIP of MIP // https://stackoverflow.com/a/61916199/2926815  Fire interrupt.
	} else {
	    state.mip &= ~(@as(u32, 1)<<@as(u32, 7));
    }

	if( state.extraflags & @as(u32,4) != 0 ) {
        return 1;
    }

    var icount:usize = 0;
    while(icount < count) : (icount += 1) {

        var ir:u32 = 0;
        var trap:u32 = 0;   // If positive, is a trap or interrupt.  If negative, is fatal error.
        var rval:u32 = 0;

		// Increment both wall-clock and instruction count time.  (NOTE: Not strictly needed to run Linux)
        state.cyclel +%= 1;
		if( state.cyclel == 0 ) {
            state.cycleh +%= 1;
        }
        
        var pc:u32 = state.pc;
        const ofs_pc = pc - MINIRV32_RAM_IMAGE_OFFSET;
    
        if (ofs_pc >= ramSize) {
            trap = 1 + 1;  // Handle access violation on instruction read.
        } else if (ofs_pc & 3 != 0) {
            trap = 1 + 0;  //Handle PC-misaligned access
        } else {
            ir = image4[ofs_pc/4];

            //_ = stdout.writer().print("ir={x} regs[11]={x}\n", .{ir, state.regs[11]}) catch 0;


            var rdid = (ir >> 7) & 0x1f;

			switch( ir & 0x7f ) {
                0b0110111 => rval = ( ir & 0xfffff000 ), // LUI
				0b0010111 => rval = pc +% ( ir & 0xfffff000 ), // AUIPC
				0b1101111 => { // JAL
					var reladdy:i32 = @intCast(i32, ((ir & 0x80000000)>>11) | ((ir & 0x7fe00000)>>20) | ((ir & 0x00100000)>>9) | ((ir&0x000ff000)));
					if( (reladdy & 0x00100000) != 0 ) {
                        reladdy = @bitCast(i32, @bitCast(u32, reladdy) | 0xffe00000); // Sign extension.
                    }
					rval = pc + 4;
                    pc = pc +% @bitCast(u32, reladdy - 4);
				},
                0b1100111 => { // JALR
                    const imm:u32 = ir >> 20;
                    var imm_se:i32 = @intCast(i32, imm);
                    if (imm & 0x800 != 0) {
                        imm_se = @bitCast(i32, imm | 0xfffff000);
                    }
					rval = pc + 4;
                    const newpc:u32 = ((state.regs[(ir >> 15) & 0x1f ] +% @bitCast(u32, imm_se)) & ~@as(u32,1)) - 4;
                    pc = newpc;
				},
                0b1100011 => { // Branch
					var immm4:u32 = ((ir & 0xf00)>>7) | ((ir & 0x7e000000)>>20) | ((ir & 0x80) << 4) | ((ir >> 31)<<12);
					if( immm4 & 0x1000 != 0 ) immm4 |= 0xffffe000;
					const rs1:i32 = @bitCast(i32, state.regs[(ir >> 15) & 0x1f]);
					const rs2:i32 = @bitCast(i32, state.regs[(ir >> 20) & 0x1f]);
					immm4 = pc +% (immm4 - 4);
					rdid = 0;

					switch( ( ir >> 12 ) & 0x7 ) {
						// BEQ, BNE, BLT, BGE, BLTU, BGEU 
						0b000 => { if( rs1 == rs2 ) pc = immm4; },
						0b001 => { if( rs1 != rs2 ) pc = immm4; },
						0b100 => { if( rs1 < rs2 ) pc = immm4; },
						0b101 => { if( rs1 >= rs2 ) pc = immm4; }, //BGE
						0b110 => { if( @bitCast(u32,rs1) < @bitCast(u32,rs2) ) pc = immm4; },  //BLTU
						0b111 => { if( @bitCast(u32,rs1) >= @bitCast(u32,rs2) ) pc = immm4; },  //BGEU
						else => { trap = (2+1); },
					}
				},
                0b0000011 => { // Load
                    const rs1:u32 = state.regs[(ir >> 15) & 0x1f];
                    const imm:u32 = ir >> 20;
                    var imm_se:i32 = @bitCast(i32, imm);
                    if (imm & 0x800 != 0) {
                        imm_se = @bitCast(i32, imm | 0xfffff000);
                    }
                    var rsval:u32 = @bitCast(u32, @bitCast(i32, rs1) + imm_se);

					rsval -%= MINIRV32_RAM_IMAGE_OFFSET;

					if( rsval >= ramSize-3 ) {
						rsval +%= MINIRV32_RAM_IMAGE_OFFSET;

						if( rsval >= 0x10000000 and rsval < 0x12000000 ) {  // UART, CLNT
							if( rsval == 0x1100bffc ) { // https://chromitem-soc.readthedocs.io/en/latest/clint.html
								rval = state.timerh;
							} else if( rsval == 0x1100bff8 ) {
								rval = state.timerl;
							} else {
                                rval = HandleControlLoad(rsval);
                            }
						}
						else {
							trap = (5+1);
							rval = rsval;
						}
					} else {
						switch( ( ir >> 12 ) & 0x7 ) {
							//LB, LH, LW, LBU, LHU
							0b000 => rval = @bitCast(u32, @intCast(i32, @bitCast(i8, image1[rsval]))),
							0b001 => rval = @bitCast(u32, @intCast(i32, @bitCast(i16, image2[rsval/2]))),
							0b010 => rval = image4[rsval/4],
							0b100 => rval = image1[rsval],
							0b101 => rval = image2[rsval/2],
                            else => trap = (2+1),
						}
					}
				},
                0b0100011 => { // Store
                    const rs1:u32 = state.regs[(ir >> 15) & 0x1f];
                    const rs2:u32 = state.regs[(ir >> 20) & 0x1f];
                    var addy:u32 = ( ( ir >> 7 ) & 0x1f ) | ( ( ir & 0xfe000000 ) >> 20 );

					if( addy & 0x800 != 0 ) {
                        addy |= 0xfffff000;
                    }
					addy +%= rs1 -% MINIRV32_RAM_IMAGE_OFFSET;
					rdid = 0;

					if( addy >= ramSize-3 ) {
						addy +%= MINIRV32_RAM_IMAGE_OFFSET;
						if( addy >= 0x10000000 and addy < 0x12000000 ) {
							// Should be stuff like SYSCON, 8250, CLNT
							if( addy == 0x11004004 ) { //CLNT
								state.timermatchh = rs2;
							} else if( addy == 0x11004000 ) { //CLNT
								state.timermatchl = rs2;
							} else if( addy == 0x11100000 ) { //SYSCON (reboot, poweroff, etc.)
                                state.pc = pc + 4;
								return @bitCast(i32, rs2); // NOTE: PC will be PC of Syscon.
							} else {
                                if (HandleControlStore(addy, rs2) > 0) {
                                    return @bitCast(i32, rs2);
                                }
                            }
						}
						else {
							trap = (7+1); // Store access fault.
							rval = addy;
						}
					} else {
						switch( ( ir >> 12 ) & 0x7 ) {
							//SB, SH, SW
							0b000 => image1[addy] = @truncate(u8, rs2),
							0b001 => image2[addy/2] = @truncate(u16, rs2),
							0b010 => {
                                //_ = stdout.writer().print("SW addy={x} rs2={x}\n", .{addy, rs2}) catch 0;
                                //std.os.exit(0);
                                image4[addy/4] = rs2;
                            },
							else => trap = (2+1),
						}
					}
				},
                0b0010011, 0b0110011 => { // Op-immediate, Op
                    var imm:u32 = ir >> 20;
                    if (imm & 0x800 != 0) {
                        imm |= 0xfffff000;
                    }

                    const rs1:u32 = state.regs[(ir >> 15) & 0x1f];
                    const is_reg = (ir & 0b100000) != 0;
                    var rs2 = imm;
                    if (is_reg) {
                        rs2 = state.regs[imm & 0x1f];
                    }

					if( is_reg and ( ir & 0x02000000 != 0 ) ) {
						switch( (ir>>12)&7 ) { //0x02000000 = RV32M
							0b000 => rval = rs1 *% rs2, // MUL
                            0b001 => rval = @intCast(u32, @as(i64, @bitCast(i32, rs1)) *% @as(i64, @bitCast(i32, rs2)) >> 32), // MULH
                            0b010 => rval = @intCast(u32, (@as(i64, @bitCast(i32, rs1)) *% @intCast(i64, rs2)) >> 32), // MULHSU
                            0b011 => rval = @intCast(u32, (@intCast(u64, rs1) *% @intCast(u64, rs2)) >> 32), // MULHU
							0b100 => {  // DIV
                                if (rs2 == 0) {
                                    rval = @bitCast(u32, @as(i32, -1));
                                } else {
                                    rval = @bitCast(u32, @divTrunc(@bitCast(i32,rs1), @bitCast(i32,rs2)));
                                }
                            },
                            0b101 => {  // DIVU
                                if (rs2 == 0) {
                                    rval = 0xffffffff;
                                } else {
                                    rval = @divFloor(rs1, rs2);
                                }
                            },
                            0b110 => { // REM
                                if (rs2 == 0) {
                                    rval = rs1;
                                } else {
                                    rval = @intCast(u32, @mod(@bitCast(i32, rs1), @bitCast(i32, rs2)));
                                }
                            },
                            0b111 => { // REMU
                                if (rs2 == 0) {
                                    rval = rs1;
                                } else {
                                    rval = @rem(rs1, rs2);
                                }
                            },
                            else => unreachable,
						}
					} else {
						switch( (ir>>12)&7 ) { // These could be either op-immediate or op commands.  Be careful.
							0b000 => {
                                if (is_reg and (ir & 0x40000000) != 0 ) {
                                    rval = rs1 -% rs2;
                                } else {
                                    rval = rs1 +% rs2;
                                }
                            },
                            0b001 => {
                                rval = @shlWithOverflow(rs1, @intCast(u5, @mod(rs2, 32)))[0];
                            },
                            0b010 => {
                                if (@bitCast(i32, rs1) < @bitCast(i32, rs2)) {
                                    rval = 1;
                                } else {
                                    rval = 0;
                                }
                            },
                            0b011 => {
                                if (rs1 < rs2) {
                                    rval = 1;
                                } else {
                                    rval = 0;
                                }
                            },
							0b100 => rval = rs1 ^ rs2,
                            0b101 => {
                                const n = @intCast(u5, rs2 & 0x1F);
                                if (ir & 0x40000000 != 0) {
                                    if (n < 32) {
                                        rval = @bitCast(u32, @bitCast(i32, rs1) >> n);
                                    } else {
                                        rval = 0;
                                    }
                                } else if (rs2 >= 32) {
                                    rval = 0;
                                } else {
                                    rval = rs1 >> n; 
                                }
                            },
							0b110 => rval = rs1 | rs2,
							0b111 => rval = rs1 & rs2,
                            else => unreachable,
						}
					}
				},
                0b0001111 => rdid = 0, // fencetype = (ir >> 12) & 0b111; We ignore fences in this impl.
                0b1110011 => {  // Zifencei+Zicsr
                    const csrno:u32 = ir >> 20;
                    const microop:u32 = ( ir >> 12 ) & 0b111;
					if( (microop & 3) != 0 ) { // It's a Zicsr function.
                        const rs1imm:u32 = (ir >> 15) & 0x1f;
                        const rs1 = state.regs[rs1imm];
                        var writeval:u32 = rs1;

						// https://raw.githubusercontent.com/riscv/virtual-memory/main/specs/663-Svpbmt.pdf
						// Generally, support for Zicsr
						switch( csrno ) {
                            0x340 => rval = state.mscratch,
                            0x305 => rval = state.mtvec,
                            0x304 => rval = state.mie,
                            0xC00 => rval = state.cyclel,
                            0x344 => rval = state.mip,
                            0x341 => rval = state.mepc,
                            0x300 => rval = state.mstatus, //mstatus
                            0x342 => rval = state.mcause,
                            0x343 => rval = state.mtval,
                            0xf11 => rval = 0xff0ff0ff, //mvendorid
                            0x301 => rval = 0x40401101, //misa (XLEN=32, IMA+X)
                            //0x3B0 => rval = 0, //pmpaddr0
                            //0x3a0 => rval = 0, //pmpcfg0
                            //0xf12 => rval = 0x00000000, //marchid
                            //0xf13 => rval = 0x00000000, //mimpid
                            //0xf14 => rval = 0x00000000, //mhartid
                            else => {},//MINIRV32_OTHERCSR_READ( csrno, rval ),
						}	

						switch( microop ) {
							0b001 => writeval = rs1,  			//CSRRW
							0b010 => writeval = rval | rs1,		//CSRRS
							0b011 => writeval = rval & ~rs1,		//CSRRC
							0b101 => writeval = rs1imm,			//CSRRWI
							0b110 => writeval = rval | rs1imm,	//CSRRSI
							0b111 => writeval = rval & ~rs1imm,	//CSRRCI
                            else => unreachable,
        				}

						switch( csrno ) {
                            0x340 => state.mscratch = writeval,
                            0x305 => state.mtvec = writeval,
                            0x304 => state.mie = writeval,
                            0x344 => state.mip = writeval,
                            0x341 => state.mepc = writeval,
                            0x300 => state.mstatus = writeval, //mstatus
                            0x342 => state.mcause = writeval,
                            0x343 => state.mtval = writeval,
                            //0x3a0 =>  ; //pmpcfg0
                            //0x3B0 =>  ; //pmpaddr0
                            //0xf11 =>  ; //mvendorid
                            //0xf12 =>  ; //marchid
                            //0xf13 =>  ; //mimpid
                            //0xf14 =>  ; //mhartid
                            //0x301 =>  ; //misa
                            else => {}, //MINIRV32_OTHERCSR_WRITE( csrno, writeval );
                        }
        			}
					else if( microop == 0b000 ) { // "SYSTEM"
						rdid = 0;
						if( csrno == 0x105 ) { //WFI (Wait for interrupts)
							state.mstatus |= 8;    //Enable interrupts
							state.extraflags |= 4; //Infor environment we want to go to sleep.
							state.pc = pc + 4;
							return 1;
						}
						else if( ( ( csrno & 0xff ) == 0x02 ) ) {  // MRET
							//https://raw.githubusercontent.com/riscv/virtual-memory/main/specs/663-Svpbmt.pdf
							//Table 7.6. MRET then in mstatus/mstatush sets MPV=0, MPP=0, MIE=MPIE, and MPIE=1. La
							// Should also update mstatus to reflect correct mode.
							const startmstatus = state.mstatus;
							const startextraflags = state.extraflags;
							state.mstatus = (( startmstatus & 0x80) >> 4) | ((startextraflags&3) << 11) | 0x80;
							state.extraflags = (startextraflags & ~@as(u32, 3)) | ((startmstatus >> 11) & @as(u32, 3));
							pc = state.mepc - 4;
						} else {
							switch( csrno ) {
                                0 => {  // ECALL; 8 = "Environment call from U-mode"; 11 = "Environment call from M-mode"
                                    if (state.extraflags & 3 != 0) {
                                        trap = (11+1);
                                    } else {
                                        trap = (8+1);
                                    }
                                },
                                1 => trap = (3+1), // EBREAK 3 = "Breakpoint"
                                else => trap = (2+1), // Illegal opcode.
							}
						}
					} else {
                        trap = (2+1); // Note micrrop 0b100 == undefined.
                    }
				},
                0b0101111 => { // RV32A
                    var rs1:u32 = state.regs[(ir >> 15) & 0x1f];
                    var rs2:u32 = state.regs[(ir >> 20) & 0x1f];
                    const irmid:u32 = ( ir>>27 ) & 0x1f;

					rs1 -= MINIRV32_RAM_IMAGE_OFFSET;

					// We don't implement load/store from UART or CLNT with RV32A here.
					if( rs1 >= ramSize-3 ) {
						trap = (7+1); //Store/AMO access fault
						rval = rs1 + MINIRV32_RAM_IMAGE_OFFSET;
					} else {
                        rval = image4[rs1/4];
						// Referenced a little bit of https://github.com/franzflasch/riscv_em/blob/master/src/core/core.c
                        var dowrite:u32 = 1;
						switch( irmid ) {
							0b00010 => {
                                dowrite = 0;
                                state.extraflags |= 8; //LR.W
                            },
							0b00011 => {
                                if (!(state.extraflags & 8 != 0)) { // SC.W (Lie and always say it's good)
                                    rval = 1;
                                } else {
                                    rval = 0;
                                }
                            },
							0b00001 => {}, //AMOSWAP.W
							0b00000 => rs2 +%= rval, //AMOADD.W
							0b00100 => {
                                rs2 ^= rval; //AMOXOR.W
                            },
							0b01100 => rs2 &= rval, //AMOAND.W
							0b01000 => rs2 |= rval, //AMOOR.W
							0b10000 => { // AMOMIN.W
                                if (!(@bitCast(i32, rs2) < @bitCast(i32, rval))) {
                                    rs2 = rval;
                                }
                            },
							0b10100 => { // AMOMAX.W
                                if (!(@bitCast(i32, rs2) > @bitCast(i32, rval))) {
                                    rs2 = rval;
                                }
                            },
                            0b11000 => { // AMOMINU.W
                                if (!(rs2 < rval)) {
                                    rs2 = rval;
                                }
                            },
							0b11100 => { // AMOMAXU.W
                                if (!(rs2 > rval)) {
                                    rs2 = rval;
                                }
                            },
							else => {
                                trap = (2+1);
                                dowrite = 0; //Not supported.
                            },
						}
						if( dowrite != 0) {
                            image4[rs1/4] = rs2;
                        }
					}
				},
                else => trap = (2+1) // Fault: Invalid opcode.
			}

			if( trap == 0 ) {
				if( rdid != 0) {
                    state.regs[rdid] = rval;
				} else if( ( state.mip & (1<<7) != 0 ) and ( state.mie & (1<<7) != 0) and ( state.mstatus & 0x8 != 0 ) )
				{   // Write back register.
					trap = 0x80000007; // Timer interrupt.
				}
			}
        }

        if( trap > 0 ) {
            if( fail_on_all_faults ) {
                //*printf( "FAULT\n" );*/
                return 3;
            } else { 
                trap = HandleException( ir, trap );
            }
        }

		// Handle traps and interrupts.
		if( trap != 0 ) {
			if( trap & 0x80000000 != 0 ) { // If prefixed with 1 in MSB, it's an interrupt, not a trap.
				state.extraflags &= ~@as(u32,8);
				state.mcause = trap;
				state.mtval = 0;
				pc += 4; // PC needs to point to where the PC will return to.
			} else {
				state.mcause = trap - 1;
                if (trap > 5 and trap <= 8) {
                    state.mtval = rval;
                } else {
                    state.mtval = pc;
                }
			}
			state.mepc = pc; //TRICKY: The kernel advances mepc automatically.
			//CSR( mstatus ) & 8 = MIE, & 0x80 = MPIE
			// On an interrupt, the system moves current MIE into MPIE
			state.mstatus = (( ( state.mstatus ) & 0x08) << 4) | (( ( state.extraflags ) & 3 ) << 11);
			pc = (( state.mtvec ) - 4);

			// XXX TODO: Do we actually want to check here? Is this correct?
			if( !(trap & 0x80000000 != 0) ) {
				state.extraflags |= 3;
            }
		}
		state.pc = pc + 4;
    }
    return 0;
}




