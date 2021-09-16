// Commander X16 Emulator
// Copyright (c) 2021 Simone Rolando
// All rights reserved. License: 2-clause BSD

#ifndef _FAKE_6502_HPP
#define _FAKE_6502_HPP

#include <cstdint>
#include <cstdio>
#include <iostream>

namespace cpu
{

    constexpr auto FLAG_CARRY = 0x01;
    constexpr auto FLAG_ZERO = 0x02;
    constexpr auto FLAG_INTERRUPT = 0x04;
    constexpr auto FLAG_DECIMAL = 0x08;
    constexpr auto FLAG_BREAK = 0x10;
    constexpr auto FLAG_CONSTANT = 0x20;
    constexpr auto FLAG_OVERFLOW = 0x40;
    constexpr auto FLAG_SIGN = 0x80;

    constexpr auto BASE_STACK = 0x100;

    class fake_6502 {
    public:
        void reset();
        void step();
        void exec(uint32_t tick_count);
        void irq();
        void nmi();

        // Bind external function
        void hook_external(void(*func_ptr)());
    private:
        // 6502 registers
        uint16_t pc;
        uint8_t sp, a, x, y, status;

        // Variables
        uint32_t instructions = 0;
        uint32_t clock_ticks = 0, clock_goal = 0;
        uint16_t oldpc, ea, reladdr, value, result;
        uint8_t opcode, oldstatus;

        uint8_t penalty_op, penalty_addr;
        uint8_t waiting = 0;

        // Table function pointers
        typedef void (cpu::fake_6502::*voidfun)();

        // Tables
        voidfun addrtable[256] = {
            imp, indx,  imp,  imp,   zp,   zp,   zp,   zp,  imp,  imm,  acc,  imp, abso, abso, abso,zprel,
            rel, indy, ind0,  imp,   zp,  zpx,  zpx,   zp,  imp, absy,  acc,  imp, abso, absx, absx,zprel,
            abso, indx,  imp,  imp,   zp,   zp,   zp,   zp,  imp,  imm,  acc,  imp, abso, abso, abso,zprel,
            rel, indy, ind0,  imp,  zpx,  zpx,  zpx,   zp,  imp, absy,  acc,  imp, absx, absx, absx,zprel,
            imp, indx,  imp,  imp,  imp,   zp,   zp,   zp,  imp,  imm,  acc,  imp, abso, abso, abso,zprel,
            rel, indy, ind0,  imp,  imp,  zpx,  zpx,   zp,  imp, absy,  imp,  imp,  imp, absx, absx,zprel,
            mp, indx,  imp,  imp,   zp,   zp,   zp,   zp,  imp,  imm,  acc,  imp,  ind, abso, abso,zprel,
            rel, indy, ind0,  imp,  zpx,  zpx,  zpx,   zp,  imp, absy,  imp,  imp, ainx, absx, absx,zprel,
            rel, indx,  imp,  imp,   zp,   zp,   zp,   zp,  imp,  imm,  imp,  imp, abso, abso, abso,zprel,
            rel, indy, ind0,  imp,  zpx,  zpx,  zpy,   zp,  imp, absy,  imp,  imp, abso, absx, absx,zprel,
            imm, indx,  imm,  imp,   zp,   zp,   zp,   zp,  imp,  imm,  imp,  imp, abso, abso, abso,zprel,
            rel, indy, ind0,  imp,  zpx,  zpx,  zpy,   zp,  imp, absy,  imp,  imp, absx, absx, absy,zprel,
            imm, indx,  imp,  imp,   zp,   zp,   zp,   zp,  imp,  imm,  imp,  imp, abso, abso, abso,zprel,
            rel, indy, ind0,  imp,  imp,  zpx,  zpx,   zp,  imp, absy,  imp,  imp,  imp, absx, absx,zprel,
            imm, indx,  imp,  imp,   zp,   zp,   zp,   zp,  imp,  imm,  imp,  imp, abso, abso, abso,zprel,
            rel, indy, ind0,  imp,  imp,  zpx,  zpx,   zp,  imp, absy,  imp,  imp,  imp, absx, absx,zprel
        };

        voidfun optable[256] = {
            brk,  ora,  nop,  nop,  tsb,  ora,  asl,  rmb0, php,  ora,  asl,  nop,  tsb,  ora,  asl, bbr0,
            ora,  ora,  nop,  trb,  ora,  asl,  rmb1, clc,  ora,  inc,  nop,  trb,  ora,  asl,  bbr1,
            jsr,  and_, nop,  nop,  bit,  and_, rol,  rmb2,  plp, and_, rol,  nop,  bit,  and_, rol, bbr2,
            bmi,  and_, and_, nop,  bit,  and_, rol,  rmb3,  sec, and_, dec,  nop,  bit,  and_, rol, bbr3,
            rti,  eor,  nop,  nop,  nop,  eor,  lsr,  rmb4,  pha,  eor,  lsr,  nop,  jmp,  eor,  lsr, bbr4,
            bvc,  eor,  eor,  nop,  nop,  eor,  lsr,  rmb5,  cli,  eor,  phy,  nop,  nop,  eor,  lsr, bbr5,
            rts,  adc,  nop,  nop,  stz,  adc,  ror,  rmb6,  pla,  adc,  ror,  nop,  jmp,  adc,  ror, bbr6,
            bvs,  adc,  adc,  nop,  stz,  adc,  ror,  rmb7,  sei,  adc,  ply,  nop,  jmp,  adc,  ror, bbr7,
            bra,  sta,  nop,  nop,  sty,  sta,  stx,  smb0,  dey,  bit,  txa,  nop,  sty,  sta,  stx, bbs0,
            bcc,  sta,  sta,  nop,  sty,  sta,  stx,  smb1,  tya,  sta,  txs,  nop,  stz,  sta,  stz, bbs1,
            ldy,  lda,  ldx,  nop,  ldy,  lda,  ldx,  smb2,  tay,  lda,  tax,  nop,  ldy,  lda,  ldx, bbs2,
            bcs,  lda,  lda,  nop,  ldy,  lda,  ldx,  smb3,  clv,  lda,  tsx,  nop,  ldy,  lda,  ldx, bbs3,
            cpy,  cmp,  nop,  nop,  cpy,  cmp,  dec,  smb4,  iny,  cmp,  dex,  wai,  cpy,  cmp,  dec, bbs4,
            bne,  cmp,  cmp,  nop,  nop,  cmp,  dec,  smb5,  cld,  cmp,  phx,  dbg,  nop,  cmp,  dec, bbs5,
            cpx,  sbc,  nop,  nop,  cpx,  sbc,  inc,  smb6,  inx,  sbc,  nop,  nop,  cpx,  sbc,  inc, bbs6,
            beq,  sbc,  sbc,  nop,  nop,  sbc,  inc,  smb7,  sed,  sbc,  plx,  nop,  nop,  sbc,  inc, bbs7
        };

        uint32_t ticktable[256] = {
            7,    6,    2,    2,    5,    3,    5,    5,    3,    2,    2,    2,    6,    4,    6,    2,
            2,    5,    5,    2,    5,    4,    6,    5,    2,    4,    2,    2,    6,    4,    7,    2,
            6,    6,    2,    2,    3,    3,    5,    5,    4,    2,    2,    2,    4,    4,    6,    2,
            2,    5,    5,    2,    4,    4,    6,    5,    2,    4,    2,    2,    4,    4,    7,    2,
            6,    6,    2,    2,    2,    3,    5,    5,    3,    2,    2,    2,    3,    4,    6,    2,
            2,    5,    5,    2,    2,    4,    6,    5,    2,    4,    3,    2,    2,    4,    7,    2,
            6,    6,    2,    2,    3,    3,    5,    5,    4,    2,    2,    2,    5,    4,    6,    2,
            2,    5,    5,    2,    4,    4,    6,    5,    2,    4,    4,    2,    6,    4,    7,    2,
            3,    6,    2,    2,    3,    3,    3,    5,    2,    2,    2,    2,    4,    4,    4,    2,
            2,    6,    5,    2,    4,    4,    4,    5,    2,    5,    2,    2,    4,    5,    5,    2,
            2,    6,    2,    2,    3,    3,    3,    5,    2,    2,    2,    2,    4,    4,    4,    2,
            2,    5,    5,    2,    4,    4,    4,    5,    2,    4,    2,    2,    4,    4,    4,    2,
            2,    6,    2,    2,    3,    3,    5,    5,    2,    2,    2,    3,    4,    4,    6,    2,
            2,    5,    5,    2,    2,    4,    6,    5,    2,    4,    3,    1,    2,    4,    7,    2,
            2,    6,    2,    2,    3,    3,    5,    5,    2,    2,    2,    2,    4,    4,    6,    2,
            2,    5,    5,    2,    2,    4,    6,    5,    2,    4,    4,    2,    2,    4,    7,    2
        };

        const char* mnemonics[256] = {
            "brk ",
            "ora ($%02x,x)",
            "nop ",
            "nop ",
            "tsb $%02x",
            "ora $%02x",
            "asl $%02x",
            "rmb0 $%02x",
            "php ",
            "ora #$%02x",
            "asl a",
            "nop ",
            "tsb $%04x",
            "ora $%04x",
            "asl $%04x",
            "bbr0 $%02x, $%04x",
            "bpl $%02x",
            "ora ($%02x),y",
            "ora ($%02x)",
            "nop ",
            "trb $%02x",
            "ora $%02x,x",
            "asl $%02x,x",
            "rmb1 $%02x",
            "clc ",
            "ora $%04x,y",
            "inc a",
            "nop ",
            "trb $%04x",
            "ora $%04x,x",
            "asl $%04x,x",
            "bbr1 $%02x, $%04x",
            "jsr $%04x",
            "and ($%02x,x)",
            "nop ",
            "nop ",
            "bit $%02x",
            "and $%02x",
            "rol $%02x",
            "rmb2 $%02x",
            "plp ",
            "and #$%02x",
            "rol a",
            "nop ",
            "bit $%04x",
            "and $%04x",
            "rol $%04x",
            "bbr2 $%02x, $%04x",
            "bmi $%02x",
            "and ($%02x),y",
            "and ($%02x)",
            "nop ",
            "bit $%02x,x",
            "and $%02x,x",
            "rol $%02x,x",
            "rmb3 $%02x",
            "sec ",
            "and $%04x,y",
            "dec a",
            "nop ",
            "bit $%04x,x",
            "and $%04x,x",
            "rol $%04x,x",
            "bbr3 $%02x, $%04x",
            "rti ",
            "eor ($%02x,x)",
            "nop ",
            "nop ",
            "nop ",
            "eor $%02x",
            "lsr $%02x",
            "rmb4 $%02x",
            "pha ",
            "eor #$%02x",
            "lsr a",
            "nop ",
            "jmp $%04x",
            "eor $%04x",
            "lsr $%04x",
            "bbr4 $%0"
            "bvc $%02x",
            "eor ($%02x),y",
            "eor ($%02x)",
            "nop ",
            "nop ",
            "eor $%02x,x",
            "lsr $%02x,x",
            "rmb5 $%02x",
            "cli ",
            "eor $%04x,y",
            "phy ",
            "nop ",
            "nop ",
            "eor $%04x,x",
            "lsr $%04x,x",
            "bbr5 $%0"
            "rts ",
            "adc ($%02x,x)",
            "nop ",
            "nop ",
            "stz $%02x",
            "adc $%02x",
            "ror $%02x",
            "rmb6 $%02x",
            "pla ",
            "adc #$%02x",
            "ror a",
            "nop ",
            "jmp ($%04x)",
            "adc $%04x",
            "ror $%04x",
            "bbr6 $%0"
            "bvs $%02x",
            "adc ($%02x),y",
            "adc ($%02x)",
            "nop ",
            "stz $%02x,x",
            "adc $%02x,x",
            "ror $%02x,x",
            "rmb7 $%02x",
            "sei ",
            "adc $%04x,y",
            "ply ",
            "nop ",
            "jmp ($%04x,x)",
            "adc $%04x,x",
            "ror $%04x,x",
            "bbr7 $%0"
            "bra $%02x",
            "sta ($%02x,x)",
            "nop ",
            "nop ",
            "sty $%02x",
            "sta $%02x",
            "stx $%02x",
            "smb0 $%02x",
            "dey ",
            "bit #$%02x",
            "txa ",
            "nop ",
            "sty $%04x",
            "sta $%04x",
            "stx $%04x",
            "bbs0 $%0"
            "bcc $%02x",
            "sta ($%02x),y",
            "sta ($%02x)",
            "nop ",
            "sty $%02x,x",
            "sta $%02x,x",
            "stx $%02x,y",
            "smb1 $%02x",
            "tya ",
            "sta $%04x,y",
            "txs ",
            "nop ",
            "stz $%04x",
            "sta $%04x,x",
            "stz $%04x,x",
            "bbs1 $%0"
            "ldy #$%02x",
            "lda ($%02x,x)",
            "ldx #$%02x",
            "nop ",
            "ldy $%02x",
            "lda $%02x",
            "ldx $%02x",
            "smb2 $%02x",
            "tay ",
            "lda #$%02x",
            "tax ",
            "nop ",
            "ldy $%04x",
            "lda $%04x",
            "ldx $%04x",
            "bbs2 $%0"
            "bcs $%02x",
            "lda ($%02x),y",
            "lda ($%02x)",
            "nop ",
            "ldy $%02x,x",
            "lda $%02x,x",
            "ldx $%02x,y",
            "smb3 $%02x",
            "clv ",
            "lda $%04x,y",
            "tsx ",
            "nop ",
            "ldy $%04x,x",
            "lda $%04x,x",
            "ldx $%04x,y",
            "bbs3 $%0"
            "cpy #$%02x",
            "cmp ($%02x,x)",
            "nop ",
            "nop ",
            "cpy $%02x",
            "cmp $%02x",
            "dec $%02x",
            "smb4 $%02x",
            "iny ",
            "cmp #$%02x",
            "dex ",
            "wai ",
            "cpy $%04x",
            "cmp $%04x",
            "dec $%04x",
            "bbs4 $%0"
            "bne $%02x",
            "cmp ($%02x),y",
            "cmp ($%02x)",
            "nop ",
            "nop ",
            "cmp $%02x,x",
            "dec $%02x,x",
            "smb5 $%02x",
            "cld ",
            "cmp $%04x,y",
            "phx ",
            "dbg ",
            "nop ",
            "cmp $%04x,x",
            "dec $%04x,x",
            "bbs5 $%0"
            "cpx #$%02x",
            "sbc ($%02x,x)",
            "nop ",
            "nop ",
            "cpx $%02x",
            "sbc $%02x",
            "inc $%02x",
            "smb6 $%02x",
            "inx ",
            "sbc #$%02x",
            "nop ",
            "nop ",
            "cpx $%04x",
            "sbc $%04x",
            "inc $%04x",
            "bbs6 $%0"
            "beq $%02x",
            "sbc ($%02x),y",
            "sbc ($%02x)",
            "nop ",
            "nop ",
            "sbc $%02x,x",
            "inc $%02x,x",
            "smb7 $%02x",
            "sed ",
            "sbc $%04x,y",
            "plx ",
            "nop ",
            "nop ",
            "sbc $%04x,x",
            "inc $%04x,x",
            "bbs7 $%02x, $%04x"
        };

        // Additionals
        uint8_t call_external = 0;
        void (*loop_external)();
    private:
        // 6502 instructions;
        void adc();
        void and_();
        void asl();
        void bcc();
        void bcs();
        void beq();
        void bit();
        void bmi();
        void bne();
        void bpl();
        void brk();
        void bvc();
        void bvs();
        void clc();
        void cld();
        void cli();
        void clv();
        void cmp();
        void cpx();
        void cpy();
        void dec();
        void dex();
        void dey();
        void eor();
        void inc();
        void inx();
        void iny();
        void jmp();
        void jsr();
        void lda();
        void ldx();
        void ldy();
        void lsr();
        void nop();
        void ora();
        void pha();
        void php();
        void pla();
        void plp();
        void rol();
        void ror();
        void rti();
        void rts();
        void sbc();
        void sec();
        void sed();
        void sei();
        void sta();
        void stx();
        void sty();
        void tax();
        void tay();
        void tsx();
        void txa();
        void txs();
        void tya();

        // 65c02 instructions
        void ind0();
        void ainx();
        void stz();
        void bra();
        void phx();
        void plx();
        void phy();
        void ply();
        void tsb();
        void trb();
        void dbg();
        void wai();
        void bbr(uint16_t bitmask);

        inline void bbr0() { bbr(0x01); }
        inline void bbr1() { bbr(0x02); }
        inline void bbr2() { bbr(0x04); }
        inline void bbr3() { bbr(0x08); }
        inline void bbr4() { bbr(0x10); }
        inline void bbr5() { bbr(0x20); }
        inline void bbr6() { bbr(0x40); }
        inline void bbr7() { bbr(0x80); }

        void bbs(uint16_t bitmask);

        inline void bbs0() { bbs(0x01); }
        inline void bbs1() { bbs(0x02); }
        inline void bbs2() { bbs(0x04); }
        inline void bbs3() { bbs(0x08); }
        inline void bbs4() { bbs(0x10); }
        inline void bbs5() { bbs(0x20); }
        inline void bbs6() { bbs(0x40); }
        inline void bbs7() { bbs(0x80); }

        inline void smb0() { put_value(get_value() | 0x01); }
        inline void smb1() { put_value(get_value() | 0x02); }
        inline void smb2() { put_value(get_value() | 0x04); }
        inline void smb3() { put_value(get_value() | 0x08); }
        inline void smb4() { put_value(get_value() | 0x10); }
        inline void smb5() { put_value(get_value() | 0x20); }
        inline void smb6() { put_value(get_value() | 0x40); }
        inline void smb7() { put_value(get_value() | 0x80); }

        inline void rmb0() { put_value(get_value() & ~0x01); }
        inline void rmb1() { put_value(get_value() & ~0x02); }
        inline void rmb2() { put_value(get_value() & ~0x04); }
        inline void rmb3() { put_value(get_value() & ~0x08); }
        inline void rmb4() { put_value(get_value() & ~0x10); }
        inline void rmb5() { put_value(get_value() & ~0x20); }
        inline void rmb6() { put_value(get_value() & ~0x40); }
        inline void rmb7() { put_value(get_value() & ~0x80); }

    private:
        // Addressing modes
        void imp();
        void acc();
        void imm();
        void zp();
        void zpx();
        void zpy();
        void rel();
        void abso();
        void absx();
        void absy();
        void ind();
        void indx();
        void indy();
        void zprel();


    private:
        uint8_t read(uint16_t address);
        void write(uint16_t address, uint8_t value);

        // Flag helper functions
        constexpr void set_carry() { status |= FLAG_CARRY; }
        constexpr void clear_carry() { status &= (~FLAG_CARRY); }
        constexpr void set_zero() { status |= FLAG_ZERO; }
        constexpr void clear_zero() { status &= (~FLAG_ZERO); }
        constexpr void set_interrupt() { status |= FLAG_INTERRUPT; }
        constexpr void clear_interrupt() { status &= (~FLAG_INTERRUPT); }
        constexpr void set_decimal() { status |= FLAG_DECIMAL; }
        constexpr void clear_decimal() { status &= (~FLAG_DECIMAL); }
        constexpr void set_overflow() { status |= FLAG_OVERFLOW; }
        constexpr void clear_overflow() { status &= (~FLAG_OVERFLOW); }
        constexpr void set_sign() { status |= FLAG_SIGN; }
        constexpr void clear_sign() { status &= (~FLAG_SIGN); }

        // Value helper functions
        inline uint16_t get_value()
        {
            if (addrtable[opcode] == acc)
                return static_cast<uint16_t>(a);
            return static_cast<uint16_t>(read(ea));
        }

        __attribute__((unused)) inline uint16_t get_value16()
        {
            return static_cast<uint16_t>(read(ea)) | static_cast<uint16_t>(read(ea + 1) << 8);
        }

        inline void put_value(uint16_t val)
        {
            if (addrtable[opcode] == acc)
                a = static_cast<uint8_t>(val & 0x00FF);
            else
                write(ea, (val & 0x00FF));
        }

        // Generic helper functions
        constexpr uint8_t save_accum(uint16_t n)
        {
            return static_cast<uint8_t>(n & 0x00FF);
        }

        constexpr void zero_calc(uint16_t n)
        {
            if (n & 0x00FF) clear_zero();
            else set_zero();
        }

        constexpr void sign_calc(uint16_t n)
        {
            if (n & 0x0080) set_sign();
            else clear_sign();
        }

        constexpr void carry_calc(uint16_t n)
        {
            if (n & 0xFF00) set_carry();
            else clear_carry();
        }

        constexpr void overflow_calc(uint16_t n, uint16_t m, uint16_t o)
        {
            if ((n ^ m) & (n ^ 0) & 0x0080) set_overflow();
            else clear_overflow();
        }

        constexpr void push16(uint16_t val)
        {
            write(BASE_STACK + sp, (val >> 8) & 0xFF);
            write(BASE_STACK + ((sp - 1) & 0xFF), val & 0xFF);
            sp -= 2;
        }

        constexpr void push8(uint8_t val)
        {
            write(BASE_STACK + sp--, val);
        }

        constexpr uint16_t pull16()
        {
            uint16_t temp =
                read(BASE_STACK + ((sp + 1) & 0xFF) |
                static_cast<uint16_t>(BASE_STACK + ((sp + 2) & 0xFF) << 8));

            sp += 2;
            return temp;
        }

        constexpr uint8_t pull8()
        {
            return read(BASE_STACK + ++sp);
        }
    };
}

#endif
