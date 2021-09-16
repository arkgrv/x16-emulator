// Commander X16 Emulator
// Copyright (c) 2021 Simone Rolando
// All rights reserved. License: 2-clause BSD

#ifndef _FAKE_6502_HPP
#define _FAKE_6502_HPP

#include <cstdint>
#include <cstdio>
#include <iostream>
#include <functional>
#include <variant>

#include "modes.h"

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

        // Tables
        void (*addrtable[256])();
        void (*optable[256])();
        uint32_t ticktable[256];

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
