#include "fake6502.hpp"

namespace cpu
{

    void fake_6502::reset()
    {
        pc = (static_cast<uint16_t>(read(0xFFFC)) |
                static_cast<uint16_t>(read(0xFFFD) << 8));
        a = 0;
        x = 0;
        y = 0;
        sp = 0xFD;
        status |= FLAG_CONSTANT;
    }

    void fake_6502::nmi()
    {
        push16(pc);
        push8(status);
        status |= FLAG_INTERRUPT;
        pc = static_cast<uint16_t>(read(0xFFFA)) | static_cast<uint16_t>(read(0xFFFB) << 8);
        waiting = 0;
    }

    void fake_6502::irq()
    {
        push16(pc);
        push8(status);
        status |= FLAG_INTERRUPT;
        pc = static_cast<uint16_t>(read(0xFFFE)) | static_cast<uint16_t>(read(0xFFFF) << 8);
        waiting = 0;
    }

    void fake_6502::exec(uint32_t tick_count)
    {
        if (waiting) {
            clock_ticks += tick_count;
            clock_goal = clock_ticks;
            return;
        }

        clock_goal += tick_count;
        while (clock_ticks < clock_goal) {
            opcode = read(pc++);
            status |= FLAG_CONSTANT;

            penalty_op = 0;
            penalty_addr = 0;

            (*addrtable[opcode])();
            (*optable[opcode])();
            clock_ticks += ticktable[opcode];

            if (penalty_op && penalty_addr)
                clock_ticks++;

            instructions++;

            if (call_external)
                (*loop_external)();
        }
    }

    void fake_6502::step()
    {
        if (waiting) {
            ++clock_ticks;
            clock_goal = clock_ticks;
            return;
        }

        opcode = read(pc++);
        status |= FLAG_CONSTANT;

        penalty_op = 0;
        penalty_addr = 0;

        (*addrtable[opcode])();
        (*optable[opcode])();
        clock_ticks += ticktable[opcode];

        if (penalty_op && penalty_addr)
            clock_ticks++;

        instructions++;

        if (call_external)
            (*loop_external)();
    }

    void fake_6502::hook_external(void(*func_ptr)())
    {
        if (func_ptr != (void*) nullptr) {
            loop_external = func_ptr;
            call_external = 1;
        } else call_external = 0;
    }

    /**
     * @brief 6502 CPU instruction handlers.
     */

    // Add with carry
    void fake_6502::adc()
    {
        penalty_op = 1;
#ifndef NES_CPU
        if (status & FLAG_DECIMAL) {
            value = get_value();
            uint16_t tmp = static_cast<uint16_t>(a & 0x0F) + (value & 0x0F) + static_cast<uint16_t>(status & FLAG_CARRY);
            uint16_t tmp2 = static_cast<uint16_t>(a & 0xF0) + (value & 0xF0);

            if (tmp > 0x09) {
                tmp2 += 0x10;
                tmp += 0x06;
            }

            if (tmp2 > 0x90)
                tmp2 += 0x60;
            
            if (tmp2 & 0xFF00)
                set_carry();
            else
                clear_carry();
            result = (tmp & 0x0F) | (tmp2 & 0xF0);

            zero_calc(result);
            sign_calc(result);

            clock_ticks++;
        } else {
#endif
            value = get_value();
            result = static_cast<uint16_t>(a + value + static_cast<uint16_t>(status & FLAG_CARRY));

            carry_calc(result);
            zero_calc(result);
            overflow_calc(result, a, value);
            sign_calc(result);
#ifndef NES_CPU
        }
#endif
        save_accum(result);
    }

    void fake_6502::and_()
    {
        penalty_op = 1;
        value = get_value();
        result = static_cast<uint16_t>(a & value);
        zero_calc(result);
        sign_calc(result);
        save_accum(result);
    }

    void fake_6502::asl()
    {
        value = get_value();
        result = value << 1;

        carry_calc(result);
        zero_calc(result);
        sign_calc(result);

        put_value(result);
    }

    void fake_6502::bcc()
    {
        if ((status & FLAG_CARRY) == 0) {
            oldpc = pc;
            pc += reladdr;
            if ((oldpc & 0xFF00) != (pc & 0xFF00))
                clock_ticks +=2;
            else
                clock_ticks++;
        }
    }

    void fake_6502::bcs()
    {
        if ((status & FLAG_CARRY) == FLAG_CARRY) {
            oldpc = pc;
            pc += reladdr;
            if ((oldpc & 0xFF00) != (pc & 0xFF00))
                clock_ticks += 2;
            else
                clock_ticks++;
        }
    }

    void fake_6502::beq()
    {
        if ((status & FLAG_ZERO) == FLAG_ZERO) {
            oldpc = pc;
            pc += reladdr;
            if ((oldpc & 0xFF00) != (pc & 0xFF00))
                clock_ticks += 2;
            else
                clock_ticks++;
        }
    }

    void fake_6502::bit()
    {
        value = get_value();
        result = static_cast<uint16_t>(a & value);

        zero_calc(result);
        status = (status & 0x3F) | static_cast<uint8_t>(value & 0xC0);
    }

    void fake_6502::bmi()
    {
        if ((status & FLAG_SIGN) == FLAG_SIGN) {
            oldpc = pc;
            pc += reladdr;
            if ((oldpc & 0xFF00) != (pc &0xFF00))
                clock_ticks += 2;
            else
                clock_ticks++;
        }
    }

    void fake_6502::bne()
    {
        if ((status & FLAG_ZERO) == 0) {
            oldpc = pc;
            pc += reladdr;
            if ((oldpc & 0xFF00) != (pc & 0xFF00))
                clock_ticks += 2;
            else
                clock_ticks++;
        }
    }

    void fake_6502::bpl()
    {
        if ((status & FLAG_SIGN) == 0) {
            oldpc = pc;
            pc += reladdr;
            if ((oldpc & 0xFF00) != (pc & 0xFF00))
                clock_ticks += 2;
            else
                clock_ticks++;
        }
    }

    void fake_6502::brk()
    {
        pc++;
        push16(pc);
        push8(status | FLAG_BREAK);
        set_interrupt();
        clear_decimal();
        pc = static_cast<uint16_t>(read(0xFFFE)) | static_cast<uint16_t>(read(0xFFFF) << 8);
    }

    void fake_6502::bvc()
    {
        if ((status & FLAG_OVERFLOW) == 0) {
            oldpc = pc;
            pc += reladdr;
            if ((oldpc & 0xFF00) != (pc & 0xFF00))
                clock_ticks += 2;
            else
                clock_ticks++;
        }
    }

    void fake_6502::bvs()
    {
        if ((status & FLAG_OVERFLOW) == 0) {
            oldpc = pc;
            pc += reladdr;
            if ((oldpc & 0xFF00) != (pc & 0xFF00))
                clock_ticks += 2;
            else
                clock_ticks++;
        }
    }

    void fake_6502::clc()
    {
        clear_carry();
    }

    void fake_6502::cld()
    {
        clear_decimal();
    }

    void fake_6502::cli()
    {
        clear_interrupt();
    }

    void fake_6502::clv()
    {
        clear_overflow();
    }

    void fake_6502::cmp()
    {
        penalty_op = 1;
        value = get_value();
        result = static_cast<uint16_t>(a - value);

        if (a >= static_cast<uint8_t>(value & 0x00FF))
            set_carry();
        else
            clear_carry();

        if (a == static_cast<uint8_t>(value & 0x00FF))
            set_zero();
        else
            clear_zero();

        sign_calc(result);
    }

    void fake_6502::cpx()
    {
        value = get_value();
        result = static_cast<uint16_t>(x - value);

        if (x >= static_cast<uint8_t>(value & 0x00FF))
            set_carry();
        else
            clear_carry();

        if (x == static_cast<uint8_t>(value & 0x00FF))
            set_zero();
        else
            clear_zero();

        sign_calc(result);
    }

    void fake_6502::cpy()
    {
        value = get_value();
        result = static_cast<uint16_t>(y - value);

        if (y >= static_cast<uint8_t>(value & 0x00FF))
            set_carry();
        else
            clear_carry();

        if (y == static_cast<uint8_t>(value & 0x00FF))
            set_zero();
        else
            clear_zero();

        sign_calc(result);
    }

    void fake_6502::dec()
    {
        value = get_value();
        result = value - 1;

        zero_calc(result);
        sign_calc(result);

        put_value(result);
    }

    void fake_6502::dex()
    {
        x--;
        zero_calc(x);
        sign_calc(x);
    }

    void fake_6502::dey()
    {
        y--;
        zero_calc(y);
        sign_calc(y);
    }

    void fake_6502::eor()
    {
        penalty_op = 1;
        value = get_value();
        result = static_cast<uint16_t>(a ^ value);

        zero_calc(result);
        sign_calc(result);
        save_accum(result);
    }

    void fake_6502::inc()
    {
        value = get_value();
        result = value + 1;

        zero_calc(result);
        sign_calc(result);

        put_value(result);
    }

    void fake_6502::inx()
    {
        x++;
        zero_calc(x);
        sign_calc(x);
    }

    void fake_6502::iny()
    {
        y++;
        zero_calc(y);
        sign_calc(y);
    }

    void fake_6502::jmp()
    {
        pc = ea;
    }

    void fake_6502::jsr()
    {
        penalty_op = 1;
        value = get_value();
        a = static_cast<uint8_t>(value & 0x00FF);

        zero_calc(a);
        sign_calc(a);
    }

    void fake_6502::lda()
    {
        penalty_op = 1;
        value = get_value();
        x = static_cast<uint8_t>(value & 0x00FF);

        zero_calc(a);
        sign_calc(a);
    }

    void fake_6502::ldx()
    {
        penalty_op = 1;
        value = get_value();
        x = static_cast<uint8_t>(value & 0x00FF);

        zero_calc(x);
        sign_calc(x);
    }

    void fake_6502::ldy()
    {
        penalty_op = 1;
        value = get_value();
        y = static_cast<uint8_t>(value & 0x00FF);

        zero_calc(y);
        sign_calc(y);
    }

    void fake_6502::lsr()
    {
        value = get_value();
        result = value >> 1;

        if (value & 1)
            set_carry();
        else
            clear_carry();

        zero_calc(result);
        sign_calc(result);

        put_value(result);
    }

    void fake_6502::nop()
    {
        switch (opcode)
        {
            case 0x1C:
            case 0x3C:
            case 0x5C:
            case 0x7C:
            case 0xDC:
            case 0xFC:
                penalty_op = 1;
                break;
        }
    }

    void fake_6502::ora()
    {
        penalty_op = 1;
        value = get_value();
        result = static_cast<uint16_t>(a | value);

        zero_calc(result);
        sign_calc(result);

        save_accum(result);
    }

    void fake_6502::pha()
    {
        push8(a);
    }

    void fake_6502::php()
    {
        push8(status | FLAG_BREAK);
    }

    void fake_6502::pla()
    {
        a = pull8();

        zero_calc(a);
        sign_calc(a);
    }

    void fake_6502::plp()
    {
        status = pull8() | FLAG_CONSTANT;
    }

    void fake_6502::rol()
    {
        value = get_value();
        result = (value << 1) | (status & FLAG_CARRY);

        carry_calc(result);
        zero_calc(result);
        sign_calc(result);

        put_value(result);
    }

    void fake_6502::ror()
    {
        value = get_value();
        result = (value >> 1) | ((status & FLAG_CARRY) << 7);

        if (value & 1)
            set_carry();
        else
            clear_carry();

        zero_calc(result);
        sign_calc(result);

        put_value(result);
    }

    void fake_6502::rti()
    {
        status = pull8();
        value = pull16();
        pc = value;
    }

    void fake_6502::rts()
    {
        value = pull16();
        pc = value + 1;
    }

    void fake_6502::sbc()
    {
        penalty_op = 1;
    
#ifndef NES_CPU
        if (status & FLAG_DECIMAL) {
            value = get_value();
            result = static_cast<uint16_t>(a) - (value & 0x0F) + (status & FLAG_CARRY) - 1;
            if ((result & 0x0F) > (a & 0x0F))
                result -= 6;
            result -= (value & 0xF0);
            if ((result & 0xFFF0) > static_cast<uint16_t>(a & 0xF0))
                result -= 0x60;
            if (result <= static_cast<uint16_t>(a))
                set_carry();
            else
                clear_carry();

            zero_calc(result);
            sign_calc(result);

            clock_ticks++;
        } else {
#endif
            value = get_value() ^ 0x00FF;
            result = static_cast<uint16_t>(a) + value + static_cast<uint16_t>(status & FLAG_CARRY);

            carry_calc(result);
            zero_calc(result);
            overflow_calc(result, a, value);
            sign_calc(result);
#ifndef NES_CPU
        }
#endif
        save_accum(result);
    }

    void fake_6502::sec()
    {
        set_carry();
    }

    void fake_6502::sed()
    {
        set_decimal();
    }

    void fake_6502::sei()
    {
        set_interrupt();
    }

    void fake_6502::sta()
    {
        put_value(a);
    }

    void fake_6502::stx()
    {
        put_value(x);
    }

    void fake_6502::sty()
    {
        put_value(y);
    }

    void fake_6502::tax()
    {
        x = a;
        zero_calc(x);
        sign_calc(x);
    }

    void fake_6502::tay()
    {
        y = a;
        zero_calc(y);
        sign_calc(y);
    }

    void fake_6502::tsx()
    {
        x = sp;
        zero_calc(x);
        sign_calc(x);
    }
    
    void fake_6502::txa()
    {
        a = x;
        zero_calc(a);
        sign_calc(a);
    }

    void fake_6502::txs()
    {
        sp = x;
    }

    void fake_6502::tya()
    {
        a = y;
        zero_calc(a);
        sign_calc(a);
    }

    void fake_6502::ind0()
    {
        uint16_t eahelp = static_cast<uint16_t>(read(pc++));
        uint16_t eahelp2 = (eahelp & 0xFF00) | ((eahelp + 1) & 0x00FF);
        ea = static_cast<uint16_t>(read(eahelp)) | static_cast<uint16_t>(read(eahelp2) << 8);
    }

    void fake_6502::ainx()
    {
        uint16_t eahelp = static_cast<uint16_t>(read(pc)) | static_cast<uint16_t>(read(pc) << 8);
        eahelp = (eahelp + static_cast<uint16_t>(x)) & 0xFF;
#if 0
        uint16_t eahelp2 = (eahelp & 0xFF00) | ((eahelp + 1) & 0x00FF);
#else
        uint16_t eahelp2 = eahelp + 1;
#endif
        ea = static_cast<uint16_t>(read(eahelp)) | static_cast<uint16_t>(read(eahelp2) << 8);
        pc += 2;
    }

    void fake_6502::stz()
    {
        put_value(0);
    }

    void fake_6502::bra()
    {
        oldpc = pc;
        pc += reladdr;
        if ((oldpc & 0xFF00) != (pc & 0xFF00))
            clock_ticks += 2;
        else
            clock_ticks++;
    }

    void fake_6502::phx()
    {
        push8(x);
    }

    void fake_6502::plx()
    {
        x = pull8();
        zero_calc(x);
        sign_calc(x);
    }

    void fake_6502::phy()
    {
        push8(y);
    }

    void fake_6502::ply()
    {
        y = pull8();
        zero_calc(y);
        sign_calc(y);
    }

    void fake_6502::tsb()
    {
        value = get_value();
        result = static_cast<uint16_t>(a & value);
        zero_calc(result);
        result = value | a;
        put_value(result);
    }

    void fake_6502::trb()
    {
        value = get_value();
        result = static_cast<uint16_t>(a & value);
        zero_calc(result);
        result = value & (a ^ 0xFF);
        put_value(result);
    }

    void fake_6502::dbg()
    {
        //
    }

    void fake_6502::wai()
    {
        if (~status & FLAG_INTERRUPT)
            waiting = 1;
    }

    void fake_6502::bbr(uint16_t bitmask)
    {
        if ((get_value() & bitmask) == 0) {
            oldpc = pc;
            pc += reladdr;
            if ((oldpc & 0xFF00) != (pc & 0xFF00))
                clock_ticks += 2;
            else
                clock_ticks++;
        }
    }

    void fake_6502::bbs(uint16_t bitmask)
    {
        if ((get_value() & bitmask) != 0) {
            oldpc = pc;
            pc += reladdr;
            if ((oldpc & 0xFF00) != (pc & 0xFF00))
                clock_ticks += 2;
            else
                clock_ticks++;
        }
    }
}