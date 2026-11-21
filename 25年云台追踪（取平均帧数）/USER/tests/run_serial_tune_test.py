"""Run the serial tuning regression in Cortex-M emulation, without hardware."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'build' / 'test_deps'))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_MCLASS, UC_MODE_THUMB, UC_HOOK_MEM_INVALID
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0
from unicorn.arm_const import UC_ARM_REG_XPSR, UC_ARM_REG_PC

cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
cpu.mem_map(0x08000000, 0x100000)
cpu.mem_map(0x20000000, 0x20000)
cpu.mem_map(0x40000000, 0x30000)  # USART register simulation
with open(sys.argv[1], 'rb') as binary:
    elf = ELFFile(binary)
    entry = elf.header['e_entry']
    for segment in elf.iter_segments():
        if segment['p_type'] == 'PT_LOAD' and segment['p_filesz']:
            cpu.mem_write(segment['p_vaddr'], segment.data())
    # ARMCC scatter loading normally copies initialized RAM from flash at reset.
    for section in elf.iter_sections():
        if section['sh_type'] == 'SHT_PROGBITS' and section['sh_flags'] & 2:
            address = section['sh_addr']
            if 0x20000000 <= address < 0x20020000:
                cpu.mem_write(address, section.data())
stop = 0x080FF000
cpu.reg_write(UC_ARM_REG_SP, 0x2001FFF0)
cpu.reg_write(UC_ARM_REG_LR, stop | 1)
cpu.reg_write(UC_ARM_REG_XPSR, 0x01000000)
def invalid_memory(uc, access, address, size, value, data):
    print(f'Invalid access at PC={uc.reg_read(UC_ARM_REG_PC):#x}, address={address:#x}')
    return False

cpu.hook_add(UC_HOOK_MEM_INVALID, invalid_memory)
cpu.emu_start(entry | 1, stop, count=20_000_000)
if cpu.reg_read(UC_ARM_REG_PC) != stop:
    raise RuntimeError('Test timed out or reached an unexpected peripheral trap')
result = cpu.reg_read(UC_ARM_REG_R0)
if result:
    with open(sys.argv[1], 'rb') as binary:
        symbols = ELFFile(binary).get_section_by_name('.symtab')
        import struct
        for name in ('tune', 'command_result'):
            symbol = symbols.get_symbol_by_name(name)[0]
            size = 40 if name == 'tune' else 4
            data = cpu.mem_read(symbol['st_value'], size)
            print(name, struct.unpack('<10f' if size == 40 else '<i', data))
    raise RuntimeError(f'Firmware CHECK failed: {result}')
print('PASS: command validation, fragmented commands, overflow recovery, runtime motor limits, '
      'camera timeout, mode exit, 14-channel CSV, nonblocking TX, USART IRQs, '
      'optional X gain scheduling/damping, legacy equivalence, no derivative kick, '
      'early braking, center-change and camera-loss resets, online 1/2/3-frame '
      'averaging, history reset, partial-frame preservation, filter timeout, independent XY PD, '
      'zero-D equivalence, online D changes, derivative history resets, scheduled XY PD, endpoint gains, smooth transition, threshold validation and legacy fallback')
