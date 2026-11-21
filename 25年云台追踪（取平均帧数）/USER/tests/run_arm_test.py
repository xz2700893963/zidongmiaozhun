"""Run an ARM ELF whose test_main returns zero on success.

Requires unicorn and pyelftools. The binary is built with Cortex-M4 SoftVFP,
test_main as entry, flash at 0x08000000, and RAM at 0x20000000. Unused peripheral
functions must be linked to trap stubs; only motor transmissions are mocked.
"""
import sys
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_MCLASS, UC_MODE_THUMB
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0
from unicorn.arm_const import UC_ARM_REG_XPSR, UC_ARM_REG_PC


def run(path):
    cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    cpu.mem_map(0x08000000, 0x100000)
    cpu.mem_map(0x20000000, 0x20000)
    with open(path, "rb") as binary:
        elf = ELFFile(binary)
        entry = elf.header["e_entry"]
        for segment in elf.iter_segments():
            if segment["p_type"] == "PT_LOAD" and segment["p_filesz"]:
                cpu.mem_write(segment["p_vaddr"], segment.data())

    stop = 0x080FF000
    cpu.reg_write(UC_ARM_REG_SP, 0x2001FFF0)
    cpu.reg_write(UC_ARM_REG_LR, stop | 1)
    cpu.reg_write(UC_ARM_REG_XPSR, 0x01000000)
    cpu.emu_start(entry | 1, stop, count=5_000_000)
    if cpu.reg_read(UC_ARM_REG_PC) != stop:
        raise RuntimeError("Test did not return before the instruction limit")
    result = cpu.reg_read(UC_ARM_REG_R0)
    if result:
        raise RuntimeError(f"Firmware CHECK failed: {result}")
    print("PASS: frame averaging, recovery, wraparound, frame gating, deadband, "
          "speed limits, slew, reversal, timeout stop, and mode exit")


if __name__ == "__main__":
    run(sys.argv[1])
