// rf_switch.h - Tab5 INT/EXT antenna RF switch (PI4IOE5V6408 expander, internal
// I2C @0x43, pin P0: LOW = internal antenna, HIGH = external MMCX).
//
// REGISTER MAP WARNING (learned the hard way): the PI4IOE5V6408 is NOT a
// PCA9536-style part. Its registers are:
//   0x01 Chip ID / Control  -- bit0 = SOFTWARE RESET. NEVER write this.
//   0x03 I/O Direction      -- bit = 1 -> output
//   0x05 Output State       -- the actual output register
//   0x07 Output Hi-Z        -- bit = 1 -> pin Hi-Z (POWER-ON DEFAULT). Clear
//                              the bit to actually drive the pin.
// Writing "output" to 0x01 (as the original antenna sketch did) soft-resets
// the expander; other Tab5 rails hang off this chip, so the display goes
// black until a power cycle. All ops below are read-modify-write on the
// correct registers and never touch 0x01.

#pragma once
#include <M5Unified.h>

namespace rf_switch {

static const uint8_t  ADDR     = 0x43;
static const uint8_t  REG_DIR  = 0x03;   // 1 = output
static const uint8_t  REG_OUT  = 0x05;   // output state
static const uint8_t  REG_HIZ  = 0x07;   // 1 = Hi-Z (default!)
static const uint8_t  PIN      = 0;
static const uint32_t FREQ     = 100000;

inline void init() {
    uint8_t dir = M5.In_I2C.readRegister8(ADDR, REG_DIR, FREQ);
    M5.In_I2C.writeRegister8(ADDR, REG_DIR, dir | (1u << PIN), FREQ);   // P0 output
    uint8_t hiz = M5.In_I2C.readRegister8(ADDR, REG_HIZ, FREQ);
    M5.In_I2C.writeRegister8(ADDR, REG_HIZ, hiz & ~(1u << PIN), FREQ);  // drive P0
}

// ext=false -> internal antenna, ext=true -> external MMCX.
inline bool setExternal(bool ext) {
    uint8_t out = M5.In_I2C.readRegister8(ADDR, REG_OUT, FREQ);
    if (ext) out |=  (1u << PIN);
    else     out &= ~(1u << PIN);
    return M5.In_I2C.writeRegister8(ADDR, REG_OUT, out, FREQ);
}

} // namespace rf_switch
