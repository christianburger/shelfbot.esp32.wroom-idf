# VL53L1 migration analysis (serial log ↔ code)

## What the logs prove

1. **VL53L0X path is healthy end-to-end**:
   - `configure()`, `init()`, `setup()`, `calibrate()`, and `check()` all complete.
   - Repeated reads continue after init (`Measurement complete: 8190 mm valid=0 status=0`).
2. **VL53L1 path fails at first identity read in `setup()`**:
   - Failure appears immediately at `FAILED to read model ID: ESP_ERR_INVALID_STATE`.
   - This matches `setup()` failing on `readReg8(Reg::IDENTIFICATION_MODEL_ID, &model_id)`.

## Code findings mapped to failure

### 1) `vl53l1_driver` is largely a copy of `vl53l0x_driver`
The register map and init sequence in `components/vl53l1_driver/vl53l1.cpp` match the VL53L0X-oriented implementation pattern (same 8-bit register addresses and same style sequence), which is a strong signal this is not a native VL53L1 register layer.

### 2) Register access API is 8-bit addressed
In `components/vl53l1_driver/include/vl53l1.hpp`, register helpers all take `uint8_t reg` (`readReg8`, `writeReg8`, `readMulti`, etc.).

This design is consistent with the VL53L0X driver style, but VL53L1-class devices are typically handled with a 16-bit register address space in vendor APIs.

### 3) Setup fails exactly where expected
`components/vl53l1_driver/vl53l1.cpp`:
- `setup()` starts by reading model ID via `readReg8(Reg::IDENTIFICATION_MODEL_ID, &model_id)`.
- If I2C transaction fails, it returns `"Model ID read failed"`.

This exactly matches your second log sequence (`I2C hardware timeout detected` then `FAILED to read model ID`).

### 4) I2C frequency differs from working VL53L0X path
- `vl53l0x.hpp` uses 400 kHz.
- `vl53l1.hpp` uses 40 kHz.

This mismatch alone does not prove failure, but it is another migration difference and should be intentionally justified.

## About `embvm-drivers/ST-VL53L1X`
I could not fetch the repository from this environment due outbound network restrictions (`curl ... raw.githubusercontent.com ...` returned `CONNECT tunnel failed, response 403`), so I cannot *prove* its current reliability from direct source review here.

Given that limitation, the strongest statement is:
- I cannot certify that repository from this environment.
- Independently of that repository, your local `vl53l1_driver` implementation has structural red flags (8-bit register addressing and VL53L0X-like sequence), and those align with the observed model-ID read failure.

## Practical recommendation

1. Treat current `components/vl53l1_driver` as **not production-ready** for VL53L1 hardware.
2. Port to a known VL53L1 register model with 16-bit register addressing end-to-end.
3. Keep your existing `TofSensor` lifecycle (`configure/init/setup/calibrate/check`) but replace low-level register ops.
4. Add a focused preflight step before full setup:
   - probe 0x29
   - read **VL53L1 expected model ID register(s)** using 16-bit register transaction
   - fail early with raw bus error/status and register dump.
