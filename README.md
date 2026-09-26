# IRMS-Firmware

ESP32 firmware for the IRMS knee-rehabilitation sensor (two MPU6050 IMUs, BLE). Split out of
[`yuhina0515/IRMS`](https://github.com/yuhina0515/IRMS) (`IRMS_Sensor/`, history preserved)
so that firmware releases reach devices automatically.

## How a release reaches a device

1. Bump `IRMS_FW_VERSION` in `IRMS_Sensor/config.h`, commit, then push an annotated tag with
   the same version: `git tag -a v1.1.0 -m "what changed" && git push origin v1.1.0`.
2. `release.yml` refuses a tag that differs from `IRMS_FW_VERSION`, compiles with the pinned
   ESP32 Arduino core (`esp32:esp32@3.0.7`, FQBN `esp32:esp32:esp32`, default partitions),
   writes `manifest.json` (version, size, SHA-256, MD5, notes, optional `minAppVersion`) and
   signs it with Ed25519 (`FIRMWARE_SIGNING_KEY` secret).
3. The IRMS app (1.2.0-beta.13+) checks `releases/latest` after connecting and after each
   session. It verifies the signature with its compiled-in public key, verifies the image
   hashes, and pushes the update over BLE OTA **only when idle** (no session, no hardware
   error). A running session defers the update until it ends.

## Channels

Same rule as the app's own updates, following the app's 「接收 Beta 版更新」 setting:

- **Stable**: tag `vX.Y.Z` (and `IRMS_FW_VERSION` `X.Y.Z`). Apps on the stable channel read `releases/latest`.
- **Beta**: tag `vX.Y.Z-beta.N` (and `IRMS_FW_VERSION` `X.Y.Z-beta.N`). Published as a GitHub prerelease; apps on the beta channel read the `beta-latest` pointer release, which every tag refreshes. A stable-channel app refuses a pre-release manifest even if it is served to it.

## Keys

- Private key: `IRMS_secrets/irms_firmware_ed25519.pem` (outside every repo) and this repo's
  `FIRMWARE_SIGNING_KEY` Actions secret.
- Public key (raw, base64): `9inYbAyGpSC4KbNtu1+tT/I5B/jrMDuwmfH3SsS1u/8=` — embedded in
  `IRMS_App_Tauri/src-tauri/src/firmware_update.rs`. Rotating it requires an app release first.

## Optional repository variable

`MIN_APP_VERSION` — set when a firmware change needs a newer app (for example a protocol
change); older apps will then report the update as incompatible instead of installing it.

## Local build

```
arduino-cli core install esp32:esp32@3.0.7
arduino-cli compile --fqbn esp32:esp32:esp32 --output-dir build IRMS_Sensor
arduino-cli upload -p COM7 --fqbn esp32:esp32:esp32 IRMS_Sensor   # USB flashing
```

The BLE protocol section of `config.h` is a contract with the app (`shared/protocol.ts`,
`src-tauri/src/protocol.rs`); do not change it on one side only.
