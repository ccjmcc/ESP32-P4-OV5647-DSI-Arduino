# ESP32-P4 OV5647 DSI Live Preview (Arduino)

简洁参考示例：在 Arduino IDE 下，把 `OV5647` 实时画面显示到 `10.1-inch DSI` 屏幕。  
A compact reference sketch for showing live `OV5647` camera preview on a `10.1-inch DSI` display in Arduino IDE.

## Tested Hardware

- 开发板 / Board: `Waveshare ESP32-P4-NANO-D`
- 屏幕 / Display: `10.1-inch DSI T-A`
- 摄像头 / Camera: `OV5647 (Raspberry Pi Camera Module v2 style)`

## Features

- `OV5647` chip ID detection
- `MIPI CSI` capture
- `ISP` conversion: `RAW8 -> RGB565`
- `DSI` live preview
- Triple frame buffer in PSRAM

## Files

- `ESP32-P4-OV5647-DSI-Arduino.ino`: main sketch
- `ov5647_regs.h`: OV5647 init register table

## Dependencies

- Arduino core for ESP32: `3.3.7`
- Library: `GFX Library for Arduino`
- Helper library: `displays`

说明 / Note:

This sketch expects the local `displays` helper library used by the 10.1-inch DSI touch test to be installed in your Arduino libraries folder.

## Arduino IDE Settings

- Board: `ESP32P4 Dev Module`
- Port: your serial port, for example `COM9`
- Chip Variant: `Before v3.00`
- PSRAM: `Enabled`
- Upload Mode: `UART0 / Hardware CDC`

## Build and Upload

1. Install the dependencies.
2. Open the sketch folder in Arduino IDE.
3. Apply the board settings above.
4. Upload the sketch.
5. Close Serial Monitor if the upload port is busy.

## Expected Serial Output

```text
[BOOT] ESP32-P4 OV5647 live preview
[OV5647] chip-id: 0x5647
[CAM] preview pipeline ready
[STAT] captured=16 drawn=16 ready=0
```
