# CAN Laboratory — NUCLEO‑F446ZE

This lab teaches basic CAN setup and messaging on the NUCLEO‑F446ZE using STM32 HAL. Students will primarily work in the `.ioc` (CubeMX) to configure peripherals, then run a minimal demo that sends ASCII "HELLO" over CAN and prints any received text over `USART3` (ST‑LINK VCP).

## Project Overview
- Purpose: Practice configuring CAN in CubeMX, sending/receiving frames, and verifying on UART.
- Hardware: `NUCLEO‑F446ZE` (STM32F446ZETx). For real bus tests, add CAN transceivers and proper termination.
- Demo behavior:
   - Transmit: Sends "HELLO" once per second using StdID `0x123`.
   - Receive: Prints RX ID/DLC and ASCII payload via `USART3` and toggles `LD1`.
   - Loopback mode supports single‑board testing (no transceiver required).

## Quick Start (Loopback)
1. Open the project in STM32CubeIDE (File → Open Projects from File System → select repo).
2. Open `CAN_laboratory.ioc` (double‑click). If it opens as text, see Troubleshooting below.
3. In CubeMX view, set:
    - CAN1 → Mode: `Loopback`
    - Pins: `PD0` (CAN1_RX, AF9), `PD1` (CAN1_TX, AF9)
    - Bit timing (example 500 kbit/s): Prescaler `4`, SJW `1 TQ`, BS1 `13 TQ`, BS2 `7 TQ`
    - USART3: `PD8` (TX) and `PD9` (RX), Baud `115200`, 8N1 (ST‑LINK VCP)
4. Save and click "Generate Code".
5. Build and flash the board (hammer → play).
6. Open a serial terminal on the ST‑LINK COM port at `115200`. You should see:
    - `CAN demo ready.`
    - Each second: `RX id=0x123 dlc=5 data=HELLO` (and `LD1` toggles).

## Resetting `.ioc` to Default State
If you restart the `.ioc` or create a new STM32 Project, re‑apply these minimal settings:
- RCC/Clock: Keep defaults (168 MHz SYSCLK, APB1 at 42 MHz).
- GPIO: Leave LEDs/user button as default (optional).
- USART3: Enable at 115200 8N1, `PD8/PD9` AF7.
- CAN1:
   - Mode: `Loopback` (for single‑board lab) or `Normal` (for real bus).
   - Pins: `PD0` = RX (AF9), `PD1` = TX (AF9).
   - Bit timing (500 kbit/s example): Prescaler `4`, SJW `1`, BS1 `13`, BS2 `7`.
   - NVIC: RX interrupts optional (demo uses polling).
- Generate Code to update `Core/Src/*`.

## Running on Real CAN Bus
1. Change CAN1 Mode to `Normal` in `.ioc` and regenerate.
2. Hardware: Each board needs a CAN transceiver (e.g., TJA1050/SN65HVD230). Wire `CANH`↔`CANH`, `CANL`↔`CANL`.
3. Termination: 120 Ω at both ends of the bus.
4. Matching timing: Ensure both nodes use identical bit timing.
5. Flash both boards; one node will send "HELLO" and the other will print the RX frame.

## Editing IDs and Payload
- Change StdID: `Core/Src/main.c` → `txHeader.StdId = 0x123;`
- Change payload: `Core/Src/main.c` → `CAN_SendText("HELLO");`
- Extended ID: set `txHeader.IDE = CAN_ID_EXT` and adjust filters as needed.

## Suggested Lab Exercises
- Filters: Configure accept‑only filter for a specific StdID (e.g., `0x321`) and verify behavior.
- Payload framing: Send longer text (e.g., "Hello, World!") and observe multi‑frame chunks (≤8 bytes/frame).
- Echo node: On RX, re‑transmit payload using a different ID to create ping‑pong between boards.
- Button‑triggered TX: Send when `USER_Btn` is pressed; debounce in software.

## Troubleshooting `.ioc` Opening
- In CubeIDE, if `.ioc` shows as text:
   - Try right‑click → Open With → CubeMX Editor.
   - Ensure STM32 firmware repository is set: Window → Preferences → STM32Cube → Firmware Repository → e.g., `C:\Users\<you>\STM32Cube\Repository`.
   - If the CubeMX editor still isn’t available, install packs via CubeMX standalone.
- Install packs via STM32CubeMX:
   - Help → Manage embedded software packages → Install `STM32F4`.
   - Then open `CAN_laboratory.ioc` and Generate Code.
- Manual pack install (fallback):
   - Download `STM32Cube_FW_F4_V1.xx.x.zip` from ST.
   - Extract into `C:\Users\<you>\STM32Cube\Repository\STM32Cube_FW_F4_V1.xx.x`.
   - Point CubeIDE/CubeMX to this repository path.

## Notes
- Polling RX is used for simplicity; NVIC/interrupt RX can be added later.
- In Normal mode, a single board without a second node/transceiver/termination won’t ACK TX; use Loopback for solo tests.
- This repo already includes HAL/CMSIS Drivers and startup/linker files for F446.
