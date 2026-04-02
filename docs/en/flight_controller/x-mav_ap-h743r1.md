# AP-H743-R1 Flight Controller

<Badge type="tip" text="PX4 v1.17" />

::: warning
PX4 does not manufacture this (or any) autopilot.
:::

The AP-H743-R1 is an advanced autopilot manufactured by X-MAV<sup>&reg;</sup>.

The autopilot is recommended for commercial system integration, but is also suitable for academic research and any other applications.
It brings you ultimate performance, stability, and reliability in every aspect.

![AP-H743-R1](../../assets/flight_controller/x-mav_ap-h743r1/ap-h743r1-main.png)

::: info
These flight controllers are [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

### Processors & Sensors

- FMU Processor: STM32H743VIT6
  - 32 Bit Arm® Cortex®-M7, 480MHz, 2MB flash memory, 1MB RAM
- IO Processor: STM32F103
  - 32 Bit Arm® Cortex®-M3, 72MHz, 20KB SRAM
- On-board sensors
  - Accel/Gyro: ICM-42688-P\*2(Version1), BMI270\*2(Version2)
  - Mag: QMC5883P
  - Barometer: DPS310(Version1),SPL06(Version2)

### Interfaces

- 15x PWM Servo Outputs
- 1x Dedicated S.Bus Input
- 3x TELEM Ports
- 1x SERIAL4 Port
- 2x GPS Ports
- 1x USB Port (TYPE-C)
- 3x I2C Bus Ports
- 2x CAN Ports
- 2x Power Input Ports
  - ADC Power Input
  - DroneCAN/UAVCAN Power Input
- 2x Dedicated Debug Port
  - FMU Debug
  - IO Debug

## Purchase Channels {#store}

Order from [X-MAV](https://www.x-mav.cn/).

## Radio Control

A Radio Control (RC) system is required if you want to manually control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to select a compatible transmitter/receiver and then bind them so that they communicate (read the instructions that come with your specific transmitter/receiver).

SBUS receivers connect to the SBUS-IN input port.
CRSF receiver must be wired to a spare port (UART) on the Flight Controller. Then you can bind the transmitter and receiver together.

## Serial Port Mapping

| UART   | Device     | Port    |
| ------ | ---------- | ------- |
| USART1 | /dev/ttyS0 | GPS     |
| USART2 | /dev/ttyS1 | GPS2    |
| USART3 | /dev/ttyS2 | TELEM1  |
| UART4  | /dev/ttyS3 | TELEM2  |
| UART7  | /dev/ttyS4 | TELEM3  |
| UART8  | /dev/ttyS5 | SERIAL4 |

## PWM Outputs {#pwm_outputs}

This flight controller supports up to 7 FMU PWM outputs (AUX) and 8 IO PWM outputs (MAIN).

All FMU outputs support [DShot](../peripherals/dshot.md) and [Bidirectional DShot](../peripherals/dshot.md#bidirectional-dshot-telemetry).

The 7 outputs are in 3 groups:

- Outputs 1-4 in group1 (Timer1)
- Output 7 in group2 (Timer2)
- Outputs 5-6 in group3 (Timer3)

All outputs within the same group must use the same output protocol and rate.

## Battery Monitoring

The board has connectors for 2 power monitors.

- POWER1 -- ADC
- POWER2 -- DroneCAN

The board is configure by default for a analog power monitor, and also has DroneCAN power monitor configured which is enabled.

## Building Firmware

To [build PX4](../dev_setup/building_px4.md) for this target, execute:

```sh
make x-mav_ap-h743r1_default
```

## Pinouts and Size

![AP-H743-R1 pinouts](../../assets/flight_controller/x-mav_ap-h743r1/ap-h743r1-pinouts.png)

![AP-H743-R1](../../assets/flight_controller/x-mav_ap-h743r1/ap-h743r1-size.png)

## Supported Platforms / Airframes

Any multirotor/airplane/rover or boat that can be controlled using normal RC servos or Futaba S-Bus servos.
The complete set of supported configurations can be found in the [Airframe Reference](../airframes/airframe_reference.md).

## Debug Port {#debug_port}

### SWD

The [SWD interface](../debug/swd_debug.md) operate on the **FMU-DEBUG** port (`FMU-DEBUG`).

The debug port (`FMU-DEBUG`) uses a [JST SM04B-GHS-TB](https://www.digikey.com/en/products/detail/jst-sales-america-inc/SM04B-GHS-TB/807788) connector and has the following pinout:

| Pin     | Signal    | Volt  |
| ------- | --------- | ----- |
| 1 (red) | 5V+       | +5V   |
| 2 (blk) | FMU_SWDIO | +3.3V |
| 3 (blk) | FMU_SWCLK | +3.3V |
| 4 (blk) | GND       | GND   |
