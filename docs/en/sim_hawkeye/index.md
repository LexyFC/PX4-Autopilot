# Hawkeye Flight Visualiser

[Hawkeye](https://github.com/PX4/Hawkeye) is a real-time 3D flight visualiser for PX4.
It connects to any MAVLink source (SITL or hardware) and renders vehicle state from [`HIL_STATE_QUATERNION`](https://mavlink.io/en/messages/common.html#HIL_STATE_QUATERNION) messages.

::: tip
Hawkeye is the recommended visualiser for [SIH Simulation](../sim_sih/index.md) (which doesn't have a default visualiser).
:::

Note that Hawkeye is not a simulator — it doesn't model physics or generate sensor data — it only renders what PX4 is doing.

Hawkeye has no runtime dependencies.
It is built in in C with [Raylib](https://www.raylib.com/).

## Features

- Real-time 3D rendering of vehicle state via MAVLink
- Multi-vehicle swarm visualisation (up to 16 vehicles simultaneously)
- ULog replay with ghost mode for visual flight comparison (e.g. before/after PID tuning)
- Vehicle models: quadrotor, hexarotor, fixed-wing, tailsitter, VTOL, rover, ROV
- Three camera modes: Chase, FPV, Free (WASD fly)
- Orthographic views (sidebar and fullscreen)
- HUD with compass, attitude indicator, and telemetry badges
- Theme engine with built-in themes (Grid, Rez, Snow)

## Installation

### macOS (Homebrew)

```sh
brew tap PX4/px4
brew install PX4/px4/hawkeye
```

### Linux (Debian/Ubuntu)

Download the `.deb` package from the [latest release](https://github.com/PX4/Hawkeye/releases/latest):

```sh
sudo dpkg -i hawkeye_*_amd64.deb
```

### From Source (All Platforms)

```sh
git clone --recursive https://github.com/PX4/Hawkeye.git
cd Hawkeye
make release
```

The binary is placed at `build/hawkeye`.

On Linux, the following packages are required:

```sh
sudo apt install build-essential cmake libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libgl-dev
```

On Windows, use Visual Studio with the C/C++ workload and CMake.

## Usage with SIH SITL

### Single Vehicle

In one terminal, start SIH:

```sh
make px4_sitl_sih sihsim_quadx
```

In another terminal, start Hawkeye:

```sh
hawkeye
```

Hawkeye connects on UDP port **19410** by default — the same port SIH sends `HIL_STATE_QUATERNION` on.

### Multi-Vehicle

Each SIH instance sends visualisation data on port `19410 + N` (where N is the instance number).
Launch multiple SIH instances, then start Hawkeye with the vehicle count:

```sh
hawkeye -n 3
```

### Vehicle Type Selection

By default, Hawkeye renders a quadrotor.
Use flags to select a different model:

| Flag  | Vehicle               |
| ----- | --------------------- |
| `-mc` | Multicopter (default) |
| `-fw` | Fixed-wing            |
| `-ts` | Tailsitter            |

Example for fixed-wing:

```sh
hawkeye -fw
```

## ULog Replay

Hawkeye can replay flight logs for post-flight review:

```sh
hawkeye --replay flight.ulg
```

### Ghost Mode

Overlay a second flight at reduced opacity for visual comparison:

```sh
hawkeye --replay flight_after.ulg --ghost flight_before.ulg
```

This is useful for comparing flights before and after tuning changes.

## CLI Reference

| Option                      | Description                           |
| --------------------------- | ------------------------------------- |
| `-udp <port>`               | Set UDP listen port (default: 19410)  |
| `-n <count>`                | Number of vehicles to visualise       |
| `-origin <lat> <lon> <alt>` | Set map origin                        |
| `-mc`                       | Multicopter model                     |
| `-fw`                       | Fixed-wing model                      |
| `-ts`                       | Tailsitter model                      |
| `--replay <file.ulg>`       | Replay a ULog file                    |
| `--ghost <file.ulg>`        | Overlay a ghost flight for comparison |

## Camera Controls

| Mode  | Description                          |
| ----- | ------------------------------------ |
| Chase | Follows behind the vehicle (default) |
| FPV   | First-person view from the vehicle   |
| Free  | Fly freely with WASD + mouse         |

## See Also

- [Hawkeye GitHub Repository](https://github.com/PX4/Hawkeye)
- [SIH Simulation](../sim_sih/index.md) — the headless simulator Hawkeye is designed to visualise
