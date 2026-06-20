# Lunar Lander

## Description

A C++ lunar-lander simulation and reinforcement-learning sandbox built around:

- `SFML` for windowing and drawing
- `Bullet` for rigid-body physics
- `Box2D` helpers used by the utility layer
- `OpenCV` for video capture / frame output
- `LibTorch` for the DQN agent
- `GLAD`, `GLFW`, and `GLM` for OpenGL support

The default program flow in `src/main.cpp` loads a saved model if one exists, fills a replay buffer with random experience, then trains a DQN agent against the lunar-lander environment while rendering the scene.

## Project Layout

- `src/` contains the application source files
- `include/` contains project headers
- `legacy/` keeps old scratch files and logs out of the main build
- `Makefile` wraps dependency installation and the CMake build
- `vcpkg.json` declares the C++ dependencies used by the project

## What The Code Does

- Creates a side-scrolling lunar lander environment with a landing platform and mountainous terrain
- Simulates the lander with Bullet physics
- Renders the world with SFML
- Computes state observations such as position, velocity, angle, and fuel
- Uses a DQN agent with a replay buffer and a target network to choose actions
- Saves and reloads the neural-network checkpoint as `Q_net`
- Can emit simulation videos as `Simulation_No*.mp4`

## Controls

The default runtime path in `src/main.cpp` supports:

- `Ctrl+Q` to schedule simulation mode
- `Ctrl+W` to schedule manual mode
- `Ctrl+E` to skip the current episode
- `A` / `D` or `Left` / `Right` for thrust control in manual mode
- `W` or `Up` for the boosted thrust input in manual mode

## Guide

## Installation

Clone the repository:

```bash
git clone https://github.com/malisaber/RL-Agent-Playing-LunarLander-Game.git
cd RL-Agent-Playing-LunarLander-Game
```


## Build Requirements

You will need a C++ toolchain, CMake, Git, and GNU Make or a compatible `make` implementation.

The project uses `vcpkg` to fetch libraries automatically. The first build can take a while because `opencv4` and `libtorch` are large dependencies.

For Windows, we recommend:

- Visual Studio Build Tools or another MSVC-compatible C++ toolchain
- Git
- PowerShell

For Ubuntu, we recommend:

- `build-essential`
- `cmake`
- `git`
- `make`

## Build

Install dependencies and build the project:

```sh
make deps
make build
```

Run the executable:

```sh
make run
```

This workflow works on both Windows and Ubuntu. The build output is placed under `build/bin/`.

### Make Targets

- `make deps` bootstraps `vcpkg` if needed and installs the C++ dependencies
- `make deps-check` only verifies the local toolchain
- `make deps-install` bootstraps `vcpkg` and installs the declared packages
- `make build` configures and compiles the project
- `make run` launches the executable

## Notes

- The repository now uses a standard `src/` + `include/` layout.
- Experimental and historical files live in `legacy/` and are not part of the main build.
- If you want to switch runtime modes, `src/main.cpp` contains the compile-time toggles used by the original author.

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE).
