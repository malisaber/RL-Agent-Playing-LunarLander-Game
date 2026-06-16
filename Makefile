PROJECT_NAME := LunarLander
BUILD_DIR := build
VCPKG_ROOT ?= .vcpkg
VCPKG_EXE := $(VCPKG_ROOT)/vcpkg.exe
VCPKG_TRIPLET ?= x64-windows
CONFIG ?= Release
CMAKE_TOOLCHAIN := $(abspath $(VCPKG_ROOT))/scripts/buildsystems/vcpkg.cmake
CMAKE ?= cmake

.PHONY: all deps deps-check deps-install configure build run clean

all: build

deps: deps-install

deps-check:
	powershell -NoProfile -ExecutionPolicy Bypass -Command "if (-not (Get-Command 'git' -ErrorAction SilentlyContinue)) { throw 'git was not found on PATH.' }; if (-not (Get-Command '$(CMAKE)' -ErrorAction SilentlyContinue)) { throw 'cmake was not found on PATH.' }; if (-not (Test-Path '$(VCPKG_EXE)')) { Write-Host 'vcpkg will be cloned and bootstrapped on the first deps-install run.' } else { Write-Host 'vcpkg is already present at $(VCPKG_EXE).' }"

deps-install:
	powershell -NoProfile -ExecutionPolicy Bypass -Command "if (-not (Test-Path '$(VCPKG_EXE)')) { git clone https://github.com/microsoft/vcpkg.git '$(VCPKG_ROOT)'; & '$(VCPKG_ROOT)/bootstrap-vcpkg.bat' -disableMetrics }; & '$(VCPKG_EXE)' install --triplet $(VCPKG_TRIPLET)"

configure: deps
	$(CMAKE) -S . -B $(BUILD_DIR) -DCMAKE_BUILD_TYPE=$(CONFIG) -DCMAKE_TOOLCHAIN_FILE=$(CMAKE_TOOLCHAIN) -DVCPKG_TARGET_TRIPLET=$(VCPKG_TRIPLET)

build: configure
	$(CMAKE) --build $(BUILD_DIR) --config $(CONFIG)

run: build
	powershell -NoProfile -ExecutionPolicy Bypass -Command "$$exe = Get-ChildItem -Path '$(BUILD_DIR)/bin' -Filter '$(PROJECT_NAME).exe' -Recurse | Select-Object -First 1; if (-not $$exe) { throw 'Executable not found. Run `make build` first.' }; & $$exe.FullName"

clean:
	$(CMAKE) -E rm -rf $(BUILD_DIR)
