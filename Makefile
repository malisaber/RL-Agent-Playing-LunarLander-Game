PROJECT_NAME := LunarLander
BUILD_DIR := build
CONFIG ?= Release
CMAKE ?= cmake
VCPKG_ROOT ?= .vcpkg

ifeq ($(OS),Windows_NT)
EXE_SUFFIX := .exe
VCPKG_EXE := $(VCPKG_ROOT)/vcpkg.exe
VCPKG_BOOTSTRAP := $(VCPKG_ROOT)/bootstrap-vcpkg.bat
VCPKG_TRIPLET ?= x64-windows
else
export CC := /usr/bin/gcc
export CXX := /usr/bin/g++
EXE_SUFFIX :=
VCPKG_EXE := $(VCPKG_ROOT)/vcpkg
VCPKG_BOOTSTRAP := $(VCPKG_ROOT)/bootstrap-vcpkg.sh
VCPKG_TRIPLET ?= x64-linux
endif

CMAKE_TOOLCHAIN := $(abspath $(VCPKG_ROOT))/scripts/buildsystems/vcpkg.cmake
PROJECT_BINARY := $(PROJECT_NAME)$(EXE_SUFFIX)

.PHONY: all deps deps-check deps-install configure build run clean

all: build

deps: deps-install

deps-check:
ifeq ($(OS),Windows_NT)
	powershell -NoProfile -ExecutionPolicy Bypass -Command "if (-not (Get-Command 'git' -ErrorAction SilentlyContinue)) { throw 'git was not found on PATH.' }; if (-not (Get-Command '$(CMAKE)' -ErrorAction SilentlyContinue)) { throw 'cmake was not found on PATH.' }; if (-not (Test-Path '$(VCPKG_EXE)')) { Write-Host 'vcpkg will be cloned and bootstrapped on the first deps-install run.' } else { Write-Host 'vcpkg is already present at $(VCPKG_EXE).' }"
else
	@command -v git >/dev/null 2>&1 || { echo "git was not found on PATH." >&2; exit 1; }
	@command -v "$(CMAKE)" >/dev/null 2>&1 || { echo "cmake was not found on PATH." >&2; exit 1; }
	@if [ ! -x "$(VCPKG_EXE)" ]; then echo "vcpkg will be cloned and bootstrapped on the first deps-install run."; else echo "vcpkg is already present at $(VCPKG_EXE)."; fi
endif

deps-install:
ifeq ($(OS),Windows_NT)
	powershell -NoProfile -ExecutionPolicy Bypass -Command "if (-not (Test-Path '$(VCPKG_EXE)')) { git clone https://github.com/microsoft/vcpkg.git '$(VCPKG_ROOT)'; & '$(VCPKG_BOOTSTRAP)' -disableMetrics }; & '$(VCPKG_EXE)' install --triplet $(VCPKG_TRIPLET)"
else
	@if [ ! -x "$(VCPKG_EXE)" ]; then git clone https://github.com/microsoft/vcpkg.git "$(VCPKG_ROOT)"; sh "$(VCPKG_BOOTSTRAP)"; fi
	@$(VCPKG_EXE) install --triplet $(VCPKG_TRIPLET)
endif

configure: deps
	$(CMAKE) -S . -B $(BUILD_DIR) -DCMAKE_BUILD_TYPE=$(CONFIG) -DCMAKE_TOOLCHAIN_FILE=$(CMAKE_TOOLCHAIN) -DVCPKG_TARGET_TRIPLET=$(VCPKG_TRIPLET)

build: configure
	$(CMAKE) --build $(BUILD_DIR) --config $(CONFIG)

run: build
ifeq ($(OS),Windows_NT)
	powershell -NoProfile -ExecutionPolicy Bypass -Command "$$exe = Get-ChildItem -Path '$(BUILD_DIR)/bin' -Filter '$(PROJECT_BINARY)' -Recurse | Select-Object -First 1; if (-not $$exe) { throw 'Executable not found. Run `make build` first.' }; & $$exe.FullName"
else
	@exe=$$(find "$(BUILD_DIR)/bin" -type f -name "$(PROJECT_BINARY)" -print -quit 2>/dev/null); \
	if [ -z "$$exe" ]; then echo "Executable not found. Run 'make build' first." >&2; exit 1; fi; \
	"$$exe"
endif

clean:
	$(CMAKE) -E rm -rf $(BUILD_DIR)
