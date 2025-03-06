#Detect the host OS
UNAME_S := $(shell uname -s)
UNAME_M := $(shell uname -m)

# set variables for conda installation based on OS
ifeq ($(UNAME_S),Darwin)
    # CONDA CONFIG
	ifeq ($(UNAME_M),arm64)
		MINICONDA_URL := https://repo.anaconda.com/miniconda/Miniconda3-latest-MacOSX-arm64.sh
		MINICONDA_INSTALLER := Miniconda3-latest-MacOSX-arm64.sh
	else
		MINICONDA_URL := https://repo.anaconda.com/miniconda/Miniconda3-latest-MacOSX-x86_64.sh
		MINICONDA_INSTALLER := Miniconda3-latest-MacOSX-x86_64.sh
	endif
	INSTALL_CMD := bash $(MINICONDA_INSTALLER) -b -p $(HOME)/miniconda
	SYSTEM_DEPENDENCIES := brew install cmake python tesseract
	CHECK_PACKAGE_MANAGER := @if ! command -v brew &> /dev/null; then \
								echo "Hombrew not found, installing..."; \
								/bin/bash -c "$$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"; \
							else \
								echo "Homebrew already installed."; \
							fi
endif

ifeq ($(UNAME_S),Linux)
    # CONDA CONFIG
	MINICONDA_URL := https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
	MINICONDA_INSTALLER := Miniconda3-latest-Linux-x86_64.sh
	INSTALL_CMD := bash $(MINICONDA_INSTALLER) -b -p $(HOME)/miniconda
	SYSTEM_DEPENDENCIES := sudo apt-get update && sudo apt-get install -y build-essential cmake python3 python3-pip tesseract-ocr
	CHECK_PACKAGE_MANAGER := @echo "No additional package manager needed for Linux."
endif

ifeq ($(findstring MINGW,$(UNAME_S)),MINGW)
    # CONDA CONFIG
	MINICONDA_URL := https://repo.anaconda.com/miniconda/Miniconda3-latest-Windows-x86_64.exe
	MINICONDA_INSTALLER := Miniconda3-latest-Windows-x86_64.exe
	INSTALL_CMD := start /wait $(MINICONDA_INSTALLER) /S /D=$(HOME)\miniconda
	SYSTEM_DEPENDENCIES := choco install cmake python3 tesseract
	CHECK_PACKAGE_MANAGER := @if ! command -v choco &> /dev/null; then \
								echo "Chocolatey not found, installing..."; \
								@powershell -NoProfile -ExecutionPolicy Bypass -Command "Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))"; \
							else \
								echo "Chocolatey already installed."; \
							fi
endif

ifeq ($(findstring MSYS,$(UNAME_S)),MSYS)
    # CONDA CONFIG
	MINICONDA_URL := https://repo.anaconda.com/miniconda/Miniconda3-latest-Windows-x86_64.exe
	MINICONDA_INSTALLER := Miniconda3-latest-Windows-x86_64.exe
	INSTALL_CMD := start /wait $(MINICONDA_INSTALLER) /S /D=$(HOME)\miniconda
	SYSTEM_DEPENDENCIES := choco install cmake python3 tesseract
	CHECK_PACKAGE_MANAGER := @if ! command -v choco &> /dev/null; then \
							echo "Chocolatey not found, installing..."; \
							@powershell -NoProfile -ExecutionPolicy Bypass -Command "Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))"; \
						else \
							echo "Chocolatey already installed."; \
						fi
endif

ifeq ($(findstring CYGWIN,$(UNAME_S)),CYGWIN)
    # CONDA CONFIG
	MINICONDA_URL := https://repo.anaconda.com/miniconda/Miniconda3-latest-Windows-x86_64.exe
	MINICONDA_INSTALLER := Miniconda3-latest-Windows-x86_64.exe
	INSTALL_CMD := start /wait $(MINICONDA_INSTALLER) /S /D=$(HOME)\miniconda
	SYSTEM_DEPENDENCIES := choco install cmake python3 tesseract
	CHECK_PACKAGE_MANAGER := @if ! command -v choco &> /dev/null; then \
								echo "Chocolatey not found, installing..."; \
								@powershell -NoProfile -ExecutionPolicy Bypass -Command "Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))"; \
							else \
								echo "Chocolatey already installed."; \
							fi
endif

CLEANUP_CMD := sudo rm $(MINICONDA_INSTALLER)

.PHONY: install 

install:
	@echo "Checking for package manager..."
	$(CHECK_PACKAGE_MANAGER)
	@echo "Installing system dependencies..."
	@$(SYSTEM_DEPENDENCIES)
	@echo "Checking for conda..."
	@if ! command -v conda &> /dev/null; then \
		echo "conda could not be found, installing Miniconda..."; \
		mkdir -p $(dirname $MINICONDA_INSTALLER); \
		sudo curl -fsSL $(MINICONDA_URL) -o $(MINICONDA_INSTALLER); \
		$(INSTALL_CMD); \
		$(CLEANUP_CMD); \
		export PATH="$(HOME)/miniconda/bin:$$PATH"; \
		conda init; \
	else \
		echo "conda already installed."; \
	fi
	@echo "Checking for conda environment 'pcr'..."
	@if ! conda env list | grep -q 'pcr'; then \
		echo "conda environment 'pcr' could not be found, creating it..."; \
		conda create -n pcr python=3.10 -y; \
		echo "environment 'pcr' created."; \
	else \
		echo "conda environment 'pcr' already exists."; \
	fi
	@echo "Activating environment 'pcr'..."
	@eval "$$(conda shell.bash hook) && conda activate pcr && pip install -r requirements.txt"

SRC_FILE := ./number_detection/prefix_min.c
LIB_DIR := ./number_detection/library
CC := gcc

ifeq ($(OS),Windows_NT)
	LIB_FILE := $(LIB_DIR)/number-detection-pkg.dll
	CFLAGS := -dynamiclib -o $(LIB_FILE)
else 
	ifeq ($(UNAME_S),Darwin)
		LIB_FILE := $(LIB_DIR)/number-detection-pkg.dylib
		CFLAGS := -dynamiclib -o $(LIB_FILE)
	else ifeq ($(UNAME_S),Linux)
		LIB_FILE := $(LIB_DIR)/number-detection-pkg.so
		CFLAGS := -shared -o $(LIB_FILE)
	else 
		$(error Unsupported operating system: $(UNAME_S))
	endif
endif

.PHONY: compile_lib
compile_lib:
	@echo "Compiling $(SRC_FILE) into $(LIB_FILE)..."
	@mkdir -p $(LIB_DIR)
	@$(CC) $(SRC_FILE) $(CFLAGS)
	@echo "Compilation finished."
