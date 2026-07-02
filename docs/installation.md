# Installation Guide for SO-101 Robot Arm

This guide walks you through setting up the workspace and installing dependencies for LeDroid-101. We use Python 3.13.7 and the `uv` package manager for fast, reproducible environment setup.

## Prerequisites

Before starting, ensure you have the following installed on your machine:

- **macOS** (compatible with your current OS layout)
- **uv**: A fast Python package installer and resolver.

If `uv` is not installed, you can install it using curl (macOS/Linux):

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

---

## Setting Up the Codebase

Follow these steps to initialize your environment:

### 1. Configure the Python Virtual Environment

We pin the workspace python version to `3.13.7`. Initialize the workspace and its virtual environment utilizing `uv`:

```bash
# Pin python version to 3.13.7
uv python pin 3.13.7

# Create a virtual environment using the pinned Python 3.13.7
uv venv
```

Ensure the virtual environment is activated in your terminal:

```bash
source .venv/bin/activate
```

### 2. Install Project Dependencies

Our repository contains a [pyproject.toml](pyproject.toml) configuration list that defines the required version pins and packages (like PyTorch and LERobot packages). Install all project dependencies to the isolated virtual environment by executing:

```bash
uv sync --all-extras
```

This commands reads the [pyproject.toml](pyproject.toml) (and locks them in [uv.lock](uv.lock)), installing:

- `torch>=2.11.0`
- `torchvision>=0.26.0`
- `lerobot` (with Feetech motor utilities)

### 3. Verify the Installation

To confirm everything is configured properly, run the following verification code within Python:

```bash
python -c "import torch; import lerobot; print('PyTorch:', torch.__version__); print('LeRobot:', lerobot.__version__)"
```

Once installed, please proceed to [docs/assembly.md](docs/assembly.md) to set motor IDs, assemble the SO-101, and perform joint calibration.
