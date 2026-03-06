# Contributing to RoboCup Rescue RMRC 2026

Thank you for your interest in contributing. This document provides guidelines for contributing to the project.

## Development Setup

1. **Clone the repository**
   ```bash
   git clone https://github.com/YOUR_ORG/robocup_rescue_2026.git
   cd robocup_rescue_2026
   ```

2. **Create a virtual environment**
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate   # Linux/macOS
   # or: .venv\Scripts\activate   # Windows
   pip install -r requirements.txt
   ```

3. **Copy environment template**
   ```bash
   cp .env.example .env
   # Edit .env with your Jetson IP, camera devices, etc.
   ```

## Code Style

- Use **Python 3.10+**.
- Follow PEP 8 for Python code.
- Use type hints where practical.
- Keep functions focused; prefer small, testable units.

## Pull Request Process

1. **Fork** the repository and create a branch from `main` (or `master`).
2. **Make your changes** with clear, descriptive commits.
3. **Run tests** before submitting:
   ```bash
   pytest tests/ -v
   ```
4. **Open a Pull Request** with:
   - A clear title and description
   - Reference to any related issues
   - Screenshots or logs if UI/behavior changed

## Reporting Issues

Use [GitHub Issues](https://github.com/YOUR_ORG/robocup_rescue_2026/issues) for:

- **Bug reports**: Describe the problem, steps to reproduce, and your environment (OS, Python version, Jetson model).
- **Feature requests**: Describe the use case and proposed solution.

## Project Structure

- `laptop/` — Operator station (GUI, joystick, arm leader)
- `jetson/` — Robot brain (vision, drive, SLAM, arm follower)
- `shared/` — Constants and protocol helpers
- `docs/` — Runbooks, checklists
- `tests/` — Unit and integration tests

## Questions

For competition-specific questions, refer to the [RoboCup Rescue RMRC rulebook](https://rescuerobotics.org/). For code questions, open a Discussion or Issue.
