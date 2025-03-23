#!/usr/bin/env bash
set -e

#############################################
# Documentation and Explanation
#############################################
#
# This script sets up an isolated environment using Poetry for building
# Sphinx-based documentation from Markdown files. Here's the high-level flow:
#
# 1. **pipx**:
#    - pipx is a tool to install and run Python CLI tools in isolated environments.
#    - Installing pipx "globally" (i.e., in your user environment) allows you to
#      cleanly install tools like Poetry without polluting system packages.
#
# 2. **Poetry via pipx**:
#    - Poetry is a Python dependency manager and virtual environment manager.
#    - By installing it via pipx, Poetry itself is isolated from system Python.
#
# 3. **Poetry environment**:
#    - Once Poetry is installed, we create (or use) a Poetry project that
#      contains all dependencies for generating Sphinx docs.
#    - Sphinx, myst-parser, sphinx-autodoc-typehints, and the sphinx_rtd_theme
#      are installed *inside* that Poetry environment, not in your system Python.
#
# 4. **Why these packages?**:
#    - sphinx: The main Sphinx engine to transform .md/.rst into HTML (and other formats).
#    - myst-parser: Lets Sphinx understand Markdown files using MyST syntax.
#    - sphinx-autodoc-typehints: Display Python function/method type hints in the docs
#      (useful if you had Python code, but it won’t hurt if you don’t).
#    - sphinx_rtd_theme: Popular “Read the Docs” HTML theme for Sphinx.
#    - napoleon (part of sphinx.ext) parses Google or NumPy-style docstrings (again,
#      useful if you had Python code).
#
# 5. **Isolation benefits**:
#    - You might already use Conda, virtualenv, or other environment managers for your
#      own Python projects. Poetry creates *its own* environment for the documentation
#      tools, so it won’t interfere with your personal environment or system packages.
#    - You only “activate” this Poetry environment when you want to build docs; otherwise,
#      your personal environment remains unaffected.
#
# 6. **Auto-activation**:
#    - Optional step to add “source path/to/poetry/env/bin/activate” in your ~/.bashrc
#      so the doc environment is automatically active in new shells. If you prefer
#      manual activation (with `poetry shell` or the `source` command), remove that step.
#
#############################################

#############################################
# 0. Pre-flight checks (optional)
#############################################

# Make sure Python and pip are installed (basic check)
if ! command -v python3 &> /dev/null; then
  echo "Python3 is not installed or not in PATH. Please install it first."
  exit 1
fi
if ! command -v pip &> /dev/null; then
  echo "pip is not installed or not in PATH. Please install it first."
  exit 1
fi

#############################################
# 1. Install pipx (user-wide)
#############################################
echo "Installing pipx (if not already installed)..."
pip install --user --upgrade pipx
python3 -m pipx ensurepath

# Refresh shell so that pipx is immediately recognized (optional)
# You might need to restart your terminal session, or source your rc file:
# source ~/.bashrc  # or ~/.zshrc, etc.

#############################################
# 2. Install Poetry via pipx
#############################################
echo "Installing Poetry via pipx..."
pipx install poetry

#############################################
# 3. Set up a Poetry environment for docs
#############################################
# You can use an existing project or create a new one if you want. 
# For simplicity, let's do it in the current directory.

# Initialize a pyproject.toml if you don't already have one:
if [ ! -f "pyproject.toml" ]; then
  echo "No pyproject.toml found, creating a minimal Poetry project..."
  poetry init --name doc-env --dependency "sphinx" --dependency "myst-parser" \
              --dependency "sphinx-autodoc-typehints" --dependency "sphinx_rtd_theme" \
              -n
fi

# If you do already have a pyproject.toml, you can just 'poetry add' the deps:
poetry add sphinx myst-parser sphinx-autodoc-typehints sphinx_rtd_theme

#############################################
# 4. (Optional) Auto-activate environment
#############################################
# By default, Poetry won't force an environment to be active in your shell. 
# If you prefer to manually type `poetry shell` each time you want to build docs,
# skip this step. Otherwise, we can append a "source" line to your ~/.bashrc.

echo
read -p "Do you want to automatically activate the Poetry doc environment in your shell? [y/N] " AUTO_ACTIVATE
if [[ "$AUTO_ACTIVATE" =~ ^[Yy]$ ]]; then
  # We fetch the venv path from Poetry
  POETRY_ENV_PATH=$(poetry env info --path 2>/dev/null || true)
  if [ -z "$POETRY_ENV_PATH" ]; then
    # If the environment doesn't exist yet, create it
    echo "No active virtual environment found, creating one..."
    poetry install
    POETRY_ENV_PATH=$(poetry env info --path)
  fi

  # Append the source line to ~/.bashrc (or another shell rc)
  if [ -n "$POETRY_ENV_PATH" ]; then
    echo "source \"$POETRY_ENV_PATH/bin/activate\"" >> ~/.bashrc
    echo "Added 'source \"$POETRY_ENV_PATH/bin/activate\"' to your ~/.bashrc."
    echo "Open a new terminal or 'source ~/.bashrc' to activate automatically."
  else
    echo "Could not determine Poetry environment path. Please activate manually."
  fi
fi

#############################################
# 5. Instructions for building docs
#############################################

cat << EOF

===============================================================
Installation complete!

- You now have a Poetry-based environment that includes:
  Sphinx, myst-parser, sphinx-autodoc-typehints, sphinx_rtd_theme.

- Your Sphinx config file (conf.py) can remain where it is. 
- Your docs are in Markdown (index.md, api.md, usage.md), so you'll rely on MyST.

To build your documentation:

1) Activate the environment:
   - If you chose auto-activation (above), simply open a new terminal,
     and your doc-env will be active automatically.
   - Otherwise, run:
       poetry shell
     to activate the environment.

2) From within that environment, run:
   make html

   ...or equivalently:
   sphinx-build -b html . build

   The HTML output will appear in ./build/html/.

When you no longer need the doc environment, you can 'exit' or close the shell.
This keeps your system's main Python environment clean and unaffected.
===============================================================
EOF

