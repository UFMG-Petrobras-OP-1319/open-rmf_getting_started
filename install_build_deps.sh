#!/usr/bin/env bash
set -e

#############################################
# Documentation and Explanation
#############################################
#
# This script installs pipx, installs Poetry via pipx, sets up a dedicated
# Poetry environment for Sphinx, MyST, and other doc packages, and optionally
# installs Poetry shell completions for Bash, Fish, and Zsh.
#
# Steps:
#   1. Install pipx (isolated Python CLI tool installer).
#   2. Install Poetry via pipx, so Poetry itself is isolated.
#   3. (Optional) Configure Poetry to put its venv in .env/doc at the root of the project.
#   4. Initialize or update pyproject.toml to include Sphinx, MyST Parser, etc.
#   5. Install Poetry shell completions if the corresponding shells are detected.
#   6. Optionally add an auto-activation line to ~/.bashrc, so that the doc environment
#      is sourced automatically for new Bash sessions.
#
# .venv in the root:
#   - If you want a truly local environment, you can configure Poetry to place
#     the venv at "./.venv". This keeps doc-building dependencies separate
#     from the user’s global Python or conda environment.
#
#############################################

#############################################
# 0. Pre-flight checks
#############################################

# Make sure Python and pip are installed (basic check).
if ! command -v python3 &> /dev/null; then
    sudo apt install python3 python3-venv python3-pip pipx
fi

if ! command -v pip &> /dev/null; then
    sudo apt install python3-pip
fi

if ! command -v pipx &> /dev/null; then
    sudo apt install pipx
fi

#############################################
# 1. Install pipx (user-wide)
#############################################
echo "Installing pipx (if not already installed)..."
python3 -m pipx ensurepath

#############################################
# 2. Install Poetry via pipx
#############################################
echo "Installing Poetry via pipx..."
pipx install poetry || true  # if already installed, ignore error

# Make sure 'poetry' is in PATH now. If not, user might need to re-source shell.
if ! command -v poetry &> /dev/null; then
    echo "Poetry not found in PATH. Try opening a new terminal or run:"
    echo "  source ~/.bashrc"
    echo "Then re-run this script if needed."
    exit 1
fi


# Explanation: This sets the path for all Poetry-managed virtualenvs in this project
# to .env/doc. If you only want a single local venv, you might also set:
#   poetry config virtualenvs.in-project true
# But that typically creates .venv/ in the project root. Choose whichever you prefer.

#############################################
# 4. Create or update Poetry project dependencies
#############################################
# If there's no pyproject.toml, let's create a minimal one:
if [ ! -f "pyproject.toml" ]; then
    echo "No pyproject.toml found, creating a minimal Poetry project..."
    # -n for no interactive prompts
    poetry init --name op1319_docs \
        --dependency "sphinx" \
        --dependency "myst-parser" \
        --dependency "sphinx-autodoc-typehints" \
        --dependency "sphinx_rtd_theme" \
        -n
else
    echo "pyproject.toml found, ensuring required packages are present..."
fi

#############################################
# 3. (Optional) Configure Poetry to store venv in .env/doc
#############################################
# If you want a local environment, un-comment below lines:
#
# echo "Configuring Poetry to store the virtualenv at ./.env/doc..."

# (pwd)/.env/doc
#poetry config virtualenvs.in-project = true

poetry install

#############################################
# 5. Install Poetry shell completions
#############################################
echo
echo "Installing Poetry shell completions if shells are present..."
# BASH completions
if command -v bash &> /dev/null; then
    # Typically we can write completions to ~/.local/share/bash-completion/completions/poetry
    # or /etc/bash_completion.d/poetry if we have root permissions.
    poetry completions bash >> ~/.bash_completion
    echo "Bash completions installed at ~/.bash_completion"
fi

## pyautoenv installation
if [[ ! -d /opt/pyautoenv ]]; then
    git clone https://github.com/hsaunders1904/pyautoenv.git /opt/pyautoenv
fi
# FISH completions
if command -v fish &> /dev/null; then
    # Usually fish completions live in ~/.config/fish/completions/
    FISH_COMPLETIONS_DIR="$HOME/.config/fish/completions"
    mkdir -p "$FISH_COMPLETIONS_DIR"
    poetry completions fish > "$FISH_COMPLETIONS_DIR/poetry.fish"
    echo "Fish completions installed at $FISH_COMPLETIONS_DIR/poetry.fish"

    echo 'source /opt/pyauotenv/pyautoenv.fish' >> ~/.config/fish/config.fish
fi

# ZSH completions
if command -v zsh &> /dev/null; then
    # There's no universal standard for user-level zsh completion. 
    # We'll create ~/.zfunc and add it to fpath if not already set.
    ZFUNC_DIR="$HOME/.zfunc"
    mkdir -p "$ZFUNC_DIR"
    poetry completions zsh > "$ZFUNC_DIR/_poetry"

    # Add to fpath in ~/.zshrc if not present:
    if ! grep -q "$ZFUNC_DIR" "$HOME/.zshrc" 2>/dev/null; then
        echo "fpath+=${ZFUNC_DIR}" >> "$HOME/.zshrc"
        echo "Added 'fpath+=${ZFUNC_DIR}' to ~/.zshrc for Zsh completions."
    fi
    echo "Zsh completions installed at $ZFUNC_DIR/_poetry"
    echo "Restart or re-source your shell to enable completions."

    echo 'source /opt/pyauotenv/pyautoenv.zsh' >> ~/.zshrc
fi

#############################################
# 7. Final Instructions
#############################################
cat << EOF

===============================================================
Installation complete!

Here's a summary:

1) A Poetry-based environment for Sphinx docs is now set up:
- Packages installed: Sphinx, myst-parser, sphinx-autodoc-typehints,
sphinx_rtd_theme (and the built-in napoleon in sphinx).

2) Poetry completions:
- Installed completions for each shell found (bash, fish, zsh).
- You may need to restart or re-source your shell for them to take effect.

3) Activating the environment:
- If you enabled auto-activation, open a new Bash shell and it should
already be activated. Otherwise, use:
poetry shell
from this project directory.

4) Building docs:
- Run "make html" if you have a Makefile that calls sphinx-build.
- Or use "sphinx-build -b html . build" (assuming conf.py is in this folder).
- Your HTML docs will appear in ./build/html/.

5) Nested environments:
- While possible, it's usually not recommended to have Poetry's environment
active within another environment (like conda/venv). Deactivate or exit
your existing environment first to avoid conflicts.

6) Using .env/doc:
- If you uncommented the lines to set "poetry config virtualenvs.path ...",
your environment is stored locally at ./.env/doc. That helps isolate doc
dependencies from your system or conda environments.

All done! Enjoy your Sphinx docs in an isolated Poetry environment.
===============================================================
EOF
