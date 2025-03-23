
# Documentation engine installation
---

The script install_build_deps.sh sets up an isolated environment using Poetry
for building Sphinx-based documentation from Markdown files. Here's the
high-level explanation of the main package installed:

1. **pipx**:
    - pipx is a tool to install and run Python CLI tools in isolated environments.
    - Installing pipx "globally" (i.e., in your user environment) allows you to
    cleanly install tools like Poetry without polluting system packages.

2. **Poetry via pipx**:
    - Poetry is a Python dependency manager and virtual environment manager.
    - By installing it via pipx, Poetry itself is isolated from system Python.

3. **Poetry environment**:
    - Once Poetry is installed, we create (or use) a Poetry project that
    contains all dependencies for generating Sphinx docs.
    - Sphinx, myst-parser, sphinx-autodoc-typehints, and the sphinx_rtd_theme
    are installed *inside* the Poetry environment, not in your system Python.

4. **Why these packages?**:
    - sphinx: The main Sphinx engine to transform .md/.rst into HTML (and other
    formats).
    - myst-parser: Lets Sphinx understand Markdown files using MyST syntax.
    - sphinx-autodoc-typehints: Display Python function/method type hints in the
    docs (useful if you had Python code, but it won’t hurt if you don’t).
    - sphinx_rtd_theme: Popular “Read the Docs” HTML theme for Sphinx.
    - napoleon (part of sphinx.ext) parses Google or NumPy-style docstrings 
    (again, useful if you had Python code).

5. **Isolation benefits**:
    - You might already use Conda, virtualenv, or other environment managers for
    your own Python projects. Poetry creates *its own* environment for the 
    documentation tools, so it won’t interfere with your personal environment or
    system packages.
    - You only “activate” this Poetry environment when you want to build docs;
    otherwise, your personal environment remains unaffected.

6. **Auto-activation**:
    - pyautoenv is automatically installed, so you can automatically activate the
    project environment while cd'ing into its root and deactivate when leaving
    the project folder

