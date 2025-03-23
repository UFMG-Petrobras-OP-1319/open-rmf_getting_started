# How to build the html or latex documentation

The process of installation gave you now have a Poetry-based (python) environment
that includes:
- Sphinx: static html with its own markup language (.rst files)
- myst-parser: A markup language parser that converts .md to .rst
- sphinx-autodoc-typehints: sphinx extension that improves codeblocks
- sphinx_rtd_theme: the sphinx theme extension

Your Sphinx config file (conf.py) can remain where it is. Your docs are in
Markdown, so you'll rely on MyST.

To build your documentation:

1) Activate the environment:
   - If you have another environment activated, deactivate first to avoid conflicts
   running:
   ```bash
   deactivate
   ```
   or for conda environmens:
   ```bash
   conda deactivate
   ```

   - Otherwise, run inside the repository folder:
   ```bash
   poetry env activate
   ```
   and after building the docs deactivate by running:
   ```bash
   deactivate
   ```

2) From root of the repository, run:
    ```bash
    make html
    ```
   or equivalently:
   ```bash
   sphinx-build -b html . build
   ```

   The HTML output will appear in ./build/html/.

