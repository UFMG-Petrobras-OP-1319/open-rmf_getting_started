# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information
import sphinx_rtd_theme

project = 'op1319'
copyright = '2025, Iuro Nascimento'
author = 'Iuro Nascimento'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "myst_parser",             # For parsing Markdown with MyST
    "sphinx.ext.autodoc",      # Core autodoc extension
    "sphinx.ext.napoleon",     # If you use Google or NumPy style docstrings
    "sphinx_autodoc_typehints" # If you want type hints in the generated docs
]
# So Sphinx knows it should treat .md files as Markdown
source_suffix = {
    '.md': 'markdown',
    '.rst': 'restructuredtext',
}

# import os
# import sys
# sys.path.insert(0, os.path.abspath('../..'))  # Make sure Sphinx can find your modules

templates_path = ['_templates']
exclude_patterns = []

# MyST configuration (optional customizations)
myst_enable_extensions = [
    "colon_fence",  # Allows ::: fenced code blocks
    # "deflist",     # Example: definition lists in Markdown
    # "linkify",     # Auto-convert URLs to links
    # ... more MyST features as needed
]

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

# If you want Sphinx’s “master” document to be named something other than 'index'
# e.g., if you want to keep an index.rst in `source/` but also reference index.md outside
# you can set master_doc = 'index' or some other name if needed.
master_doc = "index"  # or however you want to name your main doc
