# How to build the html or latex documentation
This is a brief guide on how to build the html and/or latex documentation. If you
have not installed the documentation build system (Sphinx + extensions), refer
to the installation [documentation](install-docs.md).

## Workflow and installed environment
The build system relies on python environments to isolate its dependencies from
the system packages. To isolate even further, we use poetry to manage a isolated
environment for the build system, and we automatically activate the environment 
when you type cd to go into the root of the project and automatically deactivates
this environment when leaving the project.

Once you are in the project root, you can run:
```bash
$ make html
```
or
```bash
$ make latex
```
to generate the desired documentation from the markdown. The documentation files
are built into `build/{html,latex}`. regular pdflatex can handle the latex to pdf
conversion and any server like github pages can host the html documentation
website.

Note: you don't have to change anything in the markdown files that are
compatible with GitHub Markdown, you only have to reserve the index.md file to 
include markdown + extensions from Myst parser so the build process can work.
Those extensions can be used in other markdown files, but they are not
recognized by GitHub.

## Automatically handling of environments switch
After the installation script set up a python environment for working with Sphinx,
you can `cd` into the project root, pyautoenv takes care of automatically 
deactivate any active python environment activate the current project poetry
environment. After you change directory to outside the project pyautoenv 
automatically deactivate the project environment, but do not activate the
previous environment, so if there was any environment activated, you have to
reactivate yourself.

If you don't want to isolate the build system from the system or your base
environment, refer to the installation [documentation](install-docs.md) on how to
install the build system into your environment.


## Manually handling of environment switch
to activate the environment, if you have another environment activated,
deactivate first to avoid conflicts running:
   ```bash
   deactivate
   ```
or for Conda environments:
   ```bash
   $ conda deactivate
   ```

Otherwise, run inside the repository folder:
   ```bash
   $ deactivate || eval $(poetry env activate)
   ```
After building the docs, deactivate by running and reactivating any
environment you were using before this process.

This process isolates the Sphinx from the rest of the system and
environment packages from Sphinx and dependencies.

## Building html or latex documentation from the markdown files
build the html or latex documentation:
  From root of the repository, run:
```bash
  $ make html
  ```
  or
  ```bash
  $ make latex
  ```
  or equivalently:
  ```bash
  $ sphinx-build -b html . build
  ```
  The HTML output will appear in ./build/html/.

