# Configuration file for the Sphinx documentation builder.

# -- Path setup --------------------------------------------------------------

import os
import subprocess
import sys

sys.path.insert( 0, os.path.abspath('./../') )
sys.path.insert( 0, os.path.abspath('./../scripts') )
sys.path.insert( 0, os.path.abspath('./../src') )


# -- Project information -----------------------------------------------------

project = 'RT2 Assignment 2'
copyright = '2022, Francesco Ganci (S4143910)'
author = 'Francesco Ganci (S4143910)'

# The full version, including alpha/beta/rc tags
release = '1.0.0'


# -- General configuration ---------------------------------------------------

# list of extensions
extensions = [
'myst_parser',
'sphinx.ext.autodoc',
'sphinx.ext.napoleon',
'sphinx.ext.autosummary',
'sphinx.ext.doctest',
'sphinx.ext.viewcode',
'sphinx.ext.githubpages',
'sphinx.ext.intersphinx',
'sphinx.ext.coverage',
'sphinx.ext.duration',
'breathe'
]

# template paths
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages
# see 
# https://github.com/python/python-docs-theme/issues/45
html_theme = 'sphinx_rtd_theme'

# the root of the docmentation
master_doc = 'index'

# allow Pygments to try and guess the language
highlight_language = 'guess'

# format managed by the builder
'''
source_suffix = {
    '.rst': 'restructuredtext',
    '.txt': 'markdown',
    '.md': 'markdown',
}
'''
source_suffix = [".rst", ".md"]

# Style sheets and other static files
html_static_path = ['_static']


# -- Options for breathe -----------------------------------------------------

# generate the Doxygen docs before going on going
subprocess.call( 'doxygen Doxyfile', shell=True )

# path of the doxygen generated HTML
breathe_projects = {
  "rt2_assignment_2": "build/xml/"
}

# title of the Doxygen project
breathe_default_project = "rt2_assignment_2"

# ???
breathe_default_members = ('members', 'undoc-members')
