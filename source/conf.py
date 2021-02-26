# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import pkg_resources
import os
import sys
sys.path.insert(0, os.path.abspath('source'))
sys.path.insert(0, os.path.abspath('.'))
sys.path.insert(0, os.path.abspath('..'))
import pybullet_api
import pybullet
import sphinx_bootstrap_theme


# -- Project information -----------------------------------------------------

project = 'pybullet'
copyright = '2021, pybullet authors'
author = 'pybullet authors'

version = pkg_resources.get_distribution('pybullet').version
release = pkg_resources.get_distribution('pybullet').version


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
    #"sphinx.ext.intersphinx",
    "sphinx.ext.mathjax",
    #"sphinx.ext.imgmath",
    #"matplotlib.sphinxext.plot_directive",
    "numpydoc",
    #"sphinx_gallery.gen_gallery",
]

autosummary_generate = True  # generate files at doc/source/_apidoc
class_members_toctree = False

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = []


html_theme = "bootstrap"
html_theme_path = sphinx_bootstrap_theme.get_html_theme_path()
html_theme_options = {
    "bootswatch_theme": "readable",
    "navbar_sidebarrel": False,
    "bootstrap_version": "3",
    "nosidebar": True,
    "body_max_width": '100%',
    "navbar_links": [
        ("Contents", "index"),
        #("Examples", "_auto_examples/index"),
        ("API", "api"),
    ],
}
html_static_path = ['_static']
html_last_updated_fmt = '%b %d, %Y'
html_use_smartypants = True
html_show_sourcelink = False
html_show_sphinx = False
html_show_copyright = True
