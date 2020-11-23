# -*- coding: utf-8 -*-
#
# Configuration file for the Sphinx documentation builder.
#
# This file does only contain a selection of the most common options. For a
# full list see the documentation:
# http://www.sphinx-doc.org/en/master/config

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.

import os
import catkin_pkg.package
import sys

catkin_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
catkin_package = catkin_pkg.package.parse_package(os.path.join(catkin_dir, catkin_pkg.package.PACKAGE_MANIFEST_FILENAME))

sys.path.insert(0, os.path.abspath('../src/z_laser_projector'))

# -- General configuration ---------------------------------------------------

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.doctest',
    'sphinx.ext.intersphinx',
    'sphinx.ext.todo',
    'sphinx.ext.coverage',
    'sphinx.ext.mathjax',
    'sphinx.ext.ifconfig',
    'sphinx.ext.viewcode',
    'sphinx.ext.githubpages',
    'sphinx.ext.napoleon',
]

templates_path = ['.templates'] 

source_suffix = '.rst' 

# The master toctree document.
master_doc = 'index'

# -- Project information -----------------------------------------------------

project = u'z_laser_projector'
copyright = u'2020, Rafael Luque'
author = u'Rafael Luque'

version = catkin_package.version 
release = catkin_package.version


exclude_patterns = [u'.build', 'Thumbs.db', '.DS_Store']

pygments_style = 'sphinx' # None


# -- Options for todo extension ----------------------------------------------

todo_include_todos = True

autodoc_default_flags = ['members']
autodoc_inherit_docstrings = False

# -- Options for HTML output -------------------------------------------------

html_theme = 'nature' 

html_static_path = ['.static']

# -- Options for HTMLHelp output ---------------------------------------------

htmlhelp_basename = 'z_laser_projector_doc'


# -- Options for LaTeX output ------------------------------------------------

latex_elements = {
}

latex_documents = [
    (master_doc, 'z_laser_projector_doc.tex', u'z_laser_projector Documentation',
     u'Rafael Luque', 'manual'),
]


# -- Options for manual page output ------------------------------------------

man_pages = [
    (master_doc, 'z_laser_projector_doc', u'z_laser_projector Documentation',
     [author], 1)
]


# -- Options for Texinfo output ----------------------------------------------

texinfo_documents = [
    (master_doc, 'z_laser_projector', u'z_laser_projector Documentation',
     author, 'z_laser_projector', 'One line description of project.',
     'Miscellaneous'),
]

# -- Options for intersphinx extension ---------------------------------------

intersphinx_mapping = {'https://docs.python.org/': None}

# -- Options for Epub output -------------------------------------------------


# -- Extension interface -----------------------------------------------------
