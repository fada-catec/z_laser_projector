# -- Path setup --------------------------------------------------------------
import os
import catkin_pkg.package
import sys

catkin_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
catkin_package = catkin_pkg.package.parse_package(os.path.join(catkin_dir, catkin_pkg.package.PACKAGE_MANIFEST_FILENAME))

sys.path.insert(0, os.path.abspath('../src/z_laser_viz'))

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
templates_path = ['_templates'] 
source_suffix = '.rst' 
master_doc = 'index'

# -- Project information -----------------------------------------------------
project = u'z_laser_viz'
copyright = u'2020, Rafael Luque'
author = u'Rafael Luque'

version = catkin_package.version 
release = catkin_package.version

language = None

exclude_patterns = [u'.build', 'Thumbs.db', '.DS_Store']

pygments_style = 'sphinx' # None

# -- Options for todo extension ----------------------------------------------
todo_include_todos = True
autodoc_default_flags = ['members']
autodoc_inherit_docstrings = False

# -- Options for HTML output -------------------------------------------------
html_theme = 'nature'
html_theme_options = {
    'logo_only': True,
}
html_logo = './_static/ROSIN.png'
html_favicon = './_static/Icon_016x016.png'
html_static_path = ['_static']
html_show_sourcelink = False

# -- Options for HTMLHelp output ---------------------------------------------
htmlhelp_basename = 'z_laser_viz_doc'

# -- Options for LaTeX output ------------------------------------------------
latex_elements = {}
latex_documents = [
    (master_doc, 'z_laser_viz_doc.tex', u'z_laser_viz Documentation',
     u'Rafael Luque', 'manual'),
]

# -- Options for manual page output ------------------------------------------
man_pages = [
    (master_doc, 'z_laser_viz_doc', u'z_laser_viz Documentation',
     [author], 1)
]

# -- Options for Texinfo output ----------------------------------------------
texinfo_documents = [
    (master_doc, 'z_laser_viz', u'z_laser_viz Documentation',
     author, 'z_laser_viz', 'One line description of project.',
     'Miscellaneous'),
]

# -- Options for intersphinx extension ---------------------------------------
intersphinx_mapping = {
    'https://docs.python.org/': None,
    'https://www.sphinx-doc.org/en/1.8/': None}

rst_epilog = ""
rst_epilog += "\n.. |ROS| replace:: :abbr:`ROS (Robot Operating System)`"

# -- Options for Epub output -------------------------------------------------
epub_title = project
epub_exclude_files = ['search.html']

# -- Extension interface -----------------------------------------------------
from sphinx import addnodes  # noqa

def setup(app):
    app.add_object_type('confval', 'confval',
                        objname='configuration value',
                        indextemplate='pair: %s; configuration value')
