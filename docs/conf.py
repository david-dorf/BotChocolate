import os
import sys

sys.path.insert(0, os.path.abspath('..'))

project = 'BotChocolate'
copyright = '2022, BotChocolate'
author = 'BotChocolate'
release = '1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration
extensions = [
    'sphinx.ext.todo',
    'sphinx.ext.viewcode',
    'sphinx.ext.autodoc',
    'm2r2'
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']
autodoc_mock_imports = ["movebot_interfaces"]
source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown'
}
# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
html_context = {
  'display_github': True,
  'github_user': 'ME495-EmbeddedSystems',
  'github_repo': 'hw3group-botchocolate',
  'github_version': 'main/docs/',
}
