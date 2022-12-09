import os 
import sys
sys.path.insert(0,os.path.abspath('..'))

project = 'MoveBot'
copyright = '2022'
author = 'BotChocolate'
release = '1.0.0'

extensions = [
    'sphinx.ext.todo',
    'sphinx.ext.viewcode',
    'sphinx.ext.autodoc',
    'm2r2',
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']
autodoc_mock_imports = ["movebot_interfaces"] # ignore this for now
source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown'
}

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

