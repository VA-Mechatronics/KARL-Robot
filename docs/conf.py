import os
import sys
from pathlib import Path

""" build CLI
sphinx-build -E -b html docs docs/_build/html
"""

# Add project root directory to Python path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

project = 'KARL Robot'
copyright = '2026, VA-Mechatronics'
author = 'VA-Mechatronics'

# 1. Required Sphinx extensions
extensions = [
    'myst_parser',             # Readme extension
    'sphinx.ext.autodoc',      # Automatically extracts docstrings
    'sphinx.ext.napoleon',     # Supports Google-style docstrings (Args:, Examples:)
    'sphinx.ext.viewcode',     # Adds links to source code
    'sphinx.ext.intersphinx',  # Links to external docs (numpy, python)
]

exclude_patters = [
    'graphs/*'
]

# 2. Google-style docstring settings
napoleon_google_docstring = True
napoleon_numpy_docstring = False
napoleon_include_init_with_doc = False
napoleon_include_private_with_doc = False

# 3. Autodoc settings
autodoc_member_order = 'bysource'
autodoc_typehints = 'description'

# 4. Furo theme
html_theme = 'furo'
html_static_path = []
