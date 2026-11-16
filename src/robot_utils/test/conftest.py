import os
import sys

# Run from a source checkout: the tools are in python/, not installed.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "python"))
