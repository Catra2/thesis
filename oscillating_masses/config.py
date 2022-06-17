"""
Configurable parameters
"""

import os

class config:
    """
    File structure
    """
    base_path = os.path.abspath(os.path.dirname(__file__))

    """
    Globals
    """

    """
    AI-related parameters
    """
    learning_rate = 1e-3

    """
    MPC-related parameters
    """
    horizon = 20