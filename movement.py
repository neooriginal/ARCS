"""
RoboCrew Control System - Movement Control
Handles wheel movement commands and the continuous movement loop.
"""

import time
from config import MOVEMENT_LOOP_INTERVAL
from state import state


def execute_movement(movement):
    """
    Execute a movement command based on the movement state dict.
    Returns True if successful, False otherwise.
    """
    if state.controller is None:
        return False
    
    try:
        if movement.get('forward'):
            state.controller._wheels_write('up')
        elif movement.get('backward'):
            state.controller._wheels_write('down')
        elif movement.get('left'):
            state.controller._wheels_write('left')
        elif movement.get('right'):
            state.controller._wheels_write('right')
        else:
            state.controller._wheels_stop()
        return True
    except Exception as e:
        state.last_error = f"Movement error: {str(e)}"
        return False


def movement_loop():
    """
    Continuous movement control thread.
    Keeps wheels moving while key is held.
    """
    while state.running:
        if state.controller is None:
            time.sleep(0.1)
            continue
        
        movement = state.get_movement()
        
        try:
            execute_movement(movement)
        except Exception as e:
            state.last_error = f"Movement loop error: {str(e)}"
        
        time.sleep(MOVEMENT_LOOP_INTERVAL)


def stop_movement():
    """Stop all wheel movement."""
    if state.controller:
        try:
            state.controller._wheels_stop()
        except Exception as e:
            state.last_error = f"Stop movement error: {str(e)}"
