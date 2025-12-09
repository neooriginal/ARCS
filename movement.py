"""
RoboCrew Control System - Movement Control
Handles wheel movement commands and the continuous movement loop.
"""

import time
from config import MOVEMENT_LOOP_INTERVAL, REMOTE_TIMEOUT
from state import state


def execute_movement(movement):
    """
    Execute a movement command based on the movement state dict.
    Returns True if successful, False otherwise.
    """
    if state.controller is None:
        return False
    
    try:
        # Calculate net movement vector
        fwd = 0.0
        if movement.get('forward'): fwd += 1.0
        if movement.get('backward'): fwd -= 1.0
        
        lat = 0.0
        if movement.get('left'): lat += 1.0
        if movement.get('right'): lat -= 1.0
        
        # Use vector control if available
        if hasattr(state.controller, 'set_velocity_vector'):
            state.controller.set_velocity_vector(fwd, lat)
        else:
            # Fallback to single direction
            if fwd > 0: state.controller._wheels_write('up')
            elif fwd < 0: state.controller._wheels_write('down')
            elif lat > 0: state.controller._wheels_write('left')
            elif lat < 0: state.controller._wheels_write('right')
            else: state.controller._wheels_stop()
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

        # Safety: Stop if connection lags while moving
        if any(movement.values()) and (time.time() - state.last_movement_activity > REMOTE_TIMEOUT):
            state.stop_all_movement()
            movement = state.get_movement()  # Reset local movement to stop immediately


        
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
