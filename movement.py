"""
ARCS - Movement Control
Handles wheel movement commands and the continuous movement loop.
"""

import time
import logging
from core.config_manager import get_config
from state import state

MOVEMENT_LOOP_INTERVAL = get_config("MOVEMENT_LOOP_INTERVAL")
REMOTE_TIMEOUT = get_config("REMOTE_TIMEOUT")
STALL_CHECK_INTERVAL = get_config("STALL_CHECK_INTERVAL")
STALL_LOAD_THRESHOLD = get_config("STALL_LOAD_THRESHOLD")

logger = logging.getLogger(__name__)


def execute_movement(movement):
    if state.controller is None:
        return False
    
    try:
        fwd = 0.0
        fwd += float(movement.get('forward', 0.0))
        fwd -= float(movement.get('backward', 0.0))
        
        rot = 0.0
        rot += float(movement.get('left', 0.0))
        rot -= float(movement.get('right', 0.0))
        
        lat = 0.0
        lat += float(movement.get('slide_left', 0.0))
        lat -= float(movement.get('slide_right', 0.0))
        
        # Use vector control if available
        if hasattr(state.controller, 'set_velocity_vector'):
            state.controller.set_velocity_vector(fwd, lat, rot)
        else:
            # Fallback to single direction
            if fwd > 0: state.controller._wheels_write('up')
            elif fwd < 0: state.controller._wheels_write('down')
            elif rot > 0: state.controller._wheels_write('left')
            elif rot < 0: state.controller._wheels_write('right')
            elif lat > 0: state.controller._wheels_write('slide_left')
            elif lat < 0: state.controller._wheels_write('slide_right')
            else: state.controller._wheels_stop()
        return True
    except Exception as e:
        state.last_error = f"Movement error: {str(e)}"
        return False


def movement_loop():
    last_stall_check = 0
    
    while state.running:
        if state.controller is None:
            time.sleep(0.1)
            continue
        
        movement = state.get_movement()

        # Safety: Stop if connection lags while moving
        if any(movement.values()) and (time.time() - state.last_movement_activity > REMOTE_TIMEOUT):
            state.stop_all_movement()
            movement = state.get_movement()  # Reset local movement to stop immediately

        # Safety: Stall Detection
        if time.time() - last_stall_check > STALL_CHECK_INTERVAL:
            try:
                msg = state.controller.check_stall(STALL_LOAD_THRESHOLD)
                if msg:
                     state.last_error = f"SAFETY STOP: {msg}"
                     state.stop_all_movement()
                     print(f"!!! {msg} !!!")
            except Exception as e:
                logger.warning(f"Stall check error: {e}")
            last_stall_check = time.time()
        
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
