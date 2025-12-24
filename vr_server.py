"""
VR WebSocket Server for ARCS VR Control
Handles real-time VR controller data from Quest 3 browser
"""

import asyncio
import json
import threading
import time
from typing import Optional

try:
    import websockets
    from websockets.server import serve
    WEBSOCKETS_AVAILABLE = True
except ImportError:
    WEBSOCKETS_AVAILABLE = False
    print("Warning: websockets package not installed. VR control will be limited.")


class VRControlServer:
    """WebSocket server for VR controller data processing."""
    
    def __init__(self, host: str = '0.0.0.0', port: int = 8442):
        self.host = host
        self.port = port
        self.running = False
        self.clients = set()
        self.server = None
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.thread: Optional[threading.Thread] = None
        
        # Latest VR data
        self.vr_data = None
        self.last_update = 0
        
        # Arm control state
        self.arm_tracking_active = False
        self.initial_arm_position = None
        
    async def handler(self, websocket):
        """Handle incoming WebSocket connections."""
        self.clients.add(websocket)
        client_addr = websocket.remote_address
        print(f"[VR] Client connected: {client_addr}")
        
        try:
            async for message in websocket:
                await self.process_message(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.discard(websocket)
            print(f"[VR] Client disconnected: {client_addr}")
            self.handle_disconnect()
    
    async def process_message(self, websocket, message: str):
        """Process incoming VR controller data."""
        try:
            data = json.loads(message)
            self.vr_data = data
            self.last_update = time.time()
            
            # Handle grip release messages
            if data.get('gripReleased'):
                hand = data.get('hand', 'unknown')
                print(f"[VR] Grip released: {hand}")
                if hand == 'right':
                    self.arm_tracking_active = False
                    self.initial_arm_position = None
                return
            
            # Process controller data
            left = data.get('leftController', {})
            right = data.get('rightController', {})
            
            # Track arm mode state
            if right.get('gripActive'):
                if not self.arm_tracking_active:
                    self.arm_tracking_active = True
                    print("[VR] Arm tracking started")
            
            # Send acknowledgment
            await websocket.send(json.dumps({'status': 'ok'}))
            
        except json.JSONDecodeError as e:
            print(f"[VR] Invalid JSON: {e}")
        except Exception as e:
            print(f"[VR] Error processing message: {e}")
    
    def handle_disconnect(self):
        """Handle client disconnection - stop all movement."""
        from state import state
        state.stop_all_movement()
        self.arm_tracking_active = False
        print("[VR] Safety stop triggered on disconnect")
    
    async def broadcast(self, message: str):
        """Broadcast message to all connected clients."""
        if self.clients:
            await asyncio.gather(
                *[client.send(message) for client in self.clients],
                return_exceptions=True
            )
    
    async def start_server(self):
        """Start the WebSocket server."""
        print(f"[VR] Starting WebSocket server on ws://{self.host}:{self.port}")
        
        async with serve(self.handler, self.host, self.port) as server:
            self.server = server
            self.running = True
            print(f"[VR] WebSocket server ready on port {self.port}")
            await asyncio.Future()  # Run forever
    
    def run_in_thread(self):
        """Run the server in a background thread."""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        
        try:
            self.loop.run_until_complete(self.start_server())
        except Exception as e:
            print(f"[VR] Server error: {e}")
        finally:
            self.running = False
    
    def start(self):
        """Start the VR server in a background thread."""
        if not WEBSOCKETS_AVAILABLE:
            print("[VR] WebSockets not available - VR server disabled")
            return False
        
        if self.thread and self.thread.is_alive():
            print("[VR] Server already running")
            return True
        
        self.thread = threading.Thread(target=self.run_in_thread, daemon=True)
        self.thread.start()
        return True
    
    def stop(self):
        """Stop the VR server."""
        self.running = False
        if self.loop:
            self.loop.call_soon_threadsafe(self.loop.stop)
        if self.thread:
            self.thread.join(timeout=2)
        print("[VR] Server stopped")
    
    def get_status(self) -> dict:
        """Get current VR connection status."""
        return {
            'running': self.running,
            'clients': len(self.clients),
            'arm_tracking': self.arm_tracking_active,
            'last_update': self.last_update,
            'data_age': time.time() - self.last_update if self.last_update > 0 else None
        }
    
    def is_connected(self) -> bool:
        """Check if any VR client is connected."""
        return len(self.clients) > 0


# Global VR server instance
vr_server = VRControlServer()


def start_vr_server():
    """Start the global VR server."""
    return vr_server.start()


def stop_vr_server():
    """Stop the global VR server."""
    vr_server.stop()


def get_vr_status():
    """Get VR server status."""
    return vr_server.get_status()
