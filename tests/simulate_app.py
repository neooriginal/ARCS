import sys
import os
import time
import threading
import requests
import unittest
import logging
from unittest.mock import patch, MagicMock

# Add project root to path
sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/..'))

try:
    from tests.mocks import MockVideoCapture, MockRobot, mock_init_lidar
except ImportError:
    from mocks import MockVideoCapture, MockRobot, mock_init_lidar

# Disable default logging to keep output clean
logging.getLogger('werkzeug').setLevel(logging.ERROR)
logging.getLogger('engineio').setLevel(logging.ERROR)
logging.getLogger('socketio').setLevel(logging.ERROR)

class TestSimulation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        print("\n=== Starting Hardware-Free Simulation ===\n")
        
        # 1. Setup Patches
        # Patch cv2.VideoCapture globally
        cls.cv2_patcher = patch('cv2.VideoCapture', side_effect=MockVideoCapture)
        
        # Patch load_robot where it is USED in RobotSystem
        # core.robot_system imports it as 'from robots import load_robot' 
        cls.robot_patcher = patch('core.robot_system.load_robot', return_value=MockRobot())
        
        # Patch init_lidar where it is USED in main
        # main.py does 'from core.lidar import init_lidar' inside _deferred_init
        cls.lidar_patcher = patch('core.lidar.init_lidar', side_effect=mock_init_lidar)
        
        # Patch Auth to bypass for tests
        cls.auth_patch1 = patch('core.auth.is_auth_configured', return_value=True)
        cls.auth_patch2 = patch('core.auth.verify_token', return_value="test_user")
        cls.auth_patch3 = patch('routes.get_token_from_request', return_value="mock_token")
        
        cls.cv2_patcher.start()
        cls.robot_patcher.start()
        cls.lidar_patcher.start()
        cls.auth_patch1.start()
        cls.auth_patch2.start()
        cls.auth_patch3.start()
        
        # 2. Configure Environment
        # Set a test port
        cls.port = 5055 
        os.environ['WEB_PORT'] = str(cls.port)
        os.environ['CAMERA_PORT'] = '/dev/video0' # Dummy
        os.environ['AUTO_OPEN_DISPLAY'] = 'false'
        
        # 3. Initialize App
        from main import create_app, socketio, _deferred_init
        
        # Setup config manager to use env vars we just set
        from core.config_manager import config_manager
        # (Assuming config manager reads os.environ)
        
        # Manually trigger the deferred initialization which sets up RobotSystem
        print("[Test] Initializing Robot System...")
        _deferred_init()
        
        cls.app = create_app()
        
        # 4. Start Server in Background Thread
        print(f"[Test] Starting Server on port {cls.port}...")
        if socketio:
            target = lambda: socketio.run(cls.app, host='0.0.0.0', port=cls.port, allow_unsafe_werkzeug=True)
        else:
            print("[Test] SocketIO not available, using Flask app.run")
            target = lambda: cls.app.run(host='0.0.0.0', port=cls.port, threaded=True, use_reloader=False)
            
        cls.server_thread = threading.Thread(target=target, daemon=True)
        cls.server_thread.start()
        
        # Wait for server to come online
        print("[Test] Waiting for server...")
        for i in range(10):
            try:
                msg = requests.get(f'http://localhost:{cls.port}/login')
                if msg.status_code == 200:
                    print("[Test] Server is UP!")
                    break
            except requests.exceptions.ConnectionError:
                pass
            time.sleep(1)
        else:
            print("[Test] Server failed to start!")
            raise RuntimeError("Server start timeout")

    @classmethod
    def tearDownClass(cls):
        print("\n=== Tear Down ===")
        cls.cv2_patcher.stop()
        cls.robot_patcher.stop()
        cls.lidar_patcher.stop()
        cls.auth_patch1.stop()
        cls.auth_patch2.stop()
        cls.auth_patch3.stop()
        from state import state
        state.running = False
        # Note: socketio.run doesn't have an easy stop method in threaded mode without signals,
        # but since it's daemon=True, it will die with the main process.

    def test_01_status_endpoint(self):
        """Verify system status reports connected hardware."""
        print("Test 01: Status Endpoint")
        response = requests.get(f'http://localhost:{self.port}/status')
        self.assertEqual(response.status_code, 200)
        data = response.json()
        print(f"  -> Response: {data}")
        
        self.assertTrue(data['controller_connected'], "Controller should be simulated as connected")
        self.assertTrue(data['arm_connected'], "Arm should be simulated as connected")

    def test_02_video_stream(self):
        """Verify video feed returns data."""
        print("Test 02: Video Stream")
        response = requests.get(f'http://localhost:{self.port}/video_feed', stream=True)
        self.assertEqual(response.status_code, 200)
        
        # Read a few chunks
        chunks_read = 0
        for chunk in response.iter_content(chunk_size=1024):
            if chunk:
                chunks_read += 1
            if chunks_read > 5:
                break
        
        self.assertTrue(chunks_read > 0, "Should receive video stream data")
        print("  -> Video stream active")

    def test_03_arm_control(self):
        """Verify arm commands update state."""
        print("Test 03: Arm Control")
        # Move shoulder
        target = 45.5
        payload = {'positions': {'shoulder_pan': target}}
        response = requests.post(f'http://localhost:{self.port}/arm', json=payload)
        self.assertEqual(response.status_code, 200)
        
        # Verify response matches
        resp_data = response.json()
        self.assertEqual(resp_data['positions']['shoulder_pan'], target)
        
        # Verify internal state updated
        from state import state
        self.assertEqual(state.arm_positions['shoulder_pan'], target)
        print(f"  -> Shoulder moved to {target}")

    def test_04_lidar_reading(self):
        """Verify lidar distance is 123 from mock."""
        print("Test 04: Lidar Reading")
        # Give it a moment to update state via callback
        time.sleep(0.2)
        
        # Check display state endpoint which includes lidar info
        response = requests.get(f'http://localhost:{self.port}/display/state')
        self.assertEqual(response.status_code, 200)
        data = response.json()
        
        self.assertTrue(data['lidar_connected'])
        self.assertEqual(data['lidar_distance'], 123)
        print(f"  -> Lidar distance verified: {data['lidar_distance']}")

    def _ensure_agent_ready(self):
        """Helper to init agent if needed."""
        from state import state
        if not state.agent and state.robot_system:
             from main import _setup_agent
             _setup_agent(state.robot_system)

    def test_05_navigation_logic(self):
        """Verify basic navigation/AI logic init."""
        print("Test 05: Navigation Logic")
        
        from state import state
        # Ensure agent is ready (might have missed it during async init)
        self._ensure_agent_ready()

        # Start AI
        response = requests.post(f'http://localhost:{self.port}/ai/start')
        self.assertEqual(response.status_code, 200, f"AI Start failed: {response.text}")
        
        self.assertTrue(state.ai_enabled)
        print("  -> AI Started")
        
        # Stop AI
        response = requests.post(f'http://localhost:{self.port}/ai/stop')
        self.assertEqual(response.status_code, 200)
        self.assertFalse(state.ai_enabled)
        print("  -> AI Stopped")

if __name__ == '__main__':
    unittest.main()
