"""Text-to-Speech module for robot audio feedback."""

import threading
import queue
import subprocess
from config import TTS_ENABLED, TTS_DEVICE

class TTSEngine:
    def __init__(self):
        self.enabled = TTS_ENABLED
        self.device = TTS_DEVICE
        self.speech_queue = queue.Queue()
        self.worker_thread = None
        self.current_speed = 150  # Default speed
        self.default_speed = 150  # Store default for reset
        
        if self.enabled:
            # Test if espeak is available
            try:
                subprocess.run(['espeak', '--version'], 
                             stdout=subprocess.DEVNULL, 
                             stderr=subprocess.DEVNULL, 
                             timeout=1)
                self.worker_thread = threading.Thread(target=self._worker, daemon=True)
                self.worker_thread.start()
                print("[TTS] Initialized (using espeak)")
            except Exception as e:
                print(f"[TTS] espeak not found, TTS disabled")
                self.enabled = False
    
    def _worker(self):
        """Background worker that processes speech queue."""
        while True:
            try:
                text = self.speech_queue.get(timeout=1)
                if text is None:
                    break
                self._speak_blocking(text)
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[TTS] Error: {e}")
    
    def _speak_blocking(self, text):
        """Use espeak directly."""
        try:
            # Use espeak with male voice and max volume (-a 200)
            subprocess.run(
                ['espeak', '-v', 'en-us+m3', '-s', str(self.current_speed), '-a', '200', text],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=5
            )
        except Exception as e:
            # Fallback without voice specification
            try:
                subprocess.run(
                    ['espeak', '-s', str(self.current_speed), '-a', '200', text],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=5
                )
            except:
                pass
    
    def speak(self, text):
        """Queue text for asynchronous speech."""
        if self.enabled and text:
            self.speech_queue.put(text)
    
    def set_speed(self, speed):
        """Set TTS speed (words per minute)."""
        self.current_speed = max(80, min(300, int(speed)))  # Clamp between 80-300 WPM
    
    def reset_speed(self):
        """Reset speed to default."""
        self.current_speed = self.default_speed
    
    def get_speed(self):
        """Get current speed."""
        return self.current_speed
    
    def shutdown(self):
        """Shutdown TTS engine."""
        if self.worker_thread:
            self.speech_queue.put(None)
            self.worker_thread.join(timeout=2)


# Global TTS instance
_tts = None

def init():
    """Initialize TTS engine."""
    global _tts
    if _tts is None:
        _tts = TTSEngine()

def speak(text):
    """Speak text via TTS."""
    if _tts:
        _tts.speak(text)

def set_speed(speed):
    """Set TTS speed."""
    if _tts:
        _tts.set_speed(speed)

def reset_speed():
    """Reset TTS speed to default."""
    if _tts:
        _tts.reset_speed()

def get_speed():
    """Get current TTS speed."""
    if _tts:
        return _tts.get_speed()
    return 150  # Default

def shutdown():
    """Shutdown TTS."""
    if _tts:
        _tts.shutdown()
