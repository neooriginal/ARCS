"""Text-to-Speech module for robot audio feedback."""

import threading
import queue
from config import TTS_ENABLED, TTS_DEVICE

class TTSEngine:
    def __init__(self):
        self.enabled = TTS_ENABLED
        self.device = TTS_DEVICE
        self.speech_queue = queue.Queue()
        self.worker_thread = None
        self.engine = None
        
        if self.enabled:
            try:
                import pyttsx3
                # Initialize engine - don't fail if this raises warnings
                try:
                    self.engine = pyttsx3.init()
                except Exception as init_error:
                    print(f"[TTS] Engine init warning: {init_error}")
                    # Try fallback init without driver specification
                    try:
                        self.engine = pyttsx3.init(driverName='espeak')
                    except:
                        raise Exception("Could not initialize TTS engine")
                
                # Try to configure voice (don't fail if this doesn't work)
                if self.engine:
                    try:
                        voices = self.engine.getProperty('voices')
                        if voices:
                            # Try to find a male voice
                            male_voice = None
                            for voice in voices:
                                voice_name = getattr(voice, 'name', '').lower()
                                if 'male' in voice_name or 'man' in voice_name:
                                    male_voice = voice.id
                                    break
                            
                            if male_voice:
                                self.engine.setProperty('voice', male_voice)
                    except Exception as voice_error:
                        # Voice selection failed, use default
                        pass
                    
                    # Set other properties (also don't fail if this doesn't work)
                    try:
                        self.engine.setProperty('rate', 150)
                        self.engine.setProperty('volume', 1.0)
                    except:
                        pass
                
                # Start worker thread
                self.worker_thread = threading.Thread(target=self._worker, daemon=True)
                self.worker_thread.start()
                print("[TTS] Initialized successfully")
                
            except Exception as e:
                print(f"[TTS] Initialization failed, TTS disabled: {e}")
                self.enabled = False
                self.engine = None
    
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
        """Use pyttsx3 to generate speech."""
        if self.engine:
            try:
                self.engine.say(text)
                self.engine.runAndWait()
            except Exception as e:
                print(f"[TTS] Speak failed: {e}")
    
    def speak(self, text):
        """Queue text for asynchronous speech."""
        if self.enabled and text:
            self.speech_queue.put(text)
    
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

def shutdown():
    """Shutdown TTS."""
    if _tts:
        _tts.shutdown()
