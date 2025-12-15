"""Text-to-Speech module for robot audio feedback using gTTS."""

import threading
import queue
import tempfile
import os
from gtts import gTTS
from config import TTS_ENABLED, TTS_DEVICE

# Try to import pygame for audio playback (but don't initialize yet)
PYGAME_AVAILABLE = False
try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    print("[TTS] Warning: pygame not available, TTS will be disabled")

class TTSEngine:
    def __init__(self):
        self.enabled = TTS_ENABLED
        self.device = TTS_DEVICE
        self.speech_queue = queue.Queue()
        self.worker_thread = None
        self.mixer_initialized = False
        
        if self.enabled:
            # Try to initialize pygame mixer (optional)
            if PYGAME_AVAILABLE:
                try:
                    pygame.mixer.init()
                    self.mixer_initialized = True
                    print("[TTS] Initialized (using gTTS with pygame)")
                except Exception as e:
                    print(f"[TTS] Pygame mixer failed: {e}")
                    print("[TTS] Will use system audio player as fallback")
            else:
                print("[TTS] Pygame not available, will use system audio player")
            
            # Start worker thread regardless of pygame status
            self.worker_thread = threading.Thread(target=self._worker, daemon=True)
            self.worker_thread.start()
            print("[TTS] TTS enabled with Google voices")
        else:
            print("[TTS] TTS disabled in config")
    
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
        """Generate speech using gTTS and play it."""
        temp_file = None
        try:
            # Create temporary file for audio
            with tempfile.NamedTemporaryFile(delete=False, suffix='.mp3') as fp:
                temp_file = fp.name
            
            # Generate speech with gTTS
            tts = gTTS(text=text, lang='en', slow=False)
            tts.save(temp_file)
            
            # Try to play with pygame first
            if self.mixer_initialized:
                try:
                    pygame.mixer.music.load(temp_file)
                    pygame.mixer.music.play()
                    
                    # Wait for playback to finish
                    while pygame.mixer.music.get_busy():
                        pygame.time.Clock().tick(10)
                except Exception as e:
                    print(f"[TTS] Pygame playback failed: {e}, trying system command")
                    self._play_with_system_command(temp_file)
            else:
                # Fallback to system command
                self._play_with_system_command(temp_file)
                
        except Exception as e:
            print(f"[TTS] Speech error: {e}")
        finally:
            # Clean up temp file
            if temp_file:
                try:
                    os.unlink(temp_file)
                except:
                    pass
    
    def _play_with_system_command(self, audio_file):
        """Fallback audio playback using system commands."""
        import subprocess
        
        # Try different audio players in order of preference
        players = [
            ['mpg123', '-q', audio_file],           # Common on Linux
            ['ffplay', '-nodisp', '-autoexit', '-loglevel', 'quiet', audio_file],  # ffmpeg
            ['mpg321', '-q', audio_file],           # Alternative
            ['play', audio_file],                   # sox
        ]
        
        for player_cmd in players:
            try:
                subprocess.run(player_cmd, 
                             stdout=subprocess.DEVNULL, 
                             stderr=subprocess.DEVNULL,
                             timeout=10,
                             check=True)
                return  # Success
            except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired):
                continue  # Try next player
        
        print("[TTS] Warning: No audio player found. Install mpg123, ffplay, or mpg321")
    
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
