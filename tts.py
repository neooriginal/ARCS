"""Text-to-Speech module for robot audio feedback using gTTS."""

import threading
import queue
import tempfile
import os
import subprocess
from gtts import gTTS
from config import TTS_ENABLED, TTS_DEVICE


class TTSEngine:
    def __init__(self):
        self.enabled = TTS_ENABLED
        self.device = TTS_DEVICE
        self.speech_queue = queue.Queue()
        self.worker_thread = None
        
        if self.enabled:
            # Check if we have an audio player available
            self.audio_player = self._find_audio_player()
            if self.audio_player:
                self.worker_thread = threading.Thread(target=self._worker, daemon=True)
                self.worker_thread.start()
                print(f"[TTS] Initialized with Google voices (using {self.audio_player})")
            else:
                print("[TTS] No audio player found. Install mpg123, ffplay, or mpg321")
                self.enabled = False
        else:
            print("[TTS] TTS disabled in config")
    
    def _find_audio_player(self):
        """Find an available audio player on the system."""
        players = ['mpg123', 'ffplay', 'mpg321', 'play']
        
        for player in players:
            try:
                subprocess.run([player, '--version'], 
                             stdout=subprocess.DEVNULL, 
                             stderr=subprocess.DEVNULL,
                             timeout=1)
                return player
            except:
                continue
        return None
    
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
                print(f"[TTS] Worker error: {e}")
    
    def _speak_blocking(self, text):
        """Generate speech using gTTS and play it."""
        import sys
        temp_file = None
        try:
            print(f"[TTS] Generating: '{text}'")
            sys.stdout.flush()
            
            # Create temporary file for audio
            with tempfile.NamedTemporaryFile(delete=False, suffix='.mp3') as fp:
                temp_file = fp.name
            
            # Generate speech with gTTS (this requires internet)
            print(f"[TTS] Connecting to Google TTS API...")
            sys.stdout.flush()
            
            tts = gTTS(text=text, lang='en', slow=False, timeout=5)
            tts.save(temp_file)
            
            print(f"[TTS] Audio file created: {temp_file}")
            sys.stdout.flush()
            
            # Play audio based on which player we found
            if self.audio_player == 'mpg123':
                cmd = ['mpg123', '-q', temp_file]
            elif self.audio_player == 'ffplay':
                cmd = ['ffplay', '-nodisp', '-autoexit', '-loglevel', 'quiet', temp_file]
            elif self.audio_player == 'mpg321':
                cmd = ['mpg321', '-q', temp_file]
            else:  # play (sox)
                cmd = ['play', '-q', temp_file]
            
            # Play the audio
            print(f"[TTS] Playing with: {' '.join(cmd)}")
            sys.stdout.flush()
            
            result = subprocess.run(cmd, 
                         stdout=subprocess.PIPE, 
                         stderr=subprocess.PIPE,
                         timeout=10,
                         check=False)  # Don't raise exception, we'll check manually
            
            if result.returncode == 0:
                print(f"[TTS] Played successfully")
            else:
                print(f"[TTS] Player exited with code {result.returncode}")
                if result.stdout:
                    print(f"[TTS] stdout: {result.stdout.decode()}")
                if result.stderr:
                    print(f"[TTS] stderr: {result.stderr.decode()}")
            sys.stdout.flush()
                
        except subprocess.TimeoutExpired:
            print(f"[TTS] Playback timed out")
            sys.stdout.flush()
        except Exception as e:
            print(f"[TTS] Error: {e}")
            sys.stdout.flush()
            import traceback
            traceback.print_exc()
        finally:
            # Clean up temp file
            if temp_file:
                try:
                    os.unlink(temp_file)
                except:
                    pass
    
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
