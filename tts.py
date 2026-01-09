"""Text-to-Speech module using gTTS."""

import threading
import queue
import tempfile
import os
import subprocess
import logging
from core.config_manager import get_config
from gtts import gTTS

TTS_ENABLED = get_config("TTS_ENABLED")
TTS_AUDIO_DEVICE = get_config("TTS_AUDIO_DEVICE")
TTS_TLD = get_config("TTS_TLD")

logger = logging.getLogger(__name__)

try:
    from langdetect import detect
    LANGDETECT_AVAILABLE = True
except ImportError:
    LANGDETECT_AVAILABLE = False


class TTSEngine:
    def __init__(self):
        self.enabled = TTS_ENABLED
        self.audio_device = TTS_AUDIO_DEVICE
        self.tld = TTS_TLD
        self.speech_queue = queue.Queue()
        self.worker_thread = None
        self.audio_player = None
        
        if self.enabled:
            # Start background initialization and worker
            self.worker_thread = threading.Thread(target=self._worker, daemon=True)
            self.worker_thread.start()
        else:
            logger.info("TTS disabled in config")
    
    def _find_audio_player(self):
        """Find available audio player."""
        players = ['aplay', 'mpg123', 'ffplay', 'mpg321', 'play']
        
        for player in players:
            try:
                subprocess.run([player, '--version'], 
                             stdout=subprocess.DEVNULL, 
                             stderr=subprocess.DEVNULL,
                             timeout=1)
                return player
            except Exception:
                continue
        return None
    
    def _worker(self):
        """Background worker processing speech queue."""
        # Detect audio player
        self.audio_player = self._find_audio_player()
        if self.audio_player:
            logger.info(f"TTS Initialized with Google voices (using {self.audio_player})")
        else:
            logger.warning("TTS: No audio player found, speech disabled")
            self.enabled = False
            return

        while True:
            try:
                text = self.speech_queue.get(timeout=1)
                if text is None:
                    break
                self._speak_blocking(text)
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"TTS Worker error: {e}")
    
    def _detect_language(self, text):
        """Auto-detect language, fallback to English."""
        if not LANGDETECT_AVAILABLE:
            return 'en'
        
        try:
            lang = detect(text)
            return lang if lang else 'en'
        except Exception:
            return 'en'
    
    def _speak_blocking(self, text):
        """Generate and play speech."""
        import sys
        temp_mp3 = None
        temp_wav = None
        
        try:
            lang = self._detect_language(text)
            logger.info(f"Speaking ({lang}): '{text}'")
            
            with tempfile.NamedTemporaryFile(delete=False, suffix='.mp3') as fp:
                temp_mp3 = fp.name
            
            tts = gTTS(text=text, lang=lang, tld=self.tld, slow=False, timeout=5)
            tts.save(temp_mp3)
            
            with tempfile.NamedTemporaryFile(delete=False, suffix='.wav') as fp:
                temp_wav = fp.name
            
            result = subprocess.run(['mpg123', '-w', temp_wav, temp_mp3], 
                         stdout=subprocess.DEVNULL, 
                         stderr=subprocess.DEVNULL,
                         timeout=5)
            
            if result.returncode != 0:
                logger.error(f"TTS MP3 decode failed")
                return
            
            # Play at max volume on HDMI
            subprocess.run(['aplay', '-D', self.audio_device, temp_wav], 
                         stdout=subprocess.DEVNULL, 
                         stderr=subprocess.DEVNULL,
                         timeout=10)
            
        except subprocess.TimeoutExpired:
            logger.warning("TTS Timeout")
        except Exception as e:
            logger.error(f"TTS Error: {e}")
        finally:
            if temp_mp3:
                try:
                    os.unlink(temp_mp3)
                except:
                    pass
            if temp_wav:
                try:
                    os.unlink(temp_wav)
                except:
                    pass
    
    def speak(self, text):
        """Queue text for speech."""
        if self.enabled and text and self.worker_thread and self.worker_thread.is_alive():
            self.speech_queue.put(text)
    
    def shutdown(self):
        """Shutdown TTS engine."""
        if self.worker_thread:
            self.speech_queue.put(None)
            self.worker_thread.join(timeout=2)


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
