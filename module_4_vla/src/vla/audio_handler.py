"""
Audio input handling for VLA system
"""

import numpy as np
import sounddevice as sd
from pydub import AudioSegment
import io
import wave


class AudioHandler:
    """
    Handles audio input from microphone for the VLA system
    """

    def __init__(self, sample_rate=16000, channels=1, chunk_size=1024):
        self.sample_rate = sample_rate
        self.channels = channels
        self.chunk_size = chunk_size
        self.audio_buffer = []

    def record_audio(self, duration=5):
        """
        Record audio for specified duration
        """
        print(f"Recording audio for {duration} seconds...")
        audio_data = sd.rec(
            int(duration * self.sample_rate),
            samplerate=self.sample_rate,
            channels=self.channels,
            dtype='float32'
        )
        sd.wait()  # Wait for recording to complete
        return audio_data

    def normalize_audio(self, audio_data):
        """
        Normalize audio data to appropriate range
        """
        # Normalize to [-1, 1] range
        max_val = np.max(np.abs(audio_data))
        if max_val > 0:
            normalized = audio_data / max_val
        else:
            normalized = audio_data
        return normalized

    def convert_to_wav_bytes(self, audio_data):
        """
        Convert numpy audio array to WAV bytes for Whisper processing
        """
        # Normalize the audio
        normalized_audio = self.normalize_audio(audio_data)

        # Convert to 16-bit integers
        audio_int16 = (normalized_audio * 32767).astype(np.int16)

        # Create WAV bytes in memory
        wav_buffer = io.BytesIO()
        with wave.open(wav_buffer, 'wb') as wav_file:
            wav_file.setnchannels(self.channels)
            wav_file.setsampwidth(2)  # 16-bit
            wav_file.setframerate(self.sample_rate)
            wav_file.writeframes(audio_int16.tobytes())

        wav_bytes = wav_buffer.getvalue()
        wav_buffer.close()

        return wav_bytes

    def preprocess_audio(self, audio_data):
        """
        Preprocess audio for Whisper ASR
        """
        # Normalize audio
        normalized = self.normalize_audio(audio_data)

        # Convert to WAV format for Whisper
        wav_bytes = self.convert_to_wav_bytes(normalized)

        return wav_bytes