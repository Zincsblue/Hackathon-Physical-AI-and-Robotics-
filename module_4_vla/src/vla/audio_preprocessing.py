"""
Audio preprocessing functions for robotic applications
"""

import numpy as np
import scipy.signal as signal
from typing import Tuple, Optional
import librosa


class AudioPreprocessor:
    """
    Audio preprocessing for robotic applications, optimized for Whisper ASR
    """

    def __init__(self, sample_rate: int = 16000, target_level: float = 0.1):
        """
        Initialize the audio preprocessor

        Args:
            sample_rate: Target sample rate for audio (default 16kHz for Whisper)
            target_level: Target audio level for normalization
        """
        self.sample_rate = sample_rate
        self.target_level = target_level

    def resample_audio(self, audio_data: np.ndarray, original_sr: int) -> np.ndarray:
        """
        Resample audio to target sample rate

        Args:
            audio_data: Input audio data
            original_sr: Original sample rate of the audio

        Returns:
            Resampled audio data
        """
        if original_sr == self.sample_rate:
            return audio_data

        # Use librosa for high-quality resampling
        resampled = librosa.resample(audio_data, orig_sr=original_sr, target_sr=self.sample_rate)
        return resampled

    def normalize_audio(self, audio_data: np.ndarray) -> np.ndarray:
        """
        Normalize audio to target level

        Args:
            audio_data: Input audio data

        Returns:
            Normalized audio data
        """
        # Calculate current RMS
        rms = np.sqrt(np.mean(audio_data ** 2))

        if rms == 0:
            return audio_data  # Avoid division by zero

        # Normalize to target level
        normalized = audio_data * (self.target_level / rms)

        # Prevent clipping
        max_val = np.max(np.abs(normalized))
        if max_val > 1.0:
            normalized = normalized / max_val

        return normalized

    def apply_preemphasis(self, audio_data: np.ndarray, preemph: float = 0.97) -> np.ndarray:
        """
        Apply pre-emphasis filter to enhance high frequencies

        Args:
            audio_data: Input audio data
            preemph: Pre-emphasis coefficient

        Returns:
            Pre-emphasized audio data
        """
        return signal.lfilter([1.0, -preemph], 1, audio_data)

    def remove_dc_offset(self, audio_data: np.ndarray) -> np.ndarray:
        """
        Remove DC offset from audio

        Args:
            audio_data: Input audio data

        Returns:
            Audio data with DC offset removed
        """
        return audio_data - np.mean(audio_data)

    def apply_noise_reduction(self, audio_data: np.ndarray, noise_threshold: float = 0.01) -> np.ndarray:
        """
        Apply simple noise reduction by attenuating low-level noise

        Args:
            audio_data: Input audio data
            noise_threshold: Threshold below which to reduce noise

        Returns:
            Noise-reduced audio data
        """
        # Identify noise regions (values below threshold)
        noise_mask = np.abs(audio_data) < noise_threshold

        # Apply soft noise reduction (reduce amplitude of noise regions)
        reduced_audio = audio_data.copy()
        reduced_audio[noise_mask] = reduced_audio[noise_mask] * 0.5  # Reduce noise by half

        return reduced_audio

    def apply_bandpass_filter(self, audio_data: np.ndarray, low_freq: float = 80, high_freq: float = 7600) -> np.ndarray:
        """
        Apply bandpass filter to focus on human speech frequencies

        Args:
            audio_data: Input audio data
            low_freq: Low frequency cutoff (Hz)
            high_freq: High frequency cutoff (Hz)

        Returns:
            Bandpass filtered audio data
        """
        nyquist = self.sample_rate / 2
        low = low_freq / nyquist
        high = high_freq / nyquist

        # Create bandpass filter
        b, a = signal.butter(4, [low, high], btype='band', analog=False)

        # Apply filter
        filtered_audio = signal.filtfilt(b, a, audio_data)

        return filtered_audio

    def vad_simple(self, audio_data: np.ndarray, threshold: float = 0.02, min_silence_duration: float = 0.1) -> np.ndarray:
        """
        Simple Voice Activity Detection (VAD) to trim silence

        Args:
            audio_data: Input audio data
            threshold: Energy threshold for speech detection
            min_silence_duration: Minimum silence duration to trim (in seconds)

        Returns:
            Audio data with leading/trailing silence removed
        """
        # Calculate frame energy
        frame_length = int(0.025 * self.sample_rate)  # 25ms frames
        hop_length = frame_length // 2

        # Pad audio to ensure we don't miss the end
        pad_length = frame_length - (len(audio_data) % hop_length)
        padded_audio = np.pad(audio_data, (0, pad_length), mode='constant')

        # Calculate energy for each frame
        frames = librosa.util.frame(padded_audio, frame_length=frame_length, hop_length=hop_length)
        frame_energy = np.mean(frames**2, axis=0)

        # Find speech frames (above threshold)
        speech_frames = frame_energy > threshold

        # Find first and last speech frames
        speech_indices = np.where(speech_frames)[0]

        if len(speech_indices) == 0:
            return np.array([])  # No speech detected

        first_speech = speech_indices[0]
        last_speech = speech_indices[-1]

        # Calculate sample indices
        start_sample = first_speech * hop_length
        end_sample = min((last_speech + 1) * hop_length, len(audio_data))

        return audio_data[start_sample:end_sample]

    def preprocess_for_whisper(self, audio_data: np.ndarray, original_sr: int = None) -> np.ndarray:
        """
        Complete preprocessing pipeline optimized for Whisper

        Args:
            audio_data: Input audio data
            original_sr: Original sample rate (if different from target)

        Returns:
            Preprocessed audio data ready for Whisper
        """
        processed_audio = audio_data.copy()

        # Remove DC offset first
        processed_audio = self.remove_dc_offset(processed_audio)

        # Resample if needed
        if original_sr is not None and original_sr != self.sample_rate:
            processed_audio = self.resample_audio(processed_audio, original_sr)

        # Apply bandpass filter to focus on speech frequencies
        processed_audio = self.apply_bandpass_filter(processed_audio)

        # Apply noise reduction
        processed_audio = self.apply_noise_reduction(processed_audio)

        # Normalize audio
        processed_audio = self.normalize_audio(processed_audio)

        # Trim silence (optional - can be skipped if real-time processing needed)
        # processed_audio = self.vad_simple(processed_audio)

        return processed_audio

    def get_audio_features(self, audio_data: np.ndarray) -> dict:
        """
        Extract basic audio features for quality assessment

        Args:
            audio_data: Input audio data

        Returns:
            Dictionary with audio features
        """
        features = {}

        # Basic statistics
        features['duration'] = len(audio_data) / self.sample_rate
        features['rms'] = np.sqrt(np.mean(audio_data ** 2))
        features['max_amplitude'] = np.max(np.abs(audio_data))
        features['zero_crossing_rate'] = np.mean(
            np.abs(np.diff(np.sign(audio_data))) / 2
        )

        # Energy-based features
        features['total_energy'] = np.sum(audio_data ** 2)
        features['energy_per_sample'] = features['total_energy'] / len(audio_data)

        # Peak-to-average ratio
        peak_energy = features['max_amplitude'] ** 2
        avg_energy = features['energy_per_sample']
        features['peak_to_average_ratio'] = peak_energy / avg_energy if avg_energy > 0 else 0

        return features


class RoboticAudioPreprocessor(AudioPreprocessor):
    """
    Extended audio preprocessor with robotic-specific features
    """

    def __init__(self, sample_rate: int = 16000, target_level: float = 0.1):
        super().__init__(sample_rate, target_level)
        self.robot_noise_profile = None  # Could store robot-specific noise patterns

    def preprocess_for_robot_environment(self, audio_data: np.ndarray, original_sr: int = None) -> np.ndarray:
        """
        Preprocess audio specifically for robotic environments
        This accounts for potential robot noise, mechanical sounds, etc.

        Args:
            audio_data: Input audio data
            original_sr: Original sample rate

        Returns:
            Preprocessed audio data optimized for robotic environments
        """
        processed_audio = audio_data.copy()

        # Remove DC offset
        processed_audio = self.remove_dc_offset(processed_audio)

        # Resample if needed
        if original_sr is not None and original_sr != self.sample_rate:
            processed_audio = self.resample_audio(processed_audio, original_sr)

        # Apply robot-specific noise reduction
        # In a real implementation, this would use learned noise profiles
        processed_audio = self.apply_noise_reduction(processed_audio, noise_threshold=0.015)

        # Apply bandpass filter (speech frequencies)
        processed_audio = self.apply_bandpass_filter(processed_audio, low_freq=100, high_freq=8000)

        # Normalize
        processed_audio = self.normalize_audio(processed_audio)

        # Apply preemphasis to enhance speech clarity
        processed_audio = self.apply_preemphasis(processed_audio)

        return processed_audio

    def detect_robot_noise(self, audio_data: np.ndarray, noise_threshold: float = 0.05) -> bool:
        """
        Detect if audio contains robot-specific noise

        Args:
            audio_data: Input audio data
            noise_threshold: Threshold for noise detection

        Returns:
            True if robot noise is detected
        """
        # Calculate high-frequency energy (robot noise often in higher frequencies)
        sos = signal.butter(10, 4000, 'high', fs=self.sample_rate, output='sos')
        high_freq_audio = signal.sosfilt(sos, audio_data)

        high_freq_energy = np.sqrt(np.mean(high_freq_audio ** 2))

        # If high-frequency energy is above threshold, likely robot noise
        return high_freq_energy > noise_threshold

    def adaptive_preprocessing(self, audio_data: np.ndarray, original_sr: int = None) -> Tuple[np.ndarray, dict]:
        """
        Apply adaptive preprocessing based on audio characteristics

        Args:
            audio_data: Input audio data
            original_sr: Original sample rate

        Returns:
            Tuple of (preprocessed_audio, processing_info)
        """
        processing_info = {
            'robot_noise_detected': False,
            'applied_filters': [],
            'original_duration': len(audio_data) / self.sample_rate
        }

        processed_audio = audio_data.copy()

        # Check for robot noise
        if self.detect_robot_noise(processed_audio):
            processing_info['robot_noise_detected'] = True
            processed_audio = self.preprocess_for_robot_environment(processed_audio, original_sr)
            processing_info['applied_filters'].append('robot_environment')
        else:
            processed_audio = self.preprocess_for_whisper(processed_audio, original_sr)
            processing_info['applied_filters'].append('whisper_optimized')

        # Extract features for quality assessment
        features = self.get_audio_features(processed_audio)
        processing_info['features'] = features

        return processed_audio, processing_info


# Example usage and testing
def main():
    """
    Example usage of the audio preprocessing functions
    """
    print("Testing Audio Preprocessing...")

    # Initialize preprocessor
    preprocessor = RoboticAudioPreprocessor()

    # Create sample audio data (simulated)
    sample_rate = 44100  # Different from target to test resampling
    duration = 2  # seconds
    t = np.linspace(0, duration, int(sample_rate * duration), endpoint=False)
    # Create a simple simulated audio signal with some noise
    audio_data = np.sin(2 * np.pi * 440 * t).astype(np.float32)  # 440Hz tone
    audio_data += 0.1 * np.random.normal(0, 1, len(audio_data))  # Add some noise

    print(f"Original audio shape: {audio_data.shape}")
    print(f"Original sample rate: {sample_rate}Hz")

    # Test basic preprocessing
    processed_audio = preprocessor.preprocess_for_whisper(audio_data, original_sr=sample_rate)
    print(f"Processed audio shape: {processed_audio.shape}")
    print(f"Processed sample rate: {preprocessor.sample_rate}Hz")

    # Test features extraction
    features = preprocessor.get_audio_features(processed_audio)
    print(f"Audio features: {features}")

    # Test adaptive preprocessing
    adaptive_result, info = preprocessor.adaptive_preprocessing(audio_data, original_sr=sample_rate)
    print(f"Adaptive processing info: {info}")

    print("Audio preprocessing test completed successfully!")


if __name__ == "__main__":
    main()