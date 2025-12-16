---
title: Voice-to-Action with OpenAI Whisper
description: Comprehensive guide to using OpenAI Whisper for voice command recognition in humanoid robotics applications
sidebar_label: Voice-to-Action with Whisper
---

# Voice-to-Action with OpenAI Whisper

## Introduction to Voice Command Recognition

Voice command recognition is a fundamental capability for creating intuitive human-robot interaction in humanoid robotics. The ability to convert spoken natural language commands into actionable robot tasks enables more natural and accessible control mechanisms. This chapter explores how OpenAI Whisper can be leveraged to create robust voice recognition systems for humanoid robots.

OpenAI Whisper is a state-of-the-art speech recognition model that excels at converting audio to text with high accuracy across multiple languages and accents. For humanoid robotics applications, Whisper provides the foundation for understanding spoken commands and converting them into structured action parameters that robots can process and execute.

### Key Benefits for Robotics

- **High Accuracy**: Whisper achieves exceptional accuracy even in challenging acoustic environments
- **Multi-language Support**: Works with numerous languages and dialects
- **Robustness**: Handles various audio qualities and background noise conditions
- **API Integration**: Easy to integrate via OpenAI's API or self-hosted models
- **Real-time Processing**: Can be optimized for real-time voice command processing

### Architecture Overview

The voice-to-action pipeline involves several key components:

1. **Audio Input**: Capturing voice commands from microphones or other audio sources
2. **Preprocessing**: Cleaning and preparing audio for recognition
3. **Speech Recognition**: Using Whisper to convert speech to text
4. **Command Parsing**: Interpreting the recognized text for robot action
5. **Action Execution**: Converting parsed commands into robot actions

## OpenAI Whisper Integration and Setup

### Prerequisites

Before integrating OpenAI Whisper into your humanoid robot system, ensure you have:

- An OpenAI API key (for API-based approach)
- Python 3.8 or higher
- Required Python packages installed
- Proper network connectivity for API calls (or local model for offline approach)

### Installation and Dependencies

```bash
pip install openai
pip install python-dotenv  # For managing API keys securely
pip install pyaudio        # For audio capture
pip install SpeechRecognition  # Alternative speech recognition library
```

### Basic Whisper Integration

```python
import openai
import os
from dotenv import load_dotenv
import speech_recognition as sr

# Load environment variables
load_dotenv()

# Set up OpenAI API key
openai.api_key = os.getenv("OPENAI_API_KEY")

def transcribe_audio_with_whisper(audio_file_path):
    """
    Transcribe audio using OpenAI Whisper API
    """
    with open(audio_file_path, "rb") as audio_file:
        transcript = openai.Audio.transcribe("whisper-1", audio_file)
    return transcript.text

def real_time_transcription():
    """
    Real-time transcription using microphone input
    """
    recognizer = sr.Recognizer()
    microphone = sr.Microphone()

    # Adjust for ambient noise
    with microphone as source:
        recognizer.adjust_for_ambient_noise(source)
        print("Listening for voice command...")
        audio = recognizer.listen(source)

    # Save audio to temporary file for Whisper API
    with open("temp_audio.wav", "wb") as f:
        f.write(audio.get_wav_data())

    # Transcribe using Whisper
    transcript = transcribe_audio_with_whisper("temp_audio.wav")

    # Clean up temporary file
    os.remove("temp_audio.wav")

    return transcript
```

### Configuration Options

Whisper provides several configuration options to optimize performance for your specific use case:

```python
def transcribe_with_options(audio_file_path, language="en", response_format="json", temperature=0):
    """
    Transcribe audio with specific options
    """
    with open(audio_file_path, "rb") as audio_file:
        transcript = openai.Audio.transcribe(
            model="whisper-1",
            file=audio_file,
            language=language,  # Specify language for better accuracy
            response_format=response_format,  # json, text, srt, verbose_json, vtt
            temperature=temperature  # Controls randomness (0-1)
        )
    return transcript
```

## Audio Preprocessing and Noise Reduction Techniques

### Importance of Audio Quality

High-quality audio input is crucial for accurate speech recognition. Poor audio quality can significantly impact the performance of Whisper and other speech recognition systems. Effective preprocessing helps improve recognition accuracy, especially in real-world environments where background noise is common.

### Noise Reduction Approaches

#### 1. Real-time Noise Reduction

```python
import numpy as np
import scipy.signal as signal
from scipy.io import wavfile

def reduce_noise_real_time(audio_data, sample_rate):
    """
    Apply noise reduction to audio data
    """
    # Apply a high-pass filter to remove low-frequency noise
    nyquist = sample_rate / 2
    cutoff_freq = 100  # Hz
    normal_cutoff = cutoff_freq / nyquist

    # Create and apply Butterworth filter
    b, a = signal.butter(4, normal_cutoff, btype='high', analog=False)
    filtered_audio = signal.filtfilt(b, a, audio_data)

    return filtered_audio

def preprocess_audio_for_recognition(audio_file_path):
    """
    Preprocess audio file for optimal Whisper recognition
    """
    sample_rate, audio_data = wavfile.read(audio_file_path)

    # Convert to mono if stereo
    if len(audio_data.shape) > 1:
        audio_data = audio_data.mean(axis=1)

    # Apply noise reduction
    processed_audio = reduce_noise_real_time(audio_data, sample_rate)

    # Normalize audio levels
    processed_audio = processed_audio / np.max(np.abs(processed_audio))

    # Write processed audio to temporary file
    temp_path = "temp_processed.wav"
    wavfile.write(temp_path, sample_rate, (processed_audio * 32767).astype(np.int16))

    return temp_path
```

#### 2. Using Librosa for Advanced Preprocessing

```python
import librosa
import soundfile as sf

def advanced_audio_preprocessing(audio_file_path):
    """
    Advanced audio preprocessing using librosa
    """
    # Load audio file
    y, sr = librosa.load(audio_file_path, sr=None)

    # Apply noise reduction using spectral gating
    # This technique helps remove stationary background noise
    y_denoised = librosa.effects.remix(y, intervals=librosa.effects.split(y))

    # Normalize the audio
    y_normalized = librosa.util.normalize(y_denoised)

    # Write to temporary file for Whisper processing
    temp_path = "temp_advanced_processed.wav"
    sf.write(temp_path, y_normalized, sr)

    return temp_path
```

#### 3. Real-time Preprocessing Pipeline

```python
import pyaudio
import numpy as np
import threading
import queue

class AudioProcessor:
    def __init__(self, chunk_size=1024, sample_rate=16000):
        self.chunk_size = chunk_size
        self.sample_rate = sample_rate
        self.audio_queue = queue.Queue()
        self.is_recording = False

    def start_recording(self):
        """
        Start real-time audio recording with preprocessing
        """
        self.is_recording = True
        audio_thread = threading.Thread(target=self._record_audio)
        audio_thread.start()

    def _record_audio(self):
        """
        Internal method for audio recording and preprocessing
        """
        p = pyaudio.PyAudio()

        stream = p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_size
        )

        while self.is_recording:
            # Read audio chunk
            data = stream.read(self.chunk_size)
            audio_chunk = np.frombuffer(data, dtype=np.int16)

            # Apply preprocessing (simple noise thresholding example)
            processed_chunk = self._preprocess_chunk(audio_chunk)

            # Add to queue for further processing
            self.audio_queue.put(processed_chunk)

        stream.stop_stream()
        stream.close()
        p.terminate()

    def _preprocess_chunk(self, audio_chunk):
        """
        Preprocess a single audio chunk
        """
        # Simple noise floor reduction
        threshold = 500  # Adjust based on your environment
        audio_chunk[np.abs(audio_chunk) < threshold] = 0

        return audio_chunk

    def stop_recording(self):
        """
        Stop audio recording
        """
        self.is_recording = False
```

## Speech-to-Text Conversion Workflow

### Basic Workflow

The speech-to-text conversion workflow for humanoid robotics involves several key steps:

1. **Audio Capture**: Recording voice commands from the environment
2. **Preprocessing**: Cleaning and preparing the audio for recognition
3. **Transcription**: Converting speech to text using Whisper
4. **Validation**: Checking the quality and confidence of the transcription
5. **Processing**: Preparing the text for command interpretation

### Complete Voice Recognition Pipeline

```python
import openai
import os
import time
import json
from datetime import datetime
from dataclasses import dataclass
from typing import Optional, Dict, Any

@dataclass
class VoiceCommand:
    """
    Represents a voice command with metadata
    """
    id: str
    audio_path: str
    transcript: str
    confidence: float
    timestamp: datetime
    source: str
    status: str = "RECEIVED"
    user_context: Optional[Dict[str, Any]] = None

class VoiceRecognitionPipeline:
    def __init__(self, api_key: str):
        openai.api_key = api_key
        self.command_history = []

    def capture_and_recognize(self, audio_file_path: str, user_context: Dict[str, Any] = None) -> VoiceCommand:
        """
        Complete pipeline for capturing and recognizing voice commands
        """
        start_time = time.time()

        # Step 1: Preprocess audio
        processed_audio_path = preprocess_audio_for_recognition(audio_file_path)

        # Step 2: Transcribe with Whisper
        transcript = self.transcribe_with_confidence(processed_audio_path)

        # Step 3: Calculate confidence score
        confidence = self.estimate_confidence(transcript, audio_file_path)

        # Step 4: Create voice command object
        command = VoiceCommand(
            id=f"cmd_{int(time.time())}",
            audio_path=audio_file_path,
            transcript=transcript,
            confidence=confidence,
            timestamp=datetime.now(),
            source="microphone" if "temp" in audio_file_path else "file",
            user_context=user_context
        )

        # Step 5: Validate command
        if confidence > 0.7:  # Threshold for acceptable confidence
            command.status = "VALID"
        else:
            command.status = "LOW_CONFIDENCE"

        # Step 6: Add to history
        self.command_history.append(command)

        # Step 7: Clean up temporary files
        if "temp" in processed_audio_path:
            os.remove(processed_audio_path)

        processing_time = time.time() - start_time
        print(f"Voice recognition completed in {processing_time:.2f} seconds")

        return command

    def transcribe_with_confidence(self, audio_file_path: str) -> str:
        """
        Transcribe audio and return transcript
        """
        with open(audio_file_path, "rb") as audio_file:
            result = openai.Audio.transcribe(
                model="whisper-1",
                file=audio_file,
                response_format="verbose_json",
                timestamp_granularities=["segment"]
            )
        return result.text

    def estimate_confidence(self, transcript: str, audio_file_path: str) -> float:
        """
        Estimate confidence in the transcription
        This is a simplified approach - in practice, you might use more sophisticated methods
        """
        # Simple heuristic: longer transcripts with common words might be more reliable
        if len(transcript.strip()) < 3:
            return 0.1  # Very low confidence for very short transcripts

        # Check for common robot command keywords
        common_keywords = ["move", "go", "turn", "stop", "pick", "place", "follow"]
        keyword_count = sum(1 for word in common_keywords if word.lower() in transcript.lower())

        # Base confidence on keyword presence and transcript length
        base_confidence = min(0.5 + (keyword_count * 0.2), 1.0)

        # Additional factors could include audio quality metrics, etc.
        return base_confidence

# Example usage
def example_voice_recognition():
    """
    Example of using the voice recognition pipeline
    """
    # Initialize pipeline
    pipeline = VoiceRecognitionPipeline(api_key=os.getenv("OPENAI_API_KEY"))

    # Simulate voice command recognition
    # In a real scenario, this would come from microphone input
    command = pipeline.capture_and_recognize("sample_command.wav")

    print(f"Recognized: {command.transcript}")
    print(f"Confidence: {command.confidence:.2f}")
    print(f"Status: {command.status}")

    return command
```

### Streaming Recognition for Real-time Applications

For humanoid robots that need to respond to voice commands in real-time, streaming recognition can be more appropriate:

```python
import asyncio
import websockets
import json
import base64

class StreamingVoiceRecognizer:
    def __init__(self, api_key: str):
        self.api_key = api_key
        self.is_listening = False

    async def start_streaming_recognition(self, audio_stream):
        """
        Start streaming recognition for real-time voice commands
        Note: This is a conceptual example - actual implementation would depend on
        specific streaming Whisper services or self-hosted models
        """
        # This would connect to a streaming Whisper service
        # For now, showing the conceptual approach
        pass

    def process_streaming_chunk(self, audio_chunk):
        """
        Process a chunk of streaming audio
        """
        # Process audio chunk and return partial results
        # This enables real-time feedback during speech
        pass
```

## Error Handling and Confidence Scoring

### Importance of Confidence Scoring

Confidence scoring is critical for humanoid robot applications where incorrect command interpretation can lead to unsafe or undesired behaviors. Proper confidence scoring helps the system determine when to ask for clarification or reject low-confidence commands.

### Confidence Scoring Implementation

```python
class ConfidenceScorer:
    @staticmethod
    def calculate_comprehensive_confidence(
        transcript: str,
        audio_features: Dict[str, float],
        language_model_score: float
    ) -> Dict[str, float]:
        """
        Calculate comprehensive confidence score based on multiple factors
        """
        scores = {}

        # Transcript-based confidence
        scores['transcript_confidence'] = ConfidenceScorer._transcript_confidence(transcript)

        # Audio quality confidence
        scores['audio_confidence'] = ConfidenceScorer._audio_confidence(audio_features)

        # Language model confidence
        scores['language_confidence'] = language_model_score

        # Combined confidence
        scores['combined_confidence'] = (
            scores['transcript_confidence'] * 0.4 +
            scores['audio_confidence'] * 0.3 +
            scores['language_confidence'] * 0.3
        )

        return scores

    @staticmethod
    def _transcript_confidence(transcript: str) -> float:
        """
        Calculate confidence based on transcript characteristics
        """
        if not transcript.strip():
            return 0.0

        # Length-based confidence
        length_score = min(len(transcript) / 100, 0.8)  # Up to 0.8 for long transcripts

        # Check for common robot command patterns
        command_patterns = [
            r"\b(move|go|turn|stop|pick|place|follow|bring|get)\b",
            r"\b(forward|backward|left|right|up|down)\b",
            r"\b(meters|cm|feet)\b"
        ]

        pattern_score = 0.0
        for pattern in command_patterns:
            import re
            if re.search(pattern, transcript, re.IGNORECASE):
                pattern_score += 0.2

        return min(length_score + pattern_score, 1.0)

    @staticmethod
    def _audio_confidence(audio_features: Dict[str, float]) -> float:
        """
        Calculate confidence based on audio quality features
        """
        # Example audio features: signal-to-noise ratio, volume level, etc.
        snr = audio_features.get('snr', 0)
        volume = audio_features.get('volume', 0)

        # Normalize and combine audio features
        snr_score = min(snr / 30, 1.0)  # Assuming 30dB is excellent SNR
        volume_score = 0.5 if 0.1 <= volume <= 0.9 else 0.2  # Optimal volume range

        return (snr_score * 0.6 + volume_score * 0.4)

# Integration with voice recognition pipeline
def enhanced_voice_recognition_with_confidence(audio_file_path: str, api_key: str):
    """
    Enhanced voice recognition with comprehensive confidence scoring
    """
    # Perform initial transcription
    openai.api_key = api_key

    with open(audio_file_path, "rb") as audio_file:
        result = openai.Audio.transcribe(
            model="whisper-1",
            file=audio_file,
            response_format="verbose_json"
        )

    transcript = result.text

    # Extract audio features (this would be done with audio analysis libraries)
    audio_features = {
        'snr': 25.0,  # Example SNR value
        'volume': 0.6,  # Example volume level
    }

    # Calculate confidence scores
    scorer = ConfidenceScorer()
    confidence_scores = scorer.calculate_comprehensive_confidence(
        transcript,
        audio_features,
        result.get('avg_logprob', -0.5)  # Example language model score
    )

    # Create enhanced voice command
    command = VoiceCommand(
        id=f"enhanced_cmd_{int(time.time())}",
        audio_path=audio_file_path,
        transcript=transcript,
        confidence=confidence_scores['combined_confidence'],
        timestamp=datetime.now(),
        source="file"
    )

    return command, confidence_scores
```

### Error Handling Strategies

```python
from enum import Enum
from typing import List

class RecognitionError(Enum):
    API_ERROR = "api_error"
    AUDIO_QUALITY_LOW = "audio_quality_low"
    CONFIDENCE_TOO_LOW = "confidence_too_low"
    UNKNOWN_COMMAND = "unknown_command"
    TIMEOUT = "timeout"

class VoiceRecognitionErrorHandler:
    def __init__(self):
        self.error_history = []

    def handle_recognition_error(self, error_type: RecognitionError, details: str = ""):
        """
        Handle different types of recognition errors
        """
        error_entry = {
            'timestamp': datetime.now(),
            'error_type': error_type.value,
            'details': details,
            'suggested_action': self._get_suggested_action(error_type)
        }

        self.error_history.append(error_entry)

        # Log error and take appropriate action
        print(f"Recognition error: {error_type.value}")
        print(f"Suggested action: {error_entry['suggested_action']}")

        return error_entry

    def _get_suggested_action(self, error_type: RecognitionError) -> str:
        """
        Get suggested action for each error type
        """
        actions = {
            RecognitionError.API_ERROR: "Check API key and network connectivity",
            RecognitionError.AUDIO_QUALITY_LOW: "Improve microphone placement or reduce background noise",
            RecognitionError.CONFIDENCE_TOO_LOW: "Ask user to repeat command more clearly",
            RecognitionError.UNKNOWN_COMMAND: "Provide list of recognized commands to user",
            RecognitionError.TIMEOUT: "Adjust timeout parameters or check system performance"
        }

        return actions.get(error_type, "No specific action recommended")

    def get_error_summary(self) -> Dict[str, int]:
        """
        Get summary of error types encountered
        """
        summary = {}
        for error in self.error_history:
            error_type = error['error_type']
            summary[error_type] = summary.get(error_type, 0) + 1

        return summary
```

## Official Whisper API Documentation References

For comprehensive information about OpenAI Whisper, refer to the official documentation:

- [OpenAI Whisper API Reference](https://platform.openai.com/docs/api-reference/audio)
- [Whisper Model Card](https://github.com/openai/whisper)
- [Speech Recognition Best Practices](https://platform.openai.com/docs/guides/speech-recognition)

### Key Technical Resources

- **OpenAI API Documentation**: Detailed API specifications and parameters
- **Whisper GitHub Repository**: Source code and implementation details
- **Audio Processing Guidelines**: Best practices for audio input optimization
- **Rate Limits and Pricing**: Important information for production deployment

### Recommended Learning Path

1. Start with the OpenAI API quickstart guide
2. Follow the speech recognition tutorials
3. Review the technical papers on Whisper's architecture
4. Explore the examples for robotics applications

## Configuration Examples with Proper Syntax Highlighting

### Basic Whisper API Configuration

```python
# Configuration for Whisper API integration
import openai
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

class WhisperConfig:
    def __init__(self):
        # API Configuration
        self.api_key = os.getenv("OPENAI_API_KEY")
        self.model = "whisper-1"
        self.default_language = "en"
        self.response_format = "json"
        self.temperature = 0.0
        self.timeout = 30  # seconds

        # Initialize OpenAI
        openai.api_key = self.api_key

    def transcribe_audio(self, audio_file_path, language=None, response_format=None):
        """
        Transcribe audio with configurable parameters
        """
        with open(audio_file_path, "rb") as audio_file:
            result = openai.Audio.transcribe(
                model=self.model,
                file=audio_file,
                language=language or self.default_language,
                response_format=response_format or self.response_format,
                temperature=self.temperature
            )
        return result

# Example usage
config = WhisperConfig()
transcript = config.transcribe_audio("command.wav")
print(transcript.text)
```

### Advanced Configuration with Error Handling

```python
import openai
import time
import logging
from typing import Optional, Dict, Any

class AdvancedWhisperConfig:
    def __init__(self, api_key: str):
        openai.api_key = api_key
        self.setup_logging()

        # Configuration parameters
        self.model = "whisper-1"
        self.default_language = "en"
        self.response_format = "verbose_json"
        self.temperature = 0.0
        self.retry_attempts = 3
        self.retry_delay = 1  # seconds

        # Performance parameters
        self.timeout = 60
        self.max_file_size = 25 * 1024 * 1024  # 25MB limit for Whisper API

    def setup_logging(self):
        """
        Set up logging for the Whisper configuration
        """
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger(__name__)

    def robust_transcribe(self, audio_file_path: str, **kwargs) -> Optional[Dict[str, Any]]:
        """
        Robust transcription with retry logic and error handling
        """
        # Validate file size
        file_size = os.path.getsize(audio_file_path)
        if file_size > self.max_file_size:
            self.logger.error(f"File size {file_size} exceeds limit {self.max_file_size}")
            return None

        for attempt in range(self.retry_attempts):
            try:
                with open(audio_file_path, "rb") as audio_file:
                    result = openai.Audio.transcribe(
                        model=self.model,
                        file=audio_file,
                        **kwargs
                    )

                self.logger.info(f"Transcription successful on attempt {attempt + 1}")
                return result

            except openai.error.RateLimitError:
                self.logger.warning(f"Rate limit exceeded on attempt {attempt + 1}")
                if attempt < self.retry_attempts - 1:
                    time.sleep(self.retry_delay * (attempt + 1))
                else:
                    self.logger.error("Rate limit exceeded after all retries")
                    return None

            except openai.error.APIError as e:
                self.logger.error(f"API error on attempt {attempt + 1}: {str(e)}")
                if attempt < self.retry_attempts - 1:
                    time.sleep(self.retry_delay * (attempt + 1))
                else:
                    return None

            except Exception as e:
                self.logger.error(f"Unexpected error on attempt {attempt + 1}: {str(e)}")
                return None

        return None

# Example usage
config = AdvancedWhisperConfig(api_key=os.getenv("OPENAI_API_KEY"))
result = config.robust_transcribe(
    "command.wav",
    language="en",
    response_format="text"
)
```

### Configuration for Robotics Applications

```python
import openai
import asyncio
from typing import Callable, Awaitable

class RoboticsWhisperConfig:
    def __init__(self, api_key: str):
        openai.api_key = api_key

        # Robotics-specific configuration
        self.robot_commands = [
            "move forward", "move backward", "turn left", "turn right",
            "stop", "pick up", "place down", "follow me", "go to",
            "find object", "bring me", "dance", "wave", "sit", "stand"
        ]

        # Performance optimization for robotics
        self.continuous_listening = False
        self.wake_word_detection = True
        self.wake_word = "Hey Robot"

        # Safety parameters
        self.confidence_threshold = 0.7
        self.command_validation_enabled = True

    def validate_robot_command(self, transcript: str) -> bool:
        """
        Validate if transcript contains a valid robot command
        """
        transcript_lower = transcript.lower()

        # Check if transcript contains any known robot commands
        for command in self.robot_commands:
            if command in transcript_lower:
                return True

        # Additional validation could include grammar, intent classification, etc.
        return False

    async def process_robot_voice_command(self, audio_file_path: str) -> Dict[str, Any]:
        """
        Process voice command specifically for robotics applications
        """
        # Transcribe the audio
        result = openai.Audio.transcribe(
            model="whisper-1",
            file=open(audio_file_path, "rb"),
            response_format="verbose_json"
        )

        transcript = result.text
        confidence = result.avg_logprob  # Use log probability as confidence proxy

        # Validate command
        is_valid_command = self.validate_robot_command(transcript)

        # Prepare response
        response = {
            'transcript': transcript,
            'confidence': confidence,
            'is_valid_command': is_valid_command,
            'timestamp': time.time(),
            'robot_action': self.extract_robot_action(transcript) if is_valid_command else None
        }

        return response

    def extract_robot_action(self, transcript: str) -> Dict[str, Any]:
        """
        Extract structured robot action from transcript
        """
        # This would use NLP or regex to extract action parameters
        # For example: "Move forward 2 meters" -> {action: "move", direction: "forward", distance: 2.0}
        return {
            'raw_command': transcript,
            'action_type': 'navigation',  # or manipulation, interaction, etc.
            'parameters': {}
        }

# Example usage in robotics context
robot_config = RoboticsWhisperConfig(api_key=os.getenv("OPENAI_API_KEY"))
command_result = await robot_config.process_robot_voice_command("robot_command.wav")
print(f"Command: {command_result['transcript']}")
print(f"Valid: {command_result['is_valid_command']}")
```

## VoiceCommand Entity Implementation in Voice Recognition Context

The VoiceCommand entity, as defined in our data model, represents a spoken natural language instruction that requires interpretation and execution. In the context of voice recognition with OpenAI Whisper, this entity captures the essential information from the speech-to-text conversion process:

- **id**: Unique identifier for the command
- **audio_path**: Reference to the original audio file
- **transcript**: The text transcription produced by Whisper
- **confidence**: Confidence score of the speech recognition (0.0-1.0)
- **timestamp**: When the command was received and processed
- **source**: Whether it came from microphone, file, etc.
- **status**: Current status in the processing pipeline (RECEIVED, PROCESSING, etc.)
- **user_context**: Associated user information for personalized interaction

This entity serves as the bridge between the voice recognition component and the subsequent cognitive planning and action execution components, ensuring that all relevant information about the original voice command is preserved and accessible throughout the VLA pipeline.

## Real-World Examples and Use Cases for Voice-to-Action Conversion

### Example 1: Home Assistance Robot
A humanoid robot in a home environment receives the command: "Please bring me a glass of water from the kitchen."
- Whisper transcribes the audio to text with high confidence
- The cognitive planning system identifies the key components: "bring", "glass of water", "kitchen"
- The action execution system plans a path to the kitchen, locates a glass, grasps it, fills with water, and brings it back

### Example 2: Industrial Collaborative Robot
In a manufacturing setting: "Robot, please inspect the assembly on station 3 and report any defects."
- Whisper accurately transcribes the technical command
- The system recognizes the specific station and inspection task
- The robot navigates to the station, performs visual inspection, and reports results

### Example 3: Educational Robot
In a classroom: "Can you demonstrate how to pour liquid from one cup to another?"
- The robot recognizes the demonstration request
- It identifies the pouring action and locates appropriate containers
- The robot performs the demonstration while explaining the steps

These examples illustrate how effective voice-to-action conversion using OpenAI Whisper enables natural human-robot interaction across diverse application domains, making robots more accessible and intuitive to use.