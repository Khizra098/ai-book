#!/usr/bin/env python3
"""
Basic Voice Recognition Example for VLA (Vision-Language-Action) System

This example demonstrates the fundamental components of voice recognition
using OpenAI Whisper API for humanoid robotics applications.

To run this example, you need:
1. OpenAI API key set in environment variable OPENAI_API_KEY
2. Required Python packages installed
3. Audio file to transcribe (or microphone access)

Installation:
pip install openai python-dotenv pyaudio SpeechRecognition

Usage:
python basic_voice_recognition.py [audio_file_path]
"""

import openai
import os
import sys
import time
import json
from datetime import datetime
from dataclasses import dataclass
from typing import Optional, Dict, Any
import argparse

# Optional imports for microphone support
try:
    import speech_recognition as sr
    MIC_AVAILABLE = True
except ImportError:
    MIC_AVAILABLE = False
    print("Warning: speech_recognition not installed. Microphone input disabled.")


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


class BasicVoiceRecognizer:
    """
    Basic voice recognition system using OpenAI Whisper API
    """
    def __init__(self, api_key: str):
        openai.api_key = api_key
        self.command_history = []

    def transcribe_audio_file(self, audio_file_path: str) -> VoiceCommand:
        """
        Transcribe an audio file using Whisper API
        """
        start_time = time.time()

        # Verify file exists
        if not os.path.exists(audio_file_path):
            raise FileNotFoundError(f"Audio file not found: {audio_file_path}")

        try:
            # Transcribe using Whisper API
            with open(audio_file_path, "rb") as audio_file:
                result = openai.Audio.transcribe(
                    model="whisper-1",
                    file=audio_file,
                    response_format="verbose_json",
                    timestamp_granularities=["segment"]
                )

            # Calculate a basic confidence score
            confidence = self._estimate_confidence(result)

            # Create voice command object
            command = VoiceCommand(
                id=f"cmd_{int(time.time())}",
                audio_path=audio_file_path,
                transcript=result.text,
                confidence=confidence,
                timestamp=datetime.now(),
                source="file"
            )

            # Validate command quality
            command.status = "VALID" if confidence > 0.5 else "LOW_CONFIDENCE"

            # Add to history
            self.command_history.append(command)

            processing_time = time.time() - start_time
            print(f"Transcription completed in {processing_time:.2f} seconds")
            print(f"Confidence: {confidence:.2f}")
            print(f"Transcript: {result.text}")

            return command

        except Exception as e:
            print(f"Error during transcription: {str(e)}")
            raise

    def _estimate_confidence(self, result) -> float:
        """
        Estimate confidence in the transcription result
        """
        # Use the average log probability from Whisper as a confidence proxy
        avg_logprob = getattr(result, 'avg_logprob', -1.0)

        # Convert log probability to confidence score (0-1 range)
        # Negative log probabilities closer to 0 indicate higher confidence
        confidence = max(0.0, min(1.0, (-avg_logprob + 2) / 4))

        return confidence

    def transcribe_from_microphone(self) -> Optional[VoiceCommand]:
        """
        Transcribe speech from microphone using Whisper API
        """
        if not MIC_AVAILABLE:
            print("Microphone input not available. speech_recognition package not installed.")
            return None

        print("Please speak now... (listening for 5 seconds)")

        # Use speech_recognition to capture audio
        recognizer = sr.Recognizer()
        microphone = sr.Microphone()

        try:
            with microphone as source:
                recognizer.adjust_for_ambient_noise(source)
                audio = recognizer.listen(source, timeout=5, phrase_time_limit=5)

            # Save audio to temporary file for Whisper API
            temp_file = "temp_microphone_input.wav"
            with open(temp_file, "wb") as f:
                f.write(audio.get_wav_data())

            print("Processing microphone input...")
            command = self.transcribe_audio_file(temp_file)

            # Clean up temporary file
            if os.path.exists(temp_file):
                os.remove(temp_file)

            return command

        except sr.WaitTimeoutError:
            print("No speech detected within timeout period")
            return None
        except Exception as e:
            print(f"Error capturing microphone input: {str(e)}")
            return None

    def get_command_summary(self) -> Dict[str, Any]:
        """
        Get summary of processed commands
        """
        if not self.command_history:
            return {"total_commands": 0}

        total_confidence = sum(cmd.confidence for cmd in self.command_history)
        avg_confidence = total_confidence / len(self.command_history)

        return {
            "total_commands": len(self.command_history),
            "average_confidence": avg_confidence,
            "successful_transcriptions": len([c for c in self.command_history if c.status == "VALID"])
        }


def main():
    """
    Main function to demonstrate basic voice recognition
    """
    parser = argparse.ArgumentParser(description="Basic Voice Recognition Example")
    parser.add_argument("audio_file", nargs="?", help="Path to audio file to transcribe")
    parser.add_argument("--mic", action="store_true", help="Use microphone input")

    args = parser.parse_args()

    # Check for API key
    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        print("Error: OPENAI_API_KEY environment variable not set.")
        print("Please set your OpenAI API key as an environment variable.")
        return 1

    # Initialize voice recognizer
    recognizer = BasicVoiceRecognizer(api_key)

    if args.mic and MIC_AVAILABLE:
        # Use microphone input
        command = recognizer.transcribe_from_microphone()
        if command:
            print(f"\nRecognized command: {command.transcript}")
            print(f"Confidence: {command.confidence:.2f}")
    elif args.audio_file:
        # Use provided audio file
        try:
            command = recognizer.transcribe_audio_file(args.audio_file)
            print(f"\nRecognized command: {command.transcript}")
            print(f"Confidence: {command.confidence:.2f}")
        except FileNotFoundError:
            print(f"Error: Audio file not found: {args.audio_file}")
            return 1
    else:
        # No input provided - show usage
        print("Usage:")
        print("  python basic_voice_recognition.py [audio_file_path]")
        print("  python basic_voice_recognition.py --mic")
        print("\nExample with sample file:")
        print("  python basic_voice_recognition.py sample_command.wav")
        print("\nExample with microphone:")
        print("  python basic_voice_recognition.py --mic")
        return 0

    # Print summary
    summary = recognizer.get_command_summary()
    print(f"\nSession Summary:")
    print(f"  Total commands processed: {summary['total_commands']}")
    if summary['total_commands'] > 0:
        print(f"  Average confidence: {summary['average_confidence']:.2f}")
        print(f"  Successful transcriptions: {summary['successful_transcriptions']}")

    return 0


if __name__ == "__main__":
    sys.exit(main())