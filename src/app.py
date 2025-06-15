# 🔧 Install dependencies
!pip install -q torch transformers soundfile datasets librosa llama-cpp-python flask

# 🔁 Imports
from transformers import (
    AutoTokenizer,
    AutoModelForCausalLM,
    WhisperProcessor,
    WhisperForConditionalGeneration,
    VitsModel,
    AutoTokenizer as TTSAutoTokenizer
)
import torch
import librosa
import soundfile as sf
import numpy as np
from IPython.display import Audio, display
from google.colab import files

import io
import gc
import logging

# 📝 Initialize logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# 🔌 Load models
def load_models():
    device = "cuda" if torch.cuda.is_available() else "cpu"
    logger.info(f"Using device: {device}")

    models = {}

    try:
        # Whisper STT
        logger.info("Loading Whisper model...")
        models['whisper_processor'] = WhisperProcessor.from_pretrained("openai/whisper-medium")
        models['whisper_model'] = WhisperForConditionalGeneration.from_pretrained(
            "openai/whisper-medium",
            torch_dtype=torch.float16 if device == "cuda" else torch.float32,
            device_map="auto"
        )

        # TinyLlama for generation
        logger.info("Loading TinyLlama model...")
        models['llm_tokenizer'] = AutoTokenizer.from_pretrained("TinyLlama/TinyLlama-1.1B-intermediate-step-1431k-3T")
        models['llm_model'] = AutoModelForCausalLM.from_pretrained(
            "TinyLlama/TinyLlama-1.1B-intermediate-step-1431k-3T",
            torch_dtype=torch.float16 if device == "cuda" else torch.float32,
            device_map="auto"
        )

        # VITS TTS
        logger.info("Loading TTS model...")
        models['tts_model'] = VitsModel.from_pretrained("facebook/mms-tts-eng")
        models['tts_tokenizer'] = TTSAutoTokenizer.from_pretrained("facebook/mms-tts-eng")

        return models

    except Exception as e:
        logger.error(f"Error loading models: {str(e)}")
        raise

# ✅ Load all models
models = load_models()

# 🎙️ Transcription
def transcribe(audio_path):
    audio, sr = librosa.load(audio_path, sr=16000)
    inputs = models['whisper_processor'](
        audio,
        sampling_rate=sr,
        return_tensors="pt"
    ).input_features.to(models['whisper_model'].device)

    predicted_ids = models['whisper_model'].generate(inputs)
    return models['whisper_processor'].batch_decode(predicted_ids, skip_special_tokens=True)[0]

# 📚 Summarization
def summarize(text):
    prompt = f"Summarize the following text:\n{text}\n\nSummary:"
    inputs = models['llm_tokenizer'](prompt, return_tensors="pt").to(models['llm_model'].device)
    outputs = models['llm_model'].generate(
        **inputs,
        max_new_tokens=100,
        temperature=0.7,
        top_p=0.9,
        do_sample=True
    )
    return models['llm_tokenizer'].decode(outputs[0], skip_special_tokens=True)

# ❓ QnA
def answer_question(context, question):
    prompt = f"Context:\n{context}\n\nQuestion: {question}\nAnswer:"
    inputs = models['llm_tokenizer'](prompt, return_tensors="pt").to(models['llm_model'].device)
    outputs = models['llm_model'].generate(
        **inputs,
        max_new_tokens=100,
        temperature=0.7,
        top_p=0.9,
        do_sample=True
    )
    return models['llm_tokenizer'].decode(outputs[0], skip_special_tokens=True)

# 🔊 TTS
def text_to_speech(text):
    inputs = models['tts_tokenizer'](text, return_tensors="pt")
    with torch.no_grad():
        output = models['tts_model'](**inputs).waveform
    return output.numpy()

# ⏯️ Upload and choose operation
uploaded = files.upload()
audio_path = next(iter(uploaded))

print("\n🔊 Transcribing...")
transcription = transcribe(audio_path)
print(f"🗣️ Transcription:\n{transcription}")

# Choose mode
print("\nChoose mode:")
print("1. Summarization")
print("2. Question Answering")
mode = input("Enter 1 or 2: ").strip()

if mode == "1":
    print("\n📚 Generating summary...")
    result = summarize(transcription)

elif mode == "2":
    question = input("❓ Enter your question based on the audio: ")
    print("\n💬 Generating answer...")
    result = answer_question(transcription, question)

else:
    print("⚠️ Invalid mode. Defaulting to summarization.")
    result = summarize(transcription)

# Show result
print("\n✅ Result:\n", result)

# 🔈 Convert to speech
print("\n🔈 Converting result to audio...")
speech_output = text_to_speech(result)
display(Audio(speech_output, rate=16000))

# 🧹 Cleanup
gc.collect()
if torch.cuda.is_available():
    torch.cuda.empty_cache()
