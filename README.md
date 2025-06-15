# AERON
Aritificial Enhanced Robotic Operational Nexus

AERON is a fully autonomous AI robot inspired by the iconic **TARS** from *Interstellar*. This project demonstrates how advanced AI, speech interaction, and robotics can run entirely **offline**, with **no cloud**, **no external APIs**, and **no internet dependency**.

🚀 What is AERON?

AERON is built to showcase edge-deployed conversational intelligence**. It processes speech, understands commands, replies like a human, and moves—**all on-device** using Jetson Nano or Raspberry Pi.

Unlike typical AI assistants that depend on cloud-based services, AERON runs a **custom LLM (Language Model)** trained on a **micro-dataset**, supported by:
- A homemade tokenizer and vector embedding engine
- A transformer-based encoder-decoder architecture
- ROS (Robot Operating System) for robot motion and simulation

## 🧠 Key Features

- 🗣️ **Offline Speech Interface**  
  Uses eSpeak and custom wavefile manipulation for both STT (speech-to-text) and TTS (text-to-speech).

- 🔤 **Custom Language Model**  
  A handcrafted transformer model built from scratch (no huggingface, no APIs) using:
  - Subword-level encoding
  - Dynamic tokenizer
  - Self-attention and positional encodings

- 🤖 **ROS Integration**  
  ROS handles all robot movements and simulations. NLP outputs are mapped to motion commands.

- 🔁 **Intent Parsing**  
  Spoken input is parsed and understood using a rule-based intent system backed by our LLM.

- 💡 **Edge-Optimized**  
  Runs entirely on devices like Jetson Nano or Raspberry Pi, with no cloud connection.

## 🧪 Use Cases Demonstrated

- Voice-commanded navigation (e.g., "move forward", "turn left")
- Smart verbal responses
- Live simulation using ROS nodes
- Full conversational loop on local hardware

## 🛠️ Technologies Used

- `Python`
- `PyTorch` (for model)
- `Wavefile IO` (for STT processing)
- `ROS` (for robotic simulation)
- `Jetson Nano`  (deployment)
