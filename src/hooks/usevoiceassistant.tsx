import { useState, useCallback } from 'react';
import { Message, ProcessingState, ApiResponse } from '../types';

// Mock API responses for demonstration
const mockResponses = [
  "Hello! I'm your AI voice assistant. How can I help you today?",
  "That's an interesting question. Let me think about that for a moment.",
  "I understand what you're asking. Here's what I think about that topic.",
  "Thank you for sharing that with me. I appreciate your input.",
  "That's a great point. I'd be happy to discuss this further with you.",
  "I see what you mean. Let me provide you with some helpful information.",
  "Interesting perspective! I'd love to explore this topic more with you.",
  "That sounds like something worth considering. What are your thoughts on it?"
];

// Mock transcription for demo purposes
const mockTranscriptions = [
  "Hello, can you help me with something?",
  "What's the weather like today?",
  "Tell me about artificial intelligence",
  "How does voice recognition work?",
  "Can you explain machine learning?",
  "What are your capabilities?",
  "Help me understand this topic",
  "Thank you for your assistance"
];

export const useVoiceAssistant = () => {
  const [messages, setMessages] = useState<Message[]>([]);
  const [processingState, setProcessingState] = useState<ProcessingState>({
    isRecording: false,
    isProcessing: false,
    currentStep: 'idle',
    error: null
  });

  const processAudio = useCallback(async (audioBlob: Blob) => {
    const messageId = Date.now().toString();
    
    // Add user message placeholder
    const userMessage: Message = {
      id: messageId + '_user',
      type: 'user',
      text: 'Processing...',
      timestamp: new Date(),
      isProcessing: true
    };
    
    setMessages(prev => [...prev, userMessage]);
    
    // Set processing state - transcribing
    setProcessingState({
      isRecording: false,
      isProcessing: true,
      currentStep: 'transcribing',
      error: null
    });

    try {
      // Simulate transcription delay
      await new Promise(resolve => setTimeout(resolve, 1500));

      // Get random mock transcription
      const mockTranscription = mockTranscriptions[Math.floor(Math.random() * mockTranscriptions.length)];

      // Update processing state - generating response
      setProcessingState(prev => ({
        ...prev,
        currentStep: 'generating'
      }));

      // Simulate AI response generation delay
      await new Promise(resolve => setTimeout(resolve, 1000));

      // Update processing state - synthesizing speech
      setProcessingState(prev => ({
        ...prev,
        currentStep: 'synthesizing'
      }));

      // Simulate speech synthesis delay
      await new Promise(resolve => setTimeout(resolve, 800));

      // Update user message with mock transcription
      setMessages(prev => prev.map(msg => 
        msg.id === userMessage.id 
          ? { ...msg, text: mockTranscription, isProcessing: false }
          : msg
      ));

      // Get random mock response
      const mockResponse = mockResponses[Math.floor(Math.random() * mockResponses.length)];

      // Add assistant response
      const assistantMessage: Message = {
        id: messageId + '_assistant',
        type: 'assistant',
        text: mockResponse,
        timestamp: new Date()
      };

      setMessages(prev => [...prev, assistantMessage]);

      // Reset processing state
      setProcessingState({
        isRecording: false,
        isProcessing: false,
        currentStep: 'idle',
        error: null
      });

    } catch (error) {
      console.error('Error processing audio:', error);
      
      // Remove processing message and show error
      setMessages(prev => prev.filter(msg => msg.id !== userMessage.id));
      
      let errorMessage = 'An unexpected error occurred during processing';
      
      if (error instanceof Error) {
        errorMessage = error.message;
      }
      
      setProcessingState({
        isRecording: false,
        isProcessing: false,
        currentStep: 'idle',
        error: errorMessage
      });
    }
  }, []);

  const clearMessages = useCallback(() => {
    setMessages([]);
  }, []);

  const clearError = useCallback(() => {
    setProcessingState(prev => ({ ...prev, error: null }));
  }, []);

  return {
    messages,
    processingState,
    processAudio,
    clearMessages,
    clearError
  };
};

