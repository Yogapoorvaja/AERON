import React, { useEffect, useRef } from 'react';
import { Trash2, MessageCircle } from 'lucide-react';
import { useAudioRecorder } from './hooks/useAudioRecorder';
import { useVoiceAssistant } from './hooks/useVoiceAssistant';
import { RecordingButton } from './components/RecordingButton';
import { MessageBubble } from './components/MessageBubble';
import { ProcessingSteps } from './components/ProcessingSteps';
import { ErrorMessage } from './components/ErrorMessage';

function App() {
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const { isRecording, audioBlob, startRecording, stopRecording, resetRecording } = useAudioRecorder();
  const { messages, processingState, processAudio, clearMessages, clearError } = useVoiceAssistant();

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Process audio when recording stops
  useEffect(() => {
    if (audioBlob && !isRecording) {
      processAudio(audioBlob);
      resetRecording();
    }
  }, [audioBlob, isRecording, processAudio, resetRecording]);

  const handleStartRecording = async () => {
    try {
      await startRecording();
    } catch (error) {
      console.error('Recording failed:', error);
    }
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-blue-50 via-white to-purple-50">
      {/* Header */}
      <header className="sticky top-0 z-10 bg-white/80 backdrop-blur-md border-b border-gray-200">
        <div className="max-w-4xl mx-auto px-4 py-4 flex items-center justify-between">
          <div className="flex items-center space-x-3">
            <div className="w-8 h-8 bg-gradient-to-r from-blue-500 to-purple-600 rounded-lg flex items-center justify-center">
              <MessageCircle className="w-5 h-5 text-white" />
            </div>
            <div>
              <h1 className="text-xl font-semibold text-gray-900">Voice Assistant</h1>
              <p className="text-sm text-gray-500">AI-powered conversations</p>
            </div>
          </div>
          
          {messages.length > 0 && (
            <button
              onClick={clearMessages}
              className="flex items-center space-x-2 px-3 py-2 text-gray-600 hover:text-red-600 hover:bg-red-50 rounded-lg transition-colors duration-200"
            >
              <Trash2 className="w-4 h-4" />
              <span className="text-sm font-medium">Clear</span>
            </button>
          )}
        </div>
      </header>

      {/* Main Content */}
      <main className="max-w-4xl mx-auto px-4 py-6 flex flex-col h-[calc(100vh-88px)]">
        {/* Error Message */}
        {processingState.error && (
          <div className="mb-4">
            <ErrorMessage message={processingState.error} onDismiss={clearError} />
          </div>
        )}

        {/* Messages Container */}
        <div className="flex-1 overflow-y-auto mb-6 space-y-4">
          {messages.length === 0 && !processingState.isProcessing && (
            <div className="flex flex-col items-center justify-center h-full text-center">
              <div className="w-16 h-16 bg-gradient-to-r from-blue-500 to-purple-600 rounded-full flex items-center justify-center mb-4">
                <MessageCircle className="w-8 h-8 text-white" />
              </div>
              <h2 className="text-2xl font-semibold text-gray-900 mb-2">Ready to Chat</h2>
              <p className="text-gray-600 max-w-md">
                Press and hold the microphone button to start a conversation with your AI assistant.
              </p>
            </div>
          )}

          {messages.map((message) => (
            <MessageBubble key={message.id} message={message} />
          ))}
          
          <div ref={messagesEndRef} />
        </div>

        {/* Processing Steps */}
        {processingState.isProcessing && (
          <div className="mb-6">
            <ProcessingSteps processingState={processingState} />
          </div>
        )}

        {/* Recording Controls */}
        <div className="flex flex-col items-center space-y-4">
          <RecordingButton
            isRecording={isRecording}
            isProcessing={processingState.isProcessing}
            onStartRecording={handleStartRecording}
            onStopRecording={stopRecording}
          />
          
          <div className="text-center">
            <p className="text-sm text-gray-600">
              {isRecording 
                ? 'Release to send your message' 
                : processingState.isProcessing 
                ? 'Processing your request...' 
                : 'Tap to start recording'
              }
            </p>
          </div>
        </div>
      </main>
    </div>
  );
}

export default App;
