import React from 'react';
import { Mic, MicOff, Loader2 } from 'lucide-react';

interface RecordingButtonProps {
  isRecording: boolean;
  isProcessing: boolean;
  onStartRecording: () => void;
  onStopRecording: () => void;
  disabled?: boolean;
}

export const RecordingButton: React.FC<RecordingButtonProps> = ({
  isRecording,
  isProcessing,
  onStartRecording,
  onStopRecording,
  disabled = false
}) => {
  const handleClick = () => {
    if (isRecording) {
      onStopRecording();
    } else {
      onStartRecording();
    }
  };

  return (
    <div className="relative flex items-center justify-center">
      {/* Recording pulse animation */}
      {isRecording && (
        <div className="absolute inset-0 rounded-full bg-red-400 animate-ping opacity-75" />
      )}
      
      <button
        onClick={handleClick}
        disabled={disabled || isProcessing}
        className={`relative w-16 h-16 rounded-full flex items-center justify-center transition-all duration-300 transform hover:scale-105 active:scale-95 ${
          isRecording 
            ? 'bg-red-500 shadow-lg shadow-red-500/50' 
            : isProcessing
            ? 'bg-gray-400 cursor-not-allowed'
            : 'bg-gradient-to-r from-blue-500 to-purple-600 hover:from-blue-600 hover:to-purple-700 shadow-lg shadow-blue-500/30'
        }`}
      >
        {isProcessing ? (
          <Loader2 className="w-6 h-6 text-white animate-spin" />
        ) : isRecording ? (
          <MicOff className="w-6 h-6 text-white" />
        ) : (
          <Mic className="w-6 h-6 text-white" />
        )}
      </button>
      
      {/* Recording indicator text */}
      {isRecording && (
        <div className="absolute -bottom-8 left-1/2 transform -translate-x-1/2">
          <span className="text-xs text-red-500 font-medium animate-pulse">Recording...</span>
        </div>
      )}
    </div>
  );
};
