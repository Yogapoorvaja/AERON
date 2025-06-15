import React from 'react';
import { Loader2, Mic, MessageSquare, Volume2 } from 'lucide-react';
import { ProcessingState } from '../types';

interface ProcessingStepsProps {
  processingState: ProcessingState;
}

export const ProcessingSteps: React.FC<ProcessingStepsProps> = ({ processingState }) => {
  if (!processingState.isProcessing && processingState.currentStep === 'idle') {
    return null;
  }

  const steps = [
    { id: 'transcribing', label: 'Transcribing Audio', icon: Mic },
    { id: 'generating', label: 'Generating Response', icon: MessageSquare },
    { id: 'synthesizing', label: 'Creating Speech', icon: Volume2 }
  ];

  return (
    <div className="flex flex-col items-center space-y-4 p-6 bg-white/10 backdrop-blur-sm rounded-2xl">
      <div className="flex items-center space-x-2">
        <Loader2 className="w-5 h-5 text-blue-500 animate-spin" />
        <span className="text-sm font-medium text-gray-700">Processing your request...</span>
      </div>
      
      <div className="flex items-center space-x-4">
        {steps.map((step, index) => {
          const Icon = step.icon;
          const isActive = processingState.currentStep === step.id;
          const isCompleted = steps.findIndex(s => s.id === processingState.currentStep) > index;
          
          return (
            <div key={step.id} className="flex flex-col items-center space-y-2">
              <div className={`w-10 h-10 rounded-full flex items-center justify-center transition-all duration-300 ${
                isActive 
                  ? 'bg-blue-500 text-white' 
                  : isCompleted 
                  ? 'bg-green-500 text-white' 
                  : 'bg-gray-200 text-gray-400'
              }`}>
                <Icon className="w-5 h-5" />
              </div>
              <span className={`text-xs font-medium text-center ${
                isActive ? 'text-blue-600' : isCompleted ? 'text-green-600' : 'text-gray-400'
              }`}>
                {step.label}
              </span>
            </div>
          );
        })}
      </div>
    </div>
  );
};

