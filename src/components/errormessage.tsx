import React from 'react';
import { AlertCircle, X } from 'lucide-react';

interface ErrorMessageProps {
  message: string;
  onDismiss: () => void;
}

export const ErrorMessage: React.FC<ErrorMessageProps> = ({ message, onDismiss }) => {
  return (
    <div className="flex items-center justify-between p-4 bg-red-50 backdrop-blur-sm border border-red-200 rounded-lg">
      <div className="flex items-center space-x-3">
        <AlertCircle className="w-5 h-5 text-red-500 flex-shrink-0" />
        <p className="text-sm text-red-700">{message}</p>
      </div>
      <button
        onClick={onDismiss}
        className="text-red-400 hover:text-red-600 transition-colors duration-200"
      >
        <X className="w-4 h-4" />
      </button>
    </div>
  );
};
