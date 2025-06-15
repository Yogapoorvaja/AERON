import React, { useState, useRef, useEffect } from 'react';
import { Play, Pause, Volume2 } from 'lucide-react';

interface AudioPlayerProps {
  audioBase64: string;
  className?: string;
}

export const AudioPlayer: React.FC<AudioPlayerProps> = ({ audioBase64, className = '' }) => {
  const [isPlaying, setIsPlaying] = useState(false);
  const [currentTime, setCurrentTime] = useState(0);
  const [duration, setDuration] = useState(0);
  const audioRef = useRef<HTMLAudioElement | null>(null);

  useEffect(() => {
    if (audioBase64) {
      const audioUrl = `data:audio/wav;base64,${audioBase64}`;
      audioRef.current = new Audio(audioUrl);
      
      const audio = audioRef.current;
      
      const handleLoadedMetadata = () => setDuration(audio.duration);
      const handleTimeUpdate = () => setCurrentTime(audio.currentTime);
      const handleEnded = () => setIsPlaying(false);
      
      audio.addEventListener('loadedmetadata', handleLoadedMetadata);
      audio.addEventListener('timeupdate', handleTimeUpdate);
      audio.addEventListener('ended', handleEnded);
      
      return () => {
        audio.removeEventListener('loadedmetadata', handleLoadedMetadata);
        audio.removeEventListener('timeupdate', handleTimeUpdate);
        audio.removeEventListener('ended', handleEnded);
        audio.pause();
      };
    }
  }, [audioBase64]);

  const togglePlayPause = () => {
    if (!audioRef.current) return;
    
    if (isPlaying) {
      audioRef.current.pause();
    } else {
      audioRef.current.play();
    }
    setIsPlaying(!isPlaying);
  };

  const formatTime = (time: number) => {
    const minutes = Math.floor(time / 60);
    const seconds = Math.floor(time % 60);
    return `${minutes}:${seconds.toString().padStart(2, '0')}`;
  };

  const progressPercentage = duration > 0 ? (currentTime / duration) * 100 : 0;

  return (
    <div className={`flex items-center space-x-3 bg-white/10 backdrop-blur-sm rounded-lg p-3 ${className}`}>
      <button
        onClick={togglePlayPause}
        className="flex items-center justify-center w-8 h-8 bg-blue-500 hover:bg-blue-600 rounded-full transition-colors duration-200"
      >
        {isPlaying ? (
          <Pause className="w-4 h-4 text-white" />
        ) : (
          <Play className="w-4 h-4 text-white ml-0.5" />
        )}
      </button>
      
      <div className="flex-1 flex items-center space-x-2">
        <Volume2 className="w-4 h-4 text-gray-400" />
        <div className="flex-1 bg-gray-200 rounded-full h-2 relative overflow-hidden">
          <div 
            className="absolute top-0 left-0 h-full bg-blue-500 transition-all duration-100 ease-linear"
            style={{ width: `${progressPercentage}%` }}
          />
        </div>
        <span className="text-xs text-gray-500 min-w-[40px]">
          {formatTime(currentTime)} / {formatTime(duration)}
        </span>
      </div>
    </div>
  );
};
