import React, { type ReactNode } from 'react';
import VisualBreathingSpace from './components/VisualBreathingSpace';

interface GlobalWrapperProps {
  children: ReactNode;
}

const GlobalWrapper = ({ children }: GlobalWrapperProps) => {
  return (
    <div style={{ position: 'relative', zIndex: 1 }}>
      {children}
      <VisualBreathingSpace />
    </div>
  );
};

export default GlobalWrapper;