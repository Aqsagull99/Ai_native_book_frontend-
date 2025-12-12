import React from 'react';
import { AuthProvider } from './contexts/AuthContext';
import { PersonalizationProvider } from './contexts/PersonalizationContext';

interface ProvidersWrapperProps {
  children: React.ReactNode;
}

const ProvidersWrapper: React.FC<ProvidersWrapperProps> = ({ children }) => {
  return (
    <AuthProvider>
      <PersonalizationProvider>
        {children}
      </PersonalizationProvider>
    </AuthProvider>
  );
};

export default ProvidersWrapper;