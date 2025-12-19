import React, { createContext, useContext, ReactNode } from 'react';
import { AuthProvider, useAuth } from './contexts/AuthContext';
import { PersonalizationProvider, usePersonalization } from './contexts/PersonalizationContext';
import FloatingRagChatbot from './components/FloatingRagChatbot';

// Create a context to coordinate between auth and personalization
interface CoordinationContextType {
  syncPersonalizationWithAuth: () => void;
}

const CoordinationContext = createContext<CoordinationContextType | undefined>(undefined);

// Inner component that has access to both contexts
const ProvidersInner: React.FC<{ children: ReactNode }> = ({ children }) => {
  const { user } = useAuth();
  const { syncWithUserProfile } = usePersonalization();

  // Function to sync personalization with auth user profile
  const syncPersonalizationWithAuth = () => {
    if (user) {
      syncWithUserProfile(user.software_experience, user.hardware_experience);
    }
  };

  return (
    <CoordinationContext.Provider value={{ syncPersonalizationWithAuth }}>
      {children}
      <FloatingRagChatbot />
    </CoordinationContext.Provider>
  );
};

interface ProvidersWrapperProps {
  children: React.ReactNode;
}

const ProvidersWrapper: React.FC<ProvidersWrapperProps> = ({ children }) => {
  return (
    <AuthProvider>
      <PersonalizationProvider>
        <ProvidersInner>
          {children}
        </ProvidersInner>
      </PersonalizationProvider>
    </AuthProvider>
  );
};

// Hook to access the coordination context
export const useCoordination = () => {
  const context = useContext(CoordinationContext);
  if (context === undefined) {
    throw new Error('useCoordination must be used within a ProvidersWrapper');
  }
  return context;
};

export default ProvidersWrapper;


// Jab user login hota hai → personalization data automatically set ho jata hai.

// Final Summary:
// Component	Kaam
// AuthProvider	Login / Signup handle karta hai
// PersonalizationProvider	User ke skills / background store karta hai
// ProvidersInner	Dono ko connect karta hai
// useCoordination()	Kisi bhi component se call karke sync kara sakti ho
// no need ui
