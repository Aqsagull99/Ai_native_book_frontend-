import React, { createContext, useContext, useState, ReactNode, useEffect } from 'react';
import { personalizationService } from '../services/personalizationService';

type ExperienceLevel = 'beginner' | 'intermediate' | 'advanced';

interface PersonalizationStatus {
  isPersonalized: boolean;
  preferences: Record<string, any> | null;
  pointsEarned: number;
}

interface UserBonusPoints {
  totalPoints: number;
  pointsBreakdown: Array<{
    chapterId: string;
    points: number;
    earnedAt: string;
  }>;
}

interface PersonalizationContextType {
  experienceLevel: ExperienceLevel;
  setExperienceLevel: (level: ExperienceLevel) => void;
  getPersonalizedContent: (content: { level: ExperienceLevel; content: string }[]) => string;
  syncWithUserProfile: (softwareExperience: string, hardwareExperience: string) => void;
  personalizationStatus: Record<string, PersonalizationStatus>;
  bonusPoints: UserBonusPoints | null;
  activatePersonalization: (chapterId: string, preferences?: Record<string, any>) => Promise<boolean>;
  getPersonalizationStatus: (chapterId: string) => Promise<PersonalizationStatus>;
  getUserBonusPoints: () => Promise<UserBonusPoints>;
  refreshPersonalizationData: () => Promise<void>;
}

const PersonalizationContext = createContext<PersonalizationContextType | undefined>(undefined);

interface PersonalizationProviderProps {
  children: ReactNode;
}

export const PersonalizationProvider: React.FC<PersonalizationProviderProps> = ({ children }) => {
  const [experienceLevel, setExperienceLevel] = useState<ExperienceLevel>('beginner');
  const [personalizationStatus, setPersonalizationStatus] = useState<Record<string, PersonalizationStatus>>({});
  const [bonusPoints, setBonusPoints] = useState<UserBonusPoints | null>(null);

  const getPersonalizedContent = (contentOptions: { level: ExperienceLevel; content: string }[]) => {
    // First, try to get content for the exact experience level
    const exactMatch = contentOptions.find(option => option.level === experienceLevel);
    if (exactMatch) {
      return exactMatch.content;
    }

    // If no exact match, find the closest appropriate level
    // For example, if we don't have advanced content, fall back to intermediate
    // If we don't have intermediate, fall back to beginner
    if (experienceLevel === 'advanced') {
      const intermediateMatch = contentOptions.find(option => option.level === 'intermediate');
      if (intermediateMatch) return intermediateMatch.content;
    }

    if (experienceLevel === 'advanced' || experienceLevel === 'intermediate') {
      const beginnerMatch = contentOptions.find(option => option.level === 'beginner');
      if (beginnerMatch) return beginnerMatch.content;
    }

    // If nothing matches, return the first available content
    return contentOptions[0]?.content || '';
  };

  const syncWithUserProfile = (softwareExperience: string, hardwareExperience: string) => {
    // Determine the experience level based on user profile
    // For simplicity, we'll use the higher of the two experience levels
    if (softwareExperience === 'advanced' || hardwareExperience === 'advanced') {
      setExperienceLevel('advanced');
    } else if (softwareExperience === 'intermediate' || hardwareExperience === 'basic') {
      setExperienceLevel('intermediate');
    } else {
      setExperienceLevel('beginner');
    }
  };

  const activatePersonalization = async (chapterId: string, preferences: Record<string, any> = {}): Promise<boolean> => {
    try {
      const result = await personalizationService.activatePersonalization(chapterId, preferences);

      if (result.success) {
        // Update local state
        setPersonalizationStatus(prev => ({
          ...prev,
          [chapterId]: {
            isPersonalized: true,
            preferences: result.personalizedContent?.preferences_applied || preferences,
            pointsEarned: result.pointsEarned || 0
          }
        }));

        // Update bonus points
        if (result.pointsEarned) {
          setBonusPoints(prev => {
            if (!prev) {
              const newBonusPoints: UserBonusPoints = {
                totalPoints: result.pointsEarned,
                pointsBreakdown: [{
                  chapterId,
                  points: result.pointsEarned,
                  earnedAt: new Date().toISOString()
                }]
              };
              return newBonusPoints;
            }

            const updatedBonusPoints: UserBonusPoints = {
              ...prev,
              totalPoints: prev.totalPoints + result.pointsEarned,
              pointsBreakdown: [
                ...prev.pointsBreakdown,
                {
                  chapterId,
                  points: result.pointsEarned,
                  earnedAt: new Date().toISOString()
                }
              ]
            };
            return updatedBonusPoints;
          });
        }

        return true;
      }
      return false;
    } catch (error) {
      console.error('Error activating personalization:', error);
      return false;
    }
  };

  const getPersonalizationStatus = async (chapterId: string): Promise<PersonalizationStatus> => {
    try {
      const status = await personalizationService.getPersonalizationStatus(chapterId);
      // Map the backend response to the frontend interface
      const frontendStatus: PersonalizationStatus = {
        isPersonalized: status.isPersonalized,
        preferences: status.preferences,
        pointsEarned: status.points_earned
      };
      setPersonalizationStatus(prev => ({
        ...prev,
        [chapterId]: frontendStatus
      }));
      return frontendStatus;
    } catch (error) {
      console.error('Error getting personalization status:', error);
      const defaultStatus: PersonalizationStatus = { isPersonalized: false, preferences: null, pointsEarned: 0 };
      setPersonalizationStatus(prev => ({
        ...prev,
        [chapterId]: defaultStatus
      }));
      return defaultStatus;
    }
  };

  const getUserBonusPoints = async (): Promise<UserBonusPoints> => {
    try {
      const points = await personalizationService.getUserBonusPoints();
      // Map the backend response to the frontend interface
      const frontendPoints: UserBonusPoints = {
        totalPoints: points.total_points,
        pointsBreakdown: points.points_breakdown.map(bp => ({
          chapterId: bp.chapter_id,
          points: bp.points,
          earnedAt: bp.earned_at
        }))
      };
      setBonusPoints(frontendPoints);
      return frontendPoints;
    } catch (error) {
      console.error('Error getting user bonus points:', error);
      const defaultPoints: UserBonusPoints = { totalPoints: 0, pointsBreakdown: [] };
      setBonusPoints(defaultPoints);
      return defaultPoints;
    }
  };

  const refreshPersonalizationData = async (): Promise<void> => {
    try {
      // Fetch user's bonus points
      const points = await personalizationService.getUserBonusPoints();
      // Map the backend response to the frontend interface
      const frontendPoints: UserBonusPoints = {
        totalPoints: points.total_points,
        pointsBreakdown: points.points_breakdown.map(bp => ({
          chapterId: bp.chapter_id,
          points: bp.points,
          earnedAt: bp.earned_at
        }))
      };
      setBonusPoints(frontendPoints);
    } catch (error) {
      console.error('Error refreshing personalization data:', error);
    }
  };

  // Load user's bonus points on initial load
  useEffect(() => {
    getUserBonusPoints();
  }, []);

  const value = {
    experienceLevel,
    setExperienceLevel,
    getPersonalizedContent,
    syncWithUserProfile,
    personalizationStatus,
    bonusPoints,
    activatePersonalization,
    getPersonalizationStatus,
    getUserBonusPoints,
    refreshPersonalizationData
  };

  return (
    <PersonalizationContext.Provider value={value}>
      {children}
    </PersonalizationContext.Provider>
  );
};

export const usePersonalization = () => {
  const context = useContext(PersonalizationContext);
  if (context === undefined) {
    throw new Error('usePersonalization must be used within a PersonalizationProvider');
  }
  return context;
};