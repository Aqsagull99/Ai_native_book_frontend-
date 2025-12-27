import React, { createContext, useContext, useState, ReactNode, useEffect } from 'react';
import { personalizationService } from '../services/personalizationService';

type ExperienceLevel = 'beginner' | 'intermediate' | 'advanced';
type ReadingLevel = 'beginner' | 'intermediate' | 'advanced';
type ExampleDensity = 'minimal' | 'normal' | 'detailed';

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

// NEW: AI Personalization Types
interface AIPreferences {
  readingLevel: ReadingLevel;
  technicalExplanations: boolean;
  exampleDensity: ExampleDensity;
}

interface PersonalizedContentEntry {
  content: string;           // AI-personalized HTML
  originalContent: string;   // Original HTML for revert
  isPersonalized: boolean;
  personalizedAt: string;    // ISO timestamp
  preferencesUsed: AIPreferences;
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

  // NEW: AI Personalization State & Methods
  aiPreferences: AIPreferences;
  personalizedContent: Record<string, PersonalizedContentEntry>;
  isPersonalizing: boolean;
  personalizationError: string | null;
  isPanelOpen: boolean;
  activeChapterId: string | null;

  // NEW: AI Personalization Methods
  updateAIPreferences: (prefs: Partial<AIPreferences>) => void;
  personalizeChapter: (chapterId: string, content: string) => Promise<boolean>;
  revertToOriginal: (chapterId: string) => void;
  openPanel: (chapterId: string) => void;
  closePanel: () => void;
  getContentForChapter: (chapterId: string) => PersonalizedContentEntry | null;
  isChapterPersonalized: (chapterId: string) => boolean;
}

const PersonalizationContext = createContext<PersonalizationContextType | undefined>(undefined);

interface PersonalizationProviderProps {
  children: ReactNode;
}

// Helper function to map user profile to default AI preferences
const mapProfileToDefaults = (softwareExperience: string): AIPreferences => ({
  readingLevel: (softwareExperience as ReadingLevel) || 'intermediate',
  technicalExplanations: softwareExperience === 'beginner',
  exampleDensity: 'normal'
});

export const PersonalizationProvider: React.FC<PersonalizationProviderProps> = ({ children }) => {
  const [experienceLevel, setExperienceLevel] = useState<ExperienceLevel>('beginner');
  const [personalizationStatus, setPersonalizationStatus] = useState<Record<string, PersonalizationStatus>>({});
  const [bonusPoints, setBonusPoints] = useState<UserBonusPoints | null>(null);

  // NEW: AI Personalization State
  const [aiPreferences, setAIPreferences] = useState<AIPreferences>({
    readingLevel: 'intermediate',
    technicalExplanations: false,
    exampleDensity: 'normal'
  });
  const [personalizedContent, setPersonalizedContent] = useState<Record<string, PersonalizedContentEntry>>({});
  const [isPersonalizing, setIsPersonalizing] = useState<boolean>(false);
  const [personalizationError, setPersonalizationError] = useState<string | null>(null);
  const [isPanelOpen, setIsPanelOpen] = useState<boolean>(false);
  const [activeChapterId, setActiveChapterId] = useState<string | null>(null);

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

    // NEW: Also update AI preferences defaults from profile
    setAIPreferences(mapProfileToDefaults(softwareExperience));
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

  // ============================================
  // NEW: AI Personalization Methods
  // ============================================

  const updateAIPreferences = (prefs: Partial<AIPreferences>) => {
    setAIPreferences(prev => ({ ...prev, ...prefs }));
  };

  const personalizeChapter = async (chapterId: string, content: string): Promise<boolean> => {
    console.log('personalizeChapter called - chapterId:', chapterId, 'content length:', content.length);
    console.log('Current preferences:', aiPreferences);

    setIsPersonalizing(true);
    setPersonalizationError(null);

    try {
      console.log('Calling personalizationService.aiPersonalize...');
      const result = await personalizationService.aiPersonalize(
        chapterId,
        content,
        {
          reading_level: aiPreferences.readingLevel,
          technical_explanations: aiPreferences.technicalExplanations,
          example_density: aiPreferences.exampleDensity
        }
      );

      console.log('API result:', result);

      if (result.success && result.personalized_content) {
        console.log('Personalization successful, updating state...');
        setPersonalizedContent(prev => ({
          ...prev,
          [chapterId]: {
            content: result.personalized_content,
            originalContent: content,
            isPersonalized: true,
            personalizedAt: new Date().toISOString(),
            preferencesUsed: { ...aiPreferences }
          }
        }));
        setIsPanelOpen(false);
        return true;
      } else {
        console.error('Personalization failed:', result.error);
        setPersonalizationError(result.error || 'Failed to personalize content');
        return false;
      }
    } catch (error: any) {
      console.error('Error personalizing chapter:', error);
      setPersonalizationError(error.message || 'An error occurred during personalization');
      return false;
    } finally {
      setIsPersonalizing(false);
    }
  };

  const revertToOriginal = (chapterId: string): void => {
    setPersonalizedContent(prev => {
      const updated = { ...prev };
      delete updated[chapterId];
      return updated;
    });
    setPersonalizationError(null);
  };

  const openPanel = (chapterId: string): void => {
    console.log('openPanel called with chapterId:', chapterId);
    setActiveChapterId(chapterId);
    setIsPanelOpen(true);
    setPersonalizationError(null);
    console.log('Panel state set - isPanelOpen: true, activeChapterId:', chapterId);
  };

  const closePanel = (): void => {
    setIsPanelOpen(false);
    setActiveChapterId(null);
    setPersonalizationError(null);
  };

  const getContentForChapter = (chapterId: string): PersonalizedContentEntry | null => {
    return personalizedContent[chapterId] || null;
  };

  const isChapterPersonalized = (chapterId: string): boolean => {
    return !!personalizedContent[chapterId]?.isPersonalized;
  };

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
    refreshPersonalizationData,

    // NEW: AI Personalization State & Methods
    aiPreferences,
    personalizedContent,
    isPersonalizing,
    personalizationError,
    isPanelOpen,
    activeChapterId,
    updateAIPreferences,
    personalizeChapter,
    revertToOriginal,
    openPanel,
    closePanel,
    getContentForChapter,
    isChapterPersonalized
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