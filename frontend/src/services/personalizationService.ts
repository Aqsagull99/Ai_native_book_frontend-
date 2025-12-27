/**
 * Personalization Service
 * Frontend service to interact with the backend personalization API
 */

interface PersonalizationStatusResponse {
  is_personalized: boolean;
  preferences: Record<string, any> | null;
  points_earned: number;
}

interface PersonalizationStatus {
  isPersonalized: boolean;
  preferences: Record<string, any> | null;
  pointsEarned: number;
}

interface UserBonusPointsResponse {
  total_points: number;
  points_breakdown: Array<{
    chapter_id: string;
    points: number;
    earned_at: string;
  }>;
}

interface UserBonusPoints {
  totalPoints: number;
  pointsBreakdown: Array<{
    chapterId: string;
    points: number;
    earnedAt: string;
  }>;
}

interface ActivatePersonalizationResponseBackend {
  success: boolean;
  message: string;
  points_earned: number;
  personalized_content?: any;
  is_duplicate?: boolean;
}

interface ActivatePersonalizationResponse {
  success: boolean;
  message: string;
  pointsEarned: number;
  personalizedContent?: any;
  isDuplicate?: boolean;
}

// NEW: AI Personalization Types
interface AIPersonalizationPreferences {
  reading_level: string;
  technical_explanations: boolean;
  example_density: string;
}

interface AIPersonalizationUserProfile {
  software_experience?: string;
  hardware_experience?: string;
}

interface AIPersonalizeResponseBackend {
  success: boolean;
  personalized_content?: string;
  chapter_id: string;
  preferences_applied?: {
    reading_level: string;
    technical_explanations: boolean;
    example_density: string;
  };
  processing_time_ms?: number;
  error?: string;
}

interface AIPersonalizeResponse {
  success: boolean;
  personalized_content?: string;
  chapter_id: string;
  preferences_applied?: {
    reading_level: string;
    technical_explanations: boolean;
    example_density: string;
  };
  processing_time_ms?: number;
  error?: string;
}

class PersonalizationService {
  private baseUrl: string;

  constructor() {
    // Get API URL from environment or fallback to default
    // In Docusaurus, environment variables need to be properly configured in the build
    let apiUrl = 'http://localhost:8000';

    // Check for environment variable in Node.js context
    if (typeof process !== 'undefined' && process && process.env) {
      apiUrl = process.env.REACT_APP_API_BASE_URL || apiUrl;
    }

    // If in browser and window object exists, we can also check for a global config
    if (typeof window !== 'undefined' && (window as any).REACT_APP_API_BASE_URL) {
      apiUrl = (window as any).REACT_APP_API_BASE_URL;
    }

    this.baseUrl = apiUrl;
  }

  /**
   * Activate personalization for a chapter and earn bonus points
   */
  async activatePersonalization(
    chapterId: string,
    preferences: Record<string, any> = {}
  ): Promise<ActivatePersonalizationResponse> {
    try {
      const response = await fetch(`${this.baseUrl}/api/personalization/activate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          // Include authentication token if available
          ...this.getAuthHeader()
        },
        body: JSON.stringify({
          chapter_id: chapterId,
          preferences
        })
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
      }

      const data: ActivatePersonalizationResponseBackend = await response.json();
      return {
        success: data.success,
        message: data.message,
        pointsEarned: data.points_earned,
        personalizedContent: data.personalized_content,
        isDuplicate: data.is_duplicate
      };
    } catch (error) {
      console.error('Error activating personalization:', error);
      throw error;
    }
  }

  /**
   * Get personalization status for a specific chapter
   */
  async getPersonalizationStatus(chapterId: string): Promise<PersonalizationStatus> {
    try {
      const response = await fetch(`${this.baseUrl}/api/personalization/status?chapter_id=${encodeURIComponent(chapterId)}`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
          // Include authentication token if available
          ...this.getAuthHeader()
        }
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
      }

      const data: PersonalizationStatusResponse = await response.json();
      return {
        isPersonalized: data.is_personalized,
        preferences: data.preferences,
        pointsEarned: data.points_earned || 0
      };
    } catch (error) {
      console.error('Error getting personalization status:', error);
      throw error;
    }
  }

  /**
   * Get all personalization preferences for the user
   */
  async getUserPreferences(): Promise<any[]> {
    try {
      const response = await fetch(`${this.baseUrl}/api/personalization/preferences`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
          // Include authentication token if available
          ...this.getAuthHeader()
        }
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data.preferences || [];
    } catch (error) {
      console.error('Error getting user preferences:', error);
      throw error;
    }
  }

  /**
   * Get user's bonus points
   */
  async getUserBonusPoints(): Promise<UserBonusPoints> {
    try {
      const response = await fetch(`${this.baseUrl}/api/user/bonus-points`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
          // Include authentication token if available
          ...this.getAuthHeader()
        }
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
      }

      const data: UserBonusPointsResponse = await response.json();
      return {
        totalPoints: data.total_points || 0,
        pointsBreakdown: (data.points_breakdown || []).map(item => ({
          chapterId: item.chapter_id,
          points: item.points,
          earnedAt: item.earned_at
        }))
      };
    } catch (error) {
      console.error('Error getting user bonus points:', error);
      throw error;
    }
  }

  /**
   * Get authentication header if token is available
   */
  private getAuthHeader(): Record<string, string> {
    if (typeof window !== 'undefined') {
      const token = localStorage.getItem('token');
      if (token) {
        return { 'Authorization': `Bearer ${token}` };
      }
    }
    return {};
  }

  // ============================================
  // NEW: AI Personalization Methods
  // ============================================

  /**
   * AI Personalize chapter content based on user preferences
   * @param chapterId The ID of the chapter to personalize
   * @param content The HTML content to personalize
   * @param preferences The personalization preferences (reading_level, technical_explanations, example_density)
   * @param userProfile Optional user profile with experience levels
   * @returns Promise<AIPersonalizeResponse>
   */
  async aiPersonalize(
    chapterId: string,
    content: string,
    preferences: AIPersonalizationPreferences,
    userProfile?: AIPersonalizationUserProfile
  ): Promise<AIPersonalizeResponse> {
    try {
      const response = await fetch(`${this.baseUrl}/api/personalization/ai-personalize`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          ...this.getAuthHeader()
        },
        body: JSON.stringify({
          chapter_id: chapterId,
          content: content,
          preferences: preferences,
          user_profile: userProfile || null
        })
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        return {
          success: false,
          chapter_id: chapterId,
          error: errorData.detail || `HTTP error! status: ${response.status}`
        };
      }

      const data: AIPersonalizeResponseBackend = await response.json();
      return {
        success: data.success,
        personalized_content: data.personalized_content,
        chapter_id: data.chapter_id,
        preferences_applied: data.preferences_applied,
        processing_time_ms: data.processing_time_ms,
        error: data.error
      };
    } catch (error: any) {
      console.error('Error calling AI personalization:', error);
      return {
        success: false,
        chapter_id: chapterId,
        error: error.message || 'Network error during AI personalization'
      };
    }
  }

  /**
   * Check the health of the personalization service
   * @returns Promise with service health status
   */
  async checkHealth(): Promise<{
    status: string;
    service: string;
    ai_available: boolean;
    openrouter_configured: boolean;
    error?: string;
  }> {
    try {
      const response = await fetch(`${this.baseUrl}/api/personalization/health`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json'
        }
      });

      if (!response.ok) {
        return {
          status: 'unhealthy',
          service: 'personalization',
          ai_available: false,
          openrouter_configured: false,
          error: `HTTP error! status: ${response.status}`
        };
      }

      return await response.json();
    } catch (error: any) {
      console.error('Error checking personalization health:', error);
      return {
        status: 'unhealthy',
        service: 'personalization',
        ai_available: false,
        openrouter_configured: false,
        error: error.message || 'Network error'
      };
    }
  }
}

// Create and export a singleton instance
export const personalizationService = new PersonalizationService();

// Also export the class for potential direct instantiation
export default PersonalizationService;