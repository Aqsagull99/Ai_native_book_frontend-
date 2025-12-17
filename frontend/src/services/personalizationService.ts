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

class PersonalizationService {
  private baseUrl: string;

  constructor() {
    // Get API URL from environment or fallback to default
    // In Docusaurus, environment variables need to be properly configured in the build
    let apiUrl = 'http://localhost:8000';

    // Check for environment variable in Node.js context
    if (typeof process !== 'undefined' && process && process.env) {
      apiUrl = process.env.REACT_APP_API_URL || apiUrl;
    }

    // If in browser and window object exists, we can also check for a global config
    if (typeof window !== 'undefined' && (window as any).REACT_APP_API_URL) {
      apiUrl = (window as any).REACT_APP_API_URL;
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
}

// Create and export a singleton instance
export const personalizationService = new PersonalizationService();

// Also export the class for potential direct instantiation
export default PersonalizationService;