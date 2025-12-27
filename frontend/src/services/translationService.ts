/**
 * Frontend service to communicate with the backend translation API.
 */

// Get API base URL - use environment variable or default to localhost:8000 for development
const getApiBaseUrl = (): string => {
  // In production (Vercel), use relative path as proxy handles routing
  if (typeof window !== 'undefined' && window.location.hostname !== 'localhost') {
    return '';
  }
  // In local development, use backend directly
  return 'http://localhost:8000';
};

const API_BASE_URL = getApiBaseUrl();

// Get auth header from localStorage
const getAuthHeader = (): Record<string, string> => {
  if (typeof window !== 'undefined') {
    const token = localStorage.getItem('token');
    if (token) {
      return { 'Authorization': `Bearer ${token}` };
    }
  }
  return {};
};

interface TranslationRequest {
  content: string;
  chapter_id: string;
  user_id?: string;
  preserve_formatting: boolean;
}

interface TranslationResponse {
  translated_content: string;
  chapter_id: string;
  user_id: string;
  source_language: string;
  target_language: string;
  translation_id: string;
  created_at: string;
}

interface TranslationCacheRequest {
  chapter_id: string;
  user_id: string;
}

interface TranslationCacheResponse {
  is_cached: boolean;
  cached_content?: string;
  chapter_id: string;
  user_id: string;
}

/**
 * Translates content to Urdu using the backend API.
 *
 * @param content The content to translate
 * @param chapterId The ID of the chapter being translated
 * @param userId The ID of the user requesting translation (optional, will be inferred from session)
 * @returns Promise resolving to the translation response
 */
export const translateContent = async (
  content: string,
  chapterId: string,
  userId?: string
): Promise<TranslationResponse> => {
  try {
    const requestBody: TranslationRequest = {
      content,
      chapter_id: chapterId,
      preserve_formatting: true,
    };

    if (userId) {
      requestBody.user_id = userId;
    }

    const response = await fetch(`${API_BASE_URL}/api/translation/translate`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        ...getAuthHeader(),
      },
      body: JSON.stringify(requestBody),
    });

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new Error(errorData.detail || `Translation failed: ${response.status} ${response.statusText}`);
    }

    const data: TranslationResponse = await response.json();
    return data;
  } catch (error) {
    console.error('Translation API error:', error);
    throw error;
  }
};

/**
 * Checks if a translation is already cached for a user and chapter.
 *
 * @param chapterId The ID of the chapter
 * @param userId The ID of the user
 * @returns Promise resolving to the cache check response
 */
export const checkTranslationCache = async (
  chapterId: string,
  userId: string
): Promise<TranslationCacheResponse> => {
  try {
    const requestBody: TranslationCacheRequest = {
      chapter_id: chapterId,
      user_id: userId,
    };

    const response = await fetch(`${API_BASE_URL}/api/translation/cache/check`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        ...getAuthHeader(),
      },
      body: JSON.stringify(requestBody),
    });

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new Error(errorData.detail || `Cache check failed: ${response.status} ${response.statusText}`);
    }

    const data: TranslationCacheResponse = await response.json();
    return data;
  } catch (error) {
    console.error('Translation cache check error:', error);
    throw error;
  }
};

/**
 * Performs async translation (for longer content).
 *
 * @param content The content to translate
 * @param chapterId The ID of the chapter being translated
 * @param userId The ID of the user requesting translation
 * @returns Promise resolving to an object with translation ID and status
 */
export const translateContentAsync = async (
  content: string,
  chapterId: string,
  userId?: string
): Promise<{ translation_id: string; status: string; chapter_id: string; user_id: string }> => {
  try {
    const requestBody: TranslationRequest = {
      content,
      chapter_id: chapterId,
      preserve_formatting: true,
    };

    if (userId) {
      requestBody.user_id = userId;
    }

    const response = await fetch(`${API_BASE_URL}/api/translation/translate-async`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        ...getAuthHeader(),
      },
      body: JSON.stringify(requestBody),
    });

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new Error(errorData.detail || `Async translation failed: ${response.status} ${response.statusText}`);
    }

    const data = await response.json();
    return data;
  } catch (error) {
    console.error('Async translation API error:', error);
    throw error;
  }
};

/**
 * Gets the status of an async translation.
 *
 * @param translationId The ID of the translation job
 * @returns Promise resolving to the translation status response
 */
export const getTranslationStatus = async (
  translationId: string
): Promise<{
  translation_id: string;
  status: string;
  translated_content?: string;
  error_message?: string;
}> => {
  try {
    const response = await fetch(`${API_BASE_URL}/api/translation/status`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        ...getAuthHeader(),
      },
      body: JSON.stringify({ translation_id: translationId }),
    });

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new Error(errorData.detail || `Status check failed: ${response.status} ${response.statusText}`);
    }

    const data = await response.json();
    return data;
  } catch (error) {
    console.error('Translation status check error:', error);
    throw error;
  }
};

/**
 * Health check for the translation service.
 *
 * @returns Promise resolving to the health status
 */
export const translationHealthCheck = async (): Promise<{ status: string; service: string; error?: string }> => {
  try {
    const response = await fetch(`${API_BASE_URL}/api/translation/health`, {
      method: 'GET',
      headers: {
        ...getAuthHeader(),
      },
    });

    if (!response.ok) {
      throw new Error(`Health check failed: ${response.status} ${response.statusText}`);
    }

    const data = await response.json();
    return data;
  } catch (error) {
    console.error('Translation health check error:', error);
    throw error;
  }
};