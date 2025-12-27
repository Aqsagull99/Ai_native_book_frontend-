import React, { createContext, useContext, useReducer, ReactNode } from 'react';

// Define TypeScript interfaces
interface TranslationState {
  isTranslated: {
    [chapterId: string]: {
      translatedContent: string;
      originalContent: string | ReactNode;
    }
  };
  loading: boolean;
  error: string | null;
}

interface TranslateToUrduAction {
  type: 'TRANSLATE_TO_URDU_START';
  chapterId: string;
}

interface TranslateToUrduSuccessAction {
  type: 'TRANSLATE_TO_URDU_SUCCESS';
  chapterId: string;
  translatedContent: string;
  originalContent: string | ReactNode;
}

interface TranslateToUrduFailureAction {
  type: 'TRANSLATE_TO_URDU_FAILURE';
  error: string;
}

interface RevertToOriginalAction {
  type: 'REVERT_TO_ORIGINAL';
  chapterId: string;
}

interface ClearErrorAction {
  type: 'CLEAR_ERROR';
}

type TranslationAction =
  | TranslateToUrduAction
  | TranslateToUrduSuccessAction
  | TranslateToUrduFailureAction
  | RevertToOriginalAction
  | ClearErrorAction;

// Initial state
const initialState: TranslationState = {
  isTranslated: {},
  loading: false,
  error: null,
};

// Reducer function
const translationReducer = (state: TranslationState, action: TranslationAction): TranslationState => {
  switch (action.type) {
    case 'TRANSLATE_TO_URDU_START':
      return {
        ...state,
        loading: true,
        error: null,
      };

    case 'TRANSLATE_TO_URDU_SUCCESS':
      return {
        ...state,
        loading: false,
        isTranslated: {
          ...state.isTranslated,
          [action.chapterId]: {
            translatedContent: action.translatedContent,
            originalContent: action.originalContent,
          },
        },
      };

    case 'TRANSLATE_TO_URDU_FAILURE':
      return {
        ...state,
        loading: false,
        error: action.error,
      };

    case 'REVERT_TO_ORIGINAL':
      const newState = { ...state };
      delete newState.isTranslated[action.chapterId];
      return {
        ...newState,
        error: null,
      };

    case 'CLEAR_ERROR':
      return {
        ...state,
        error: null,
      };

    default:
      return state;
  }
};

// Context type
interface TranslationContextType {
  isTranslated: {
    [chapterId: string]: {
      translatedContent: string;
      originalContent: string | ReactNode;
    }
  };
  loading: boolean;
  error: string | null;
  translateToUrdu: (chapterId: string, content: string) => Promise<void>;
  revertToOriginal: (chapterId: string) => void;
  clearError: () => void;
}

// Create context
const TranslationContext = createContext<TranslationContextType | undefined>(undefined);

// Provider component
interface TranslationProviderProps {
  children: ReactNode;
}

export const TranslationProvider: React.FC<TranslationProviderProps> = ({ children }) => {
  const [state, dispatch] = useReducer(translationReducer, initialState);

  const translateToUrdu = async (chapterId: string, content: string) => {
    dispatch({ type: 'TRANSLATE_TO_URDU_START', chapterId });

    try {
      // Import the translation service function
      const { translateContent } = await import('../services/translationService');

      const response = await translateContent(content, chapterId);

      dispatch({
        type: 'TRANSLATE_TO_URDU_SUCCESS',
        chapterId,
        translatedContent: response.translated_content,
        originalContent: content,
      });
    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'An unknown error occurred';
      dispatch({ type: 'TRANSLATE_TO_URDU_FAILURE', error: errorMessage });
    }
  };

  const revertToOriginal = (chapterId: string) => {
    dispatch({ type: 'REVERT_TO_ORIGINAL', chapterId });
  };

  const clearError = () => {
    dispatch({ type: 'CLEAR_ERROR' });
  };

  const value = {
    isTranslated: state.isTranslated,
    loading: state.loading,
    error: state.error,
    translateToUrdu,
    revertToOriginal,
    clearError,
  };

  return (
    <TranslationContext.Provider value={value}>
      {children}
    </TranslationContext.Provider>
  );
};

// Custom hook to use the translation context
export const useTranslation = (): TranslationContextType => {
  const context = useContext(TranslationContext);
  if (!context) {
    throw new Error('useTranslation must be used within a TranslationProvider');
  }
  return context;
};