import React from 'react';
import './AuthForm.css';

interface AuthFormProps {
  title: string;
  children: React.ReactNode;
  onSubmit: (e: React.FormEvent) => void;
  submitButtonText: string;
  footerText: string;
  footerLinkText: string;
  footerLinkPath: string;
  error?: string;
  loading?: boolean;
}

const AuthForm: React.FC<AuthFormProps> = ({
  title,
  children,
  onSubmit,
  submitButtonText,
  footerText,
  footerLinkText,
  footerLinkPath,
  error,
  loading = false
}) => {
  return (
    <div className="auth-form-container">
      <form className="auth-form" onSubmit={onSubmit}>
        <h2 className="auth-form-title">{title}</h2>

        {error && <div className="auth-form-error">{error}</div>}

        {children}

        <button
          type="submit"
          className="auth-form-button"
          disabled={loading}
        >
          {loading ? 'Loading...' : submitButtonText}
        </button>
      </form>

      <div className="auth-form-footer">
        <p>
          {footerText}{' '}
          <a href={footerLinkPath} className="auth-form-link">
            {footerLinkText}
          </a>
        </p>
      </div>
    </div>
  );
};

export default AuthForm;