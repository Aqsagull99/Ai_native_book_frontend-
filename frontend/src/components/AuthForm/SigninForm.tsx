import React, { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import { useAuth } from '../../contexts/AuthContext';
import AuthForm from './AuthForm';

const SigninForm: React.FC = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const { login } = useAuth();
  const navigate = useNavigate();

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      await login(email, password);
      navigate('/'); // Redirect to home after successful signin
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred during signin');
    } finally {
      setLoading(false);
    }
  };

  return (
    <AuthForm
      title="Sign In"
      onSubmit={handleSubmit}
      submitButtonText="Sign In"
      footerText="Don't have an account?"
      footerLinkText="Sign Up"
      footerLinkPath="/signup"
      error={error}
      loading={loading}
    >
      <div className="auth-form-group">
        <label htmlFor="email" className="auth-form-label">
          Email
        </label>
        <input
          id="email"
          type="email"
          className="auth-form-input"
          value={email}
          onChange={(e) => setEmail(e.target.value)}
          required
        />
      </div>

      <div className="auth-form-group">
        <label htmlFor="password" className="auth-form-input">
          Password
        </label>
        <input
          id="password"
          type="password"
          className="auth-form-input"
          value={password}
          onChange={(e) => setPassword(e.target.value)}
          required
        />
      </div>
    </AuthForm>
  );
};

export default SigninForm;