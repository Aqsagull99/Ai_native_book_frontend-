import React, { useState } from 'react';
import { useHistory } from '@docusaurus/router';
import { useAuth } from '../../contexts/AuthContext';
import AuthForm from './AuthForm';

const SignupForm: React.FC = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [softwareExperience, setSoftwareExperience] = useState('beginner');
  const [hardwareExperience, setHardwareExperience] = useState('none');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const { signup } = useAuth();
  const history = useHistory();

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      await signup(email, password, softwareExperience, hardwareExperience);
      history.push('/profile'); // Redirect to profile after successful signup
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred during signup');
    } finally {
      setLoading(false);
    }
  };

  return (
    <AuthForm
      title="Create Account"
      onSubmit={handleSubmit}
      submitButtonText="Sign Up"
      footerText="Already have an account?"
      footerLinkText="Sign In"
      footerLinkPath="/signin"
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
        <label htmlFor="password" className="auth-form-label">
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

      <div className="auth-form-group">
        <label htmlFor="softwareExperience" className="auth-form-label">
          Software Experience
        </label>
        <select
          id="softwareExperience"
          className="auth-form-input"
          value={softwareExperience}
          onChange={(e) => setSoftwareExperience(e.target.value)}
        >
          <option value="beginner">Beginner</option>
          <option value="intermediate">Intermediate</option>
          <option value="advanced">Advanced</option>
        </select>
      </div>

      <div className="auth-form-group">
        <label htmlFor="hardwareExperience" className="auth-form-label">
          Hardware Experience
        </label>
        <select
          id="hardwareExperience"
          className="auth-form-input"
          value={hardwareExperience}
          onChange={(e) => setHardwareExperience(e.target.value)}
        >
          <option value="none">None</option>
          <option value="basic">Basic</option>
          <option value="advanced">Advanced</option>
        </select>
      </div>
    </AuthForm>
  );
};

export default SignupForm;