import React from 'react';
import SignupForm from '../components/AuthForm/SignupForm';
import Layout from '@theme/Layout';

const SignupPage: React.FC = () => {
  return (
    <Layout title="Sign Up" description="Create your account">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <div className="glassmorphism-card">
              <SignupForm />
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default SignupPage;