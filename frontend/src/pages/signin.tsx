import React from 'react';
import SigninForm from '../components/AuthForm/SigninForm';
import Layout from '@theme/Layout';

const SigninPage: React.FC = () => {
  return (
    <Layout title="Sign In" description="Sign in to your account">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <div className="glassmorphism-card">
              <SigninForm />
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default SigninPage;