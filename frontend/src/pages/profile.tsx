import React, { useState, useEffect } from 'react';
import { useAuth } from '../contexts/AuthContext';
import Layout from '@theme/Layout';

const ProfilePage: React.FC = () => {
  const { user, loading, updateProfile } = useAuth();
  const [softwareExperience, setSoftwareExperience] = useState(user?.software_experience || '');
  const [hardwareExperience, setHardwareExperience] = useState(user?.hardware_experience || '');
  const [isEditing, setIsEditing] = useState(false);
  const [message, setMessage] = useState('');

  useEffect(() => {
    if (user) {
      setSoftwareExperience(user.software_experience);
      setHardwareExperience(user.hardware_experience);
    }
  }, [user]);

  const handleSave = async () => {
    try {
      await updateProfile(softwareExperience, hardwareExperience);
      setIsEditing(false);
      setMessage('Profile updated successfully!');
      setTimeout(() => setMessage(''), 3000); // Clear message after 3 seconds
    } catch (error) {
      setMessage('Failed to update profile. Please try again.');
      setTimeout(() => setMessage(''), 3000); // Clear message after 3 seconds
    }
  };

  if (loading) {
    return (
      <Layout title="Profile" description="Loading profile...">
        <div className="container margin-vert--lg">
          <div className="row">
            <div className="col col--6 col--offset-3">
              <p>Loading...</p>
            </div>
          </div>
        </div>
      </Layout>
    );
  }

  if (!user) {
    return (
      <Layout title="Profile" description="Please sign in to view your profile">
        <div className="container margin-vert--lg">
          <div className="row">
            <div className="col col--6 col--offset-3">
              <p>Please sign in to view your profile.</p>
            </div>
          </div>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="User Profile" description="Manage your profile information">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <h1>User Profile</h1>

            {message && (
              <div className={`alert ${message.includes('successfully') ? 'alert--success' : 'alert--error'}`}>
                {message}
              </div>
            )}

            <div className="margin-vert--md">
              <p><strong>Email:</strong> {user.email}</p>

              <div className="margin-vert--md">
                <label htmlFor="softwareExperience" className="form-label">
                  <strong>Software Experience:</strong>
                </label>
                {isEditing ? (
                  <select
                    id="softwareExperience"
                    className="form-control"
                    value={softwareExperience}
                    onChange={(e) => setSoftwareExperience(e.target.value)}
                  >
                    <option value="beginner">Beginner</option>
                    <option value="intermediate">Intermediate</option>
                    <option value="advanced">Advanced</option>
                  </select>
                ) : (
                  <p>{softwareExperience}</p>
                )}
              </div>

              <div className="margin-vert--md">
                <label htmlFor="hardwareExperience" className="form-label">
                  <strong>Hardware Experience:</strong>
                </label>
                {isEditing ? (
                  <select
                    id="hardwareExperience"
                    className="form-control"
                    value={hardwareExperience}
                    onChange={(e) => setHardwareExperience(e.target.value)}
                  >
                    <option value="none">None</option>
                    <option value="basic">Basic</option>
                    <option value="advanced">Advanced</option>
                  </select>
                ) : (
                  <p>{hardwareExperience}</p>
                )}
              </div>
            </div>

            <div className="button-group margin-vert--md">
              {!isEditing ? (
                <button
                  className="button button--primary"
                  onClick={() => setIsEditing(true)}
                >
                  Edit Profile
                </button>
              ) : (
                <>
                  <button
                    className="button button--primary margin-right--sm"
                    onClick={handleSave}
                  >
                    Save Changes
                  </button>
                  <button
                    className="button button--secondary"
                    onClick={() => {
                      setIsEditing(false);
                      // Reset to original values
                      setSoftwareExperience(user.software_experience);
                      setHardwareExperience(user.hardware_experience);
                    }}
                  >
                    Cancel
                  </button>
                </>
              )}
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default ProfilePage;