import React, { useState, useEffect } from 'react';
import { useAuth } from '../contexts/AuthContext';
import { usePersonalization } from '../contexts/PersonalizationContext';
import BonusPointsDisplay from '../components/Personalization/BonusPointsDisplay';
import Layout from '@theme/Layout';
import styles from './profile.module.css';

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
            <div className={styles.profileContainer}>
              <div className={styles.profileHeader}>
                <h1 className={styles.profileHeading}>User Profile</h1>
              </div>

              {message && (
                <div className={`${styles.messageContainer} ${message.includes('successfully') ? styles.success : styles.error}`}>
                  {message}
                </div>
              )}

              <div className={styles.profileField}>
                <label className={styles.profileLabel}>Email</label>
                <div className={styles.profileFieldDisplay}>
                  {user.email}
                </div>
              </div>

              <div className={styles.profileField}>
                <label htmlFor="softwareExperience" className={styles.profileLabel}>
                  Software Experience
                </label>
                {isEditing ? (
                  <select
                    id="softwareExperience"
                    className={styles.profileFieldEdit}
                    value={softwareExperience}
                    onChange={(e) => setSoftwareExperience(e.target.value)}
                  >
                    <option value="beginner">Beginner</option>
                    <option value="intermediate">Intermediate</option>
                    <option value="advanced">Advanced</option>
                  </select>
                ) : (
                  <div className={styles.profileFieldDisplay}>
                    {softwareExperience}
                  </div>
                )}
              </div>

              <div className={styles.profileField}>
                <label htmlFor="hardwareExperience" className={styles.profileLabel}>
                  Hardware Experience
                </label>
                {isEditing ? (
                  <select
                    id="hardwareExperience"
                    className={styles.profileFieldEdit}
                    value={hardwareExperience}
                    onChange={(e) => setHardwareExperience(e.target.value)}
                  >
                    <option value="none">None</option>
                    <option value="basic">Basic</option>
                    <option value="advanced">Advanced</option>
                  </select>
                ) : (
                  <div className={styles.profileFieldDisplay}>
                    {hardwareExperience}
                  </div>
                )}
              </div>

              <div className={styles.bonusPointsSection}>
                <div className={styles.bonusPointsTitle}>Bonus Points</div>
                <div className={styles.bonusPointsValue}>
                  <BonusPointsDisplay showHistory={false} />
                </div>
              </div>

              <div className={styles.buttonGroup}>
                {!isEditing ? (
                  <button
                    className={styles.editButton}
                    onClick={() => setIsEditing(true)}
                  >
                    Edit Profile
                  </button>
                ) : (
                  <>
                    <button
                      className={styles.saveButton}
                      onClick={handleSave}
                    >
                      Save Changes
                    </button>
                    <button
                      className={styles.cancelButton}
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
      </div>
    </Layout>
  );
};

export default ProfilePage;