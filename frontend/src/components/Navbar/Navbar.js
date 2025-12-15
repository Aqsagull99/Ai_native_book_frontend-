import React from 'react';
import { useAuth } from '../../contexts/AuthContext';
import { useNavigate } from '@docusaurus/router';

const Navbar = () => {
  const { user, loading, logout } = useAuth();
  const navigate = useNavigate();

  const handleLogout = async () => {
    try {
      await logout();
      navigate('/');
    } catch (error) {
      console.error('Logout error:', error);
    }
  };

  if (loading) {
    return null; // Or a loading spinner
  }

  return (
    <div className="navbar__item navbar__items">
      {user ? (
        // User is logged in - show profile dropdown
        <div className="dropdown dropdown--right dropdown--navbar">
          <button className="navbar__link dropdown__trigger">
            {user.email}
          </button>
          <ul className="dropdown__menu">
            <li>
              <a className="dropdown__link" href="/profile">
                Profile
              </a>
            </li>
            <li>
              <button
                className="dropdown__link dropdown__button"
                onClick={handleLogout}
              >
                Logout
              </button>
            </li>
          </ul>
        </div>
      ) : (
        // User is not logged in - show sign in/up buttons
        <>
          <a className="navbar__link" href="/signin">
            Sign In
          </a>
          <a className="navbar__link" href="/signup">
            Sign Up
          </a>
        </>
      )}
    </div>
  );
};

export default Navbar;