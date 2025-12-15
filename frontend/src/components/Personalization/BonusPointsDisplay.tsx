import React from 'react';
import { usePersonalization } from '../../contexts/PersonalizationContext';

interface BonusPointsDisplayProps {
  className?: string;
  showHistory?: boolean;
}

const BonusPointsDisplay: React.FC<BonusPointsDisplayProps> = ({
  className = '',
  showHistory = false
}) => {
  const { bonusPoints } = usePersonalization();

  if (!bonusPoints) {
    return (
      <div className={`bonus-points-display empty ${className}`}>
        <p>No bonus points data available</p>
      </div>
    );
  }

  return (
    <div
      className={`bonus-points-display ${className}`}
      style={{
        background: 'rgba(255, 255, 255, 0.05)',
        borderRadius: '8px',
        padding: '16px',
        margin: '16px 0',
        border: '1px solid rgba(255, 255, 255, 0.1)'
      }}
    >
      <div style={{
        display: 'flex',
        justifyContent: 'space-between',
        alignItems: 'center',
        marginBottom: '12px'
      }}>
        <h3 style={{ color: '#e0e0e0', margin: 0 }}>Bonus Points</h3>
        <div style={{
          display: 'flex',
          alignItems: 'baseline',
          gap: '4px'
        }}>
          <span style={{
            fontSize: '2em',
            fontWeight: 'bold',
            color: '#f9a825'
          }}>
            {bonusPoints.totalPoints}
          </span>
          <span style={{
            fontSize: '0.9em',
            color: '#b0b0b0'
          }}>
            points
          </span>
        </div>
      </div>

      {showHistory && bonusPoints.pointsBreakdown && bonusPoints.pointsBreakdown.length > 0 && (
        <div>
          <h4 style={{ margin: '16px 0 8px 0', color: '#e0e0e0' }}>Recent Activity</h4>
          <ul style={{ listStyle: 'none', padding: 0, margin: 0 }}>
            {bonusPoints.pointsBreakdown.slice(0, 5).map((entry, index) => (
              <li
                key={index}
                style={{
                  display: 'flex',
                  justifyContent: 'space-between',
                  padding: '8px 0',
                  borderBottom: index === bonusPoints.pointsBreakdown.slice(0, 5).length - 1 ? 'none' : '1px solid rgba(255, 255, 255, 0.1)',
                  color: '#e0e0e0'
                }}
              >
                <div style={{ flex: 1, fontSize: '0.9em', color: '#b0b0b0' }}>
                  {entry.chapterId}
                </div>
                <div style={{
                  fontWeight: 'bold',
                  color: '#a5d6a7',
                  minWidth: '60px',
                  textAlign: 'right',
                  margin: '0 10px'
                }}>
                  +{entry.points} pts
                </div>
                <div style={{
                  fontSize: '0.8em',
                  color: '#9e9e9e',
                  textAlign: 'right',
                  minWidth: '80px'
                }}>
                  {entry.earnedAt ? new Date(entry.earnedAt).toLocaleDateString() : 'Unknown date'}
                </div>
              </li>
            ))}
          </ul>
        </div>
      )}
    </div>
  );
};

export default BonusPointsDisplay;


