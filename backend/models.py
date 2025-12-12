from sqlalchemy import Column, Integer, String, DateTime, Text
from sqlalchemy.sql import func
from sqlalchemy.ext.declarative import declarative_base
from backend.database import Base


class UserProfile(Base):
    __tablename__ = "user_profiles"

    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(String, unique=True, index=True, nullable=False)  # This will reference Better-Auth user ID
    email = Column(String, unique=True, index=True, nullable=False)  # Store email for reference
    software_experience = Column(String, nullable=False)  # beginner, intermediate, advanced
    hardware_experience = Column(String, nullable=False)  # none, basic, advanced
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())