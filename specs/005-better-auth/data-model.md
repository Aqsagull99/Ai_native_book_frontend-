# Data Model: Better-Auth Signup & Signin

## Entities

### UserProfile
- **id**: Integer (Primary Key, Auto-increment)
- **user_id**: String (Unique, references Better-Auth user ID)
- **email**: String (Unique, not null)
- **software_experience**: String (enum: "beginner", "intermediate", "advanced", not null)
- **hardware_experience**: String (enum: "none", "basic", "advanced", not null)
- **created_at**: DateTime (not null, default: current timestamp)
- **updated_at**: DateTime (nullable, updates on modification)

**Validation Rules**:
- email must be valid email format
- software_experience must be one of the allowed values
- hardware_experience must be one of the allowed values
- email must be unique

**Relationships**:
- One-to-one with Better-Auth user (via user_id)

## Database Schema

```sql
CREATE TABLE user_profiles (
    id SERIAL PRIMARY KEY,
    user_id VARCHAR(255) UNIQUE NOT NULL,
    email VARCHAR(255) UNIQUE NOT NULL,
    software_experience VARCHAR(20) NOT NULL CHECK (software_experience IN ('beginner', 'intermediate', 'advanced')),
    hardware_experience VARCHAR(20) NOT NULL CHECK (hardware_experience IN ('none', 'basic', 'advanced')),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW() NOT NULL,
    updated_at TIMESTAMP WITH TIME ZONE
);
```

## Indexes
- Index on email for fast lookup
- Index on user_id for fast authentication lookups