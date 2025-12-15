# Quickstart: Better-Auth Signup & Signin

## Prerequisites
- Python 3.12+
- Node.js 18+
- uv package manager (or pip)
- Access to Neon Postgres database

## Setup

### Backend Setup
1. Install Python dependencies:
   ```bash
   uv pip install fastapi uvicorn sqlalchemy asyncpg python-dotenv pydantic
   # Or using pip:
   pip install fastapi uvicorn sqlalchemy asyncpg python-dotenv pydantic
   ```

2. Set up environment variables:
   ```bash
   # Create .env file
   echo "DATABASE_URL=postgresql+asyncpg://username:password@host:port/dbname" > .env
   ```

3. Run the backend:
   ```bash
   uvicorn backend.main:app --reload --port 8000
   ```

### Frontend Setup
1. Navigate to frontend directory:
   ```bash
   cd frontend
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Start the development server:
   ```bash
   npm start
   ```

## API Endpoints

### Authentication
- `POST /api/auth/signup` - Register a new user
- `POST /api/auth/signin` - Authenticate a user
- `GET /api/auth/profile` - Retrieve user profile
- `PUT /api/auth/profile` - Update user profile

### Health Check
- `GET /health` - Check API health status

## Testing

### Backend Tests
```bash
python -m pytest tests/
```

### Frontend Tests
```bash
cd frontend
npm test
```

## Configuration

### Environment Variables
- `DATABASE_URL` - Neon Postgres connection string
- `BETTER_AUTH_URL` - Better-Auth service URL (default: http://localhost:8000)

## Running the Application

1. Start the backend:
   ```bash
   uvicorn backend.main:app --reload --port 8000
   ```

2. In a separate terminal, start the frontend:
   ```bash
   cd frontend && npm start
   ```

3. Access the application at `http://localhost:3000`

## Key Features

- User registration with experience level collection
- Secure authentication with Better-Auth
- Profile management
- Content personalization based on user experience
- Responsive design for all devices