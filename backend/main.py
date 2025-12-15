from fastapi import FastAPI, Depends, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from sqlalchemy.ext.asyncio import AsyncSession
from backend.database import get_db
from backend.auth import router as auth_router
from backend.endpoints.personalization import router as personalization_router, bonus_points_router
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Create FastAPI app
app = FastAPI(
    title="Better-Auth API",
    description="API for user authentication and profile management",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include authentication routes
app.include_router(auth_router, prefix="/api/auth", tags=["auth"])

# Include personalization routes
app.include_router(personalization_router, prefix="/api", tags=["personalization"])

# Include user bonus points routes
app.include_router(bonus_points_router, prefix="/api", tags=["user"])

@app.get("/")
async def root():
    return {"message": "Better-Auth API is running!"}

@app.get("/health")
async def health_check():
    return {"status": "healthy"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=int(os.getenv("PORT", 8000)))
