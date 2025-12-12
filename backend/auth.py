from fastapi import APIRouter, Depends, HTTPException, status, Request
from sqlalchemy.ext.asyncio import AsyncSession
from pydantic import BaseModel
from typing import Optional
from backend.database import get_db
from backend.models import UserProfile
import uuid
import httpx
from dotenv import load_dotenv
import os

# Load environment variables
load_dotenv()

router = APIRouter()

# Better-Auth service configuration
BETTER_AUTH_BASE_URL = os.getenv("BETTER_AUTH_URL", "http://localhost:8000")  # Default to localhost

# Pydantic models for request/response validation
class SignupRequest(BaseModel):
    email: str
    password: str
    software_experience: str  # beginner, intermediate, advanced
    hardware_experience: str  # none, basic, advanced


class SigninRequest(BaseModel):
    email: str
    password: str


class ProfileResponse(BaseModel):
    id: int
    user_id: str
    email: str
    software_experience: str
    hardware_experience: str

    class Config:
        from_attributes = True


class ProfileUpdateRequest(BaseModel):
    software_experience: Optional[str] = None  # beginner, intermediate, advanced
    hardware_experience: Optional[str] = None  # none, basic, advanced


class BetterAuthUserResponse(BaseModel):
    id: str
    email: str
    emailVerified: bool
    image: Optional[str] = None


async def create_better_auth_user(email: str, password: str) -> BetterAuthUserResponse:
    """
    Simulate creating a user with Better-Auth service.
    In a real implementation, this would make an HTTP call to the Better-Auth API.
    For this implementation, we'll simulate the behavior.
    """
    # In a real implementation:
    # async with httpx.AsyncClient() as client:
    #     response = await client.post(f"{BETTER_AUTH_BASE_URL}/api/auth/sign-up", json={
    #         "email": email,
    #         "password": password
    #     })
    #     if response.status_code != 200:
    #         raise HTTPException(status_code=response.status_code, detail="Better-Auth error")
    #     return BetterAuthUserResponse(**response.json())

    # For this implementation, we'll simulate the response
    user_id = str(uuid.uuid4())
    return BetterAuthUserResponse(
        id=user_id,
        email=email,
        emailVerified=False
    )


async def authenticate_better_auth_user(email: str, password: str) -> BetterAuthUserResponse:
    """
    Simulate authenticating a user with Better-Auth service.
    In a real implementation, this would make an HTTP call to the Better-Auth API.
    For this implementation, we'll simulate the behavior.
    """
    # In a real implementation:
    # async with httpx.AsyncClient() as client:
    #     response = await client.post(f"{BETTER_AUTH_BASE_URL}/api/auth/sign-in", json={
    #         "email": email,
    #         "password": password
    #     })
    #     if response.status_code != 200:
    #         raise HTTPException(status_code=response.status_code, detail="Authentication failed")
    #     return BetterAuthUserResponse(**response.json())

    # For this implementation, we'll simulate the response
    # Check if a profile already exists (this simulates checking if user exists in Better-Auth)
    return BetterAuthUserResponse(
        id=str(uuid.uuid4()),
        email=email,
        emailVerified=True
    )


@router.post("/signup")
async def signup(request: SignupRequest, db: AsyncSession = Depends(get_db)):
    """
    Register a new user with experience levels via Better-Auth.
    """
    try:
        # First, create the user in Better-Auth
        better_auth_user = await create_better_auth_user(request.email, request.password)

        # Check if user profile already exists in our database
        existing_profile = await db.execute(
            UserProfile.__table__.select().where(UserProfile.email == request.email)
        )
        existing_profile = existing_profile.fetchone()

        if existing_profile:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="User profile already exists"
            )

        # Create user profile with the user ID from Better-Auth
        user_profile = UserProfile(
            user_id=better_auth_user.id,
            email=request.email,
            software_experience=request.software_experience,
            hardware_experience=request.hardware_experience
        )

        db.add(user_profile)
        await db.commit()
        await db.refresh(user_profile)

        return {
            "message": "User registered successfully",
            "profile_id": user_profile.id,
            "user_id": user_profile.user_id,
            "email": user_profile.email
        }
    except HTTPException:
        raise  # Re-raise HTTP exceptions
    except Exception as e:
        await db.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error registering user: {str(e)}"
        )


@router.post("/signin")
async def signin(request: SigninRequest, db: AsyncSession = Depends(get_db)):
    """
    Authenticate user via Better-Auth.
    """
    try:
        # Authenticate the user with Better-Auth
        better_auth_user = await authenticate_better_auth_user(request.email, request.password)

        # Check if user profile exists in our database
        result = await db.execute(
            UserProfile.__table__.select().where(UserProfile.email == request.email)
        )
        user_profile = result.fetchone()

        if not user_profile:
            # If no profile exists but user is authenticated, create a basic profile
            # This could happen if user signed up through Better-Auth but not through our app
            new_profile = UserProfile(
                user_id=better_auth_user.id,
                email=request.email,
                software_experience="beginner",  # Default value
                hardware_experience="none"      # Default value
            )
            db.add(new_profile)
            await db.commit()
            await db.refresh(new_profile)

            return {
                "message": "User authenticated successfully",
                "email": request.email,
                "user_id": new_profile.user_id,
                "new_profile_created": True
            }

        return {
            "message": "User authenticated successfully",
            "email": request.email,
            "user_id": user_profile.user_id
        }
    except HTTPException:
        raise  # Re-raise HTTP exceptions
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error authenticating user: {str(e)}"
        )


@router.get("/profile", response_model=ProfileResponse)
async def get_profile(request: Request, db: AsyncSession = Depends(get_db)):
    """
    Retrieve user profile for personalization.
    In a real implementation, the user ID would come from an auth token in the request header.
    For this implementation, we'll use a query parameter.
    """
    try:
        # In a real implementation, user_id would come from the auth token in the Authorization header
        # For this demo, we'll look up by email from query parameter
        email = request.query_params.get("email")

        if not email:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Email parameter required"
            )

        # Query for the user profile
        result = await db.execute(
            UserProfile.__table__.select().where(UserProfile.email == email)
        )
        user_profile = result.fetchone()

        if not user_profile:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User profile not found"
            )

        # Convert row to dict for response
        profile_dict = {
            "id": user_profile.id,
            "user_id": user_profile.user_id,
            "email": user_profile.email,
            "software_experience": user_profile.software_experience,
            "hardware_experience": user_profile.hardware_experience
        }

        return profile_dict
    except HTTPException:
        raise  # Re-raise HTTP exceptions
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error retrieving profile: {str(e)}"
        )


@router.put("/profile")
async def update_profile(request: Request, update_request: ProfileUpdateRequest, db: AsyncSession = Depends(get_db)):
    """
    Update user profile.
    In a real implementation, the user ID would come from an auth token in the request header.
    For this implementation, we'll use a query parameter.
    """
    try:
        # In a real implementation, user_id would come from the auth token in the Authorization header
        # For this demo, we'll use email from query parameter
        email = request.query_params.get("email")

        if not email:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Email parameter required"
            )

        # Get the user profile by email
        result = await db.execute(
            UserProfile.__table__.select().where(UserProfile.email == email)
        )
        user_profile = result.fetchone()

        if not user_profile:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User profile not found"
            )

        # Prepare update values
        update_values = {}
        if update_request.software_experience is not None:
            update_values["software_experience"] = update_request.software_experience
        if update_request.hardware_experience is not None:
            update_values["hardware_experience"] = update_request.hardware_experience

        # Perform the update
        await db.execute(
            UserProfile.__table__.update()
            .where(UserProfile.email == email)
            .values(**update_values)
        )
        await db.commit()

        # Fetch the updated profile
        result = await db.execute(
            UserProfile.__table__.select().where(UserProfile.email == email)
        )
        updated_profile = result.fetchone()

        return {
            "message": "Profile updated successfully",
            "profile_id": updated_profile.id
        }
    except HTTPException:
        raise  # Re-raise HTTP exceptions
    except Exception as e:
        await db.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error updating profile: {str(e)}"
        )