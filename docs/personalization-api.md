# Personalization API Documentation

## Overview
The Personalization API allows users to customize chapter content and earn bonus points. All endpoints are prefixed with `/api/personalization`.

## Authentication
All endpoints require a valid authentication token in the `Authorization` header:
```
Authorization: Bearer <token>
```

## Endpoints

### POST /api/personalization/activate
Activate personalization for a chapter and earn bonus points.

#### Request
```json
{
  "chapter_id": "string",
  "preferences": {
    "theme": "string",
    "fontSize": "string",
    "learning_style": "string"
  }
}
```

#### Query Parameters
- `chapter_id` (required): The ID of the chapter to personalize

#### Response
```json
{
  "success": true,
  "message": "Content personalized successfully, 50 bonus points awarded",
  "points_earned": 50,
  "personalized_content": {
    "chapter_id": "string",
    "title": "string",
    "content": "string",
    "preferences_applied": {},
    "personalization_applied": true
  }
}
```

#### Error Responses
- `401`: User not authenticated
- `409`: Personalization already activated for this chapter (duplicate)
- `400`: Invalid request or chapter doesn't exist
- `500`: Internal server error

### GET /api/personalization/status
Get the personalization status for a specific chapter.

#### Query Parameters
- `chapter_id` (required): The ID of the chapter to check

#### Response
```json
{
  "is_personalized": true,
  "preferences": {},
  "points_earned": 50
}
```

### GET /api/personalization/preferences
Get all personalization preferences set by the user.

#### Response
```json
{
  "preferences": [
    {
      "chapter_id": "string",
      "preferences": {},
      "created_at": "string",
      "points_earned": 50
    }
  ]
}
```

### GET /api/personalization/user/bonus-points
Get the total bonus points earned by the user.

#### Response
```json
{
  "total_points": 150,
  "points_breakdown": [
    {
      "chapter_id": "string",
      "points": 50,
      "earned_at": "string"
    }
  ]
}
```

## Usage Examples

### Activate Personalization
```bash
curl -X POST \
  -H "Authorization: Bearer your-token-here" \
  -H "Content-Type: application/json" \
  -d '{"chapter_id": "chapter-1-intro", "preferences": {"theme": "dark"}}' \
  "http://localhost:8000/api/personalization/activate"
```

### Check Personalization Status
```bash
curl -X GET \
  -H "Authorization: Bearer your-token-here" \
  "http://localhost:8000/api/personalization/status?chapter_id=chapter-1-intro"
```

### Get User Bonus Points
```bash
curl -X GET \
  -H "Authorization: Bearer your-token-here" \
  "http://localhost:8000/api/personalization/user/bonus-points"
```

## Rate Limits
- 100 requests per hour per user
- 10 personalization attempts per chapter (enforced by duplicate prevention)