# Quickstart: Website Content Embedding & Vector Storage Pipeline

## Prerequisites

1. **Environment Setup**:
   - Python 3.12+
   - `uv` package manager installed
   - Access to Cohere API (API key)
   - Access to Qdrant Cloud (API key and URL)

2. **Environment Variables**:
   ```bash
   # Create .env file with the following variables:
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_URL=your_qdrant_cloud_url
   QDRANT_API_KEY=your_qdrant_api_key
   BOOK_BASE_URL=https://your-book.vercel.app  # Base URL for the Docusaurus book
   Target Site:https://ai-native-book-frontend.vercel.app
   site Map URL:https://ai-native-book-frontend.vercel.app/sitemap.xml
   ```

## Installation

1. **Setup Project Environment**:
   ```bash
   cd backend/
   uv venv  # Create virtual environment using uv
   source .venv/bin/activate  # Activate the environment
   ```

2. **Install Dependencies**:
   ```bash
   # Add required packages to requirements.txt
   uv pip install httpx beautifulsoup4 cohere qdrant-client python-dotenv langchain-text-splitters
   ```

## Usage

1. **Run the Content Embedding Pipeline**:
   ```bash
   # Navigate to backend
   cd backend/

   # Run the embedding script
   python -m services.content_embedding.crawl_and_embed
   ```

2. **Alternative: Run as a Service**:
   ```bash
   # Add endpoint to main FastAPI app
   # The service will be available at /embed-content endpoint
   ```

## Configuration

- **Chunking Parameters**: Default 512 tokens with 20% overlap (102 tokens)
- **Crawling Delays**: Default 1 second between requests to respect rate limits
- **Batch Size**: Default 10 embeddings per batch for Qdrant insertion

## Verification

1. **Check Qdrant Collection**: Verify embeddings are stored in the configured Qdrant collection
2. **Check Logs**: Review the embedding process logs for any errors
3. **Test Retrieval**: Use the retrieval service to test if embeddings can be found via similarity search

## Troubleshooting

- **Rate Limiting**: If encountering rate limits, increase the delay between requests
- **Memory Issues**: If processing large sites, consider processing in smaller batches
- **Cohere API Errors**: Verify API key and check for quota limits
- **Qdrant Connection**: Verify URL and API key for Qdrant Cloud



