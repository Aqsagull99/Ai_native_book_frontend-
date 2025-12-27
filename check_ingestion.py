#!/usr/bin/env python3
"""
Script to check if book content has been properly ingested into Qdrant
"""

import asyncio
import os
import sys
import logging

# Add the backend directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from src.content_embedding.qdrant_service import validate_qdrant_connection, get_collection_info, get_sample_point
from src.content_embedding.retrieval_service import validate_qdrant_search_connection, get_validation_metrics

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

async def check_qdrant_ingestion():
    """Check if the Qdrant collection has been properly populated with book content."""

    print("🔍 Checking Qdrant ingestion status...")

    # Check connection to Qdrant
    print("\n1. Checking Qdrant connection...")
    connection_valid = validate_qdrant_connection()
    print(f"   Connection valid: {connection_valid}")

    if not connection_valid:
        print("❌ Cannot connect to Qdrant. Please check your environment variables:")
        print("   - QDRANT_URL")
        print("   - QDRANT_API_KEY")
        return False

    # Get collection info
    print("\n2. Getting collection information...")
    collection_info = get_collection_info()
    print(f"   Collection name: {collection_info.get('name', 'N/A')}")
    print(f"   Total points: {collection_info.get('total_points', 0)}")
    print(f"   Vector size: {collection_info.get('vector_size', 0)}")
    print(f"   Status: {collection_info.get('status', 'N/A')}")

    # Check if collection is empty
    if collection_info.get('total_points', 0) == 0:
        print("❌ Collection is empty - no book content has been ingested")
        return False
    else:
        print(f"✅ Collection contains {collection_info.get('total_points', 0)} embeddings")

    # Get validation metrics
    print("\n3. Getting validation metrics...")
    metrics = get_validation_metrics()
    print(f"   Total points: {metrics.get('total_points_in_collection', 0)}")
    print(f"   Collection status: {metrics.get('collection_status', 'N/A')}")
    print(f"   Vector size: {metrics.get('vector_size', 0)}")

    # Get a sample point to verify content
    print("\n4. Getting sample point from collection...")
    sample_point = get_sample_point()
    if sample_point:
        print(f"   Sample point ID: {sample_point.get('id', 'N/A')}")
        payload = sample_point.get('payload', {})
        print(f"   Content preview: {payload.get('content', '')[:100]}...")
        print(f"   URL: {payload.get('url', 'N/A')}")
        print(f"   Title: {payload.get('title', 'N/A')}")
        print(f"   Chunk index: {payload.get('chunk_index', 'N/A')}")
    else:
        print("   ❌ No sample point available")
        return False

    print("\n✅ Qdrant ingestion check completed successfully!")
    print(f"   The collection contains {collection_info.get('total_points', 0)} book content chunks")
    return True

async def test_search_functionality():
    """Test basic search functionality to ensure the ingestion worked properly."""

    print("\n🔍 Testing search functionality...")

    # Check if search connection is valid
    search_connection_valid = validate_qdrant_search_connection()
    print(f"   Search connection valid: {search_connection_valid}")

    if not search_connection_valid:
        print("❌ Search connection is not valid")
        return False

    # Try a simple search to verify functionality
    try:
        from src.content_embedding.retrieval_service import validate_semantic_search

        print("\n   Testing search with sample query: 'AI and machine learning'")
        response = validate_semantic_search("AI and machine learning", top_k=3)

        print(f"   Query: {response.query_text}")
        print(f"   Results returned: {len(response.results)}")
        print(f"   Execution time: {response.execution_time:.4f}s")

        if len(response.results) > 0:
            print("\n   Sample result:")
            first_result = response.results[0]
            print(f"     Content preview: {first_result.content_chunk[:100]}...")
            print(f"     Similarity score: {first_result.similarity_score:.4f}")
            print(f"     URL: {first_result.metadata.get('url', 'N/A')}")
            print(f"     Title: {first_result.metadata.get('title', 'N/A')}")

            print("\n✅ Search functionality is working correctly!")
            return True
        else:
            print("❌ No results returned from search")
            return False

    except Exception as e:
        print(f"❌ Error during search test: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

async def main():
    """Main function to run all checks."""
    print("🚀 Starting Qdrant ingestion verification...")

    # Check if environment variables are set
    print("\n📋 Checking environment variables...")
    required_vars = ['QDRANT_URL', 'QDRANT_API_KEY']
    missing_vars = [var for var in required_vars if not os.getenv(var)]

    if missing_vars:
        print(f"❌ Missing required environment variables: {missing_vars}")
        print("   Please set these environment variables before running the check")
        return False

    print("   All required environment variables are set")

    # Run ingestion check
    ingestion_ok = await check_qdrant_ingestion()

    if ingestion_ok:
        # Run search functionality test
        search_ok = await test_search_functionality()

        if search_ok:
            print("\n🎉 All checks passed! The book content has been properly ingested into Qdrant.")
            return True
        else:
            print("\n❌ Search functionality test failed.")
            return False
    else:
        print("\n❌ Qdrant ingestion check failed.")
        return False

if __name__ == "__main__":
    success = asyncio.run(main())
    sys.exit(0 if success else 1)