#!/usr/bin/env python3
"""
Test script to validate the enhanced Aqsa Gull AI Agent functionality
"""

import requests
import time
import json

BASE_URL = "http://localhost:8000"

def test_query(query, description):
    """Test a query and print the response."""
    print(f"\n--- {description} ---")
    print(f"Query: {query}")

    try:
        response = requests.post(
            f"{BASE_URL}/api/rag/query",
            json={
                "query_text": query,
                "top_k": 3
            },
            headers={"Content-Type": "application/json"},
            timeout=30
        )

        if response.status_code == 200:
            data = response.json()
            print(f"Answer: {data['answer'][:200]}...")
            print(f"Retrieved chunks: {len(data['retrieved_chunks'])}")
            print(f"Confidence: {data['confidence_score']:.2f}")
        else:
            print(f"Error: {response.status_code} - {response.text}")
    except Exception as e:
        print(f"Exception: {str(e)}")

def main():
    """Test various query types to validate the enhanced agent."""
    print("Testing Enhanced Aqsa Gull AI Agent...")
    print("Waiting for server to be ready...")
    time.sleep(2)  # Wait for server to be ready

    # Test 1: Book-related query
    test_query("What is ROS2?", "Book-related query")

    # Test 2: Author-related query
    test_query("Who is Aqsa Gull?", "Author-related query")

    # Test 3: General technical query
    test_query("Explain machine learning", "General technical query")

    # Test 4: Casual/funny query
    test_query("Are you human?", "Casual/funny query")

    # Test 5: Motivational query
    test_query("I'm struggling with programming", "Motivational query")

    print("\n--- Testing Complete ---")

if __name__ == "__main__":
    main()