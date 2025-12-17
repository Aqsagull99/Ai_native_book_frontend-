# Quickstart: Vector Retrieval Validation

## Prerequisites

1. **Environment Setup**
   - Python 3.12+
   - Access to Qdrant Cloud instance with `ai_native_book` collection
   - Cohere API key with embedding capabilities
   - Existing embedded content in the Qdrant collection

2. **Environment Variables**
   ```bash
   export QDRANT_URL="your-qdrant-cloud-url"
   export QDRANT_API_KEY="your-qdrant-api-key"
   export COHERE_API_KEY="your-cohere-api-key"
   ```

## Installation

1. **Install Dependencies**
   ```bash
   cd backend
   pip install -r requirements.txt
   ```

2. **Verify Setup**
   ```bash
   python -c "import qdrant_client, cohere; print('Dependencies available')"
   ```

## Running Validation

1. **Execute Basic Validation**
   ```bash
   cd backend
   python test_retrieval.py
   ```

2. **Run Comprehensive Validation**
   ```bash
   cd backend
   python -m src.content_embedding.retrieval_service --validate-all
   ```

3. **Custom Query Testing**
   ```bash
   cd backend
   python -c "
   from src.content_embedding.retrieval_service import validate_semantic_search
   result = validate_semantic_search('your query here', top_k=5)
   print(result)
   "
   ```

4. **Run Full Validation Pipeline**
   ```bash
   cd backend
   python -c "
   from src.content_embedding.retrieval_service import run_full_validation_pipeline_and_verify_all_success_criteria
   result = run_full_validation_pipeline_and_verify_all_success_criteria()
   print('Full validation result:', result['overall_success'])
   "
   ```

5. **Run 100-Query Stress Test**
   ```bash
   cd backend
   python -c "
   from src.content_embedding.retrieval_service import create_100_query_stress_test
   result = create_100_query_stress_test()
   print(f'Stress test completed: {result[\"success_rate_percentage\"]:.2f}% success rate')
   print(f'Average response time: {result[\"average_response_time\"]:.4f}s')
   print(f'Queries under 2s: {result[\"percent_queries_under_2s\"]:.2f}%')
   "
   ```

## Expected Output

The validation will produce:
- Connection status to Qdrant Cloud
- Number of points in the collection
- Sample search results with metadata
- Validation metrics (relevance, metadata accuracy, response time)
- Success/failure status of the retrieval pipeline
- Comprehensive validation report with success criteria verification
- Performance metrics showing compliance with requirements

## Troubleshooting

- **Connection Issues**: Verify QDRANT_URL and QDRANT_API_KEY are correct
- **Embedding Issues**: Ensure Cohere API key has proper permissions
- **No Results**: Confirm the `ai_native_book` collection has embedded content
- **Metadata Issues**: Check that all required metadata fields are present in stored vectors
- **Performance Issues**: If queries are slow, check your network connection to Qdrant Cloud
- **Validation Failures**: Review the validation report to identify specific failing criteria