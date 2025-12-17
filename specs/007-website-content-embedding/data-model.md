# Data Model: Website Content Embedding & Vector Storage Pipeline

## Entities

### CrawledWebPage
- **url**: string (primary key) - The URL of the web page
- **title**: string - The title of the page
- **content**: string - The extracted text content
- **html_content**: string - The original HTML content (optional, for debugging)
- **metadata**: JSON - Additional metadata (section hierarchy, etc.)
- **created_at**: datetime - When the page was crawled
- **updated_at**: datetime - When the page was last updated

### TextChunk
- **id**: string (primary key) - Unique identifier for the chunk
- **page_url**: string (foreign key) - Reference to the source page
- **content**: string - The chunked text content
- **chunk_index**: integer - Position of the chunk in the original document
- **token_count**: integer - Number of tokens in the chunk
- **metadata**: JSON - Metadata including source URL, title, etc.
- **created_at**: datetime - When the chunk was created

### VectorEmbedding
- **id**: string (primary key) - Unique identifier for the embedding
- **chunk_id**: string (foreign key) - Reference to the source chunk
- **vector**: array[float] - The embedding vector (1024+ dimensions)
- **page_url**: string - Direct reference to source page
- **metadata**: JSON - Rich metadata (URL, title, section, etc.)
- **created_at**: datetime - When the embedding was created

### EmbeddingCollection
- **name**: string (primary key) - Name of the Qdrant collection
- **description**: string - Description of the collection
- **vector_size**: integer - Size of the embedding vectors
- **created_at**: datetime - When the collection was created
- **status**: string - Status of the collection (active, processing, etc.)

## Relationships

1. **CrawledWebPage** 1 → * **TextChunk**: One web page can be split into multiple text chunks
2. **TextChunk** 1 → 1 **VectorEmbedding**: Each text chunk produces one vector embedding
3. **EmbeddingCollection** 1 → * **VectorEmbedding**: One collection contains multiple embeddings

## Indexes

- CrawledWebPage.url: For fast lookup by URL
- TextChunk.page_url: For fast lookup of chunks by page
- VectorEmbedding.page_url: For fast lookup of embeddings by page
- VectorEmbedding.metadata: For semantic search capabilities