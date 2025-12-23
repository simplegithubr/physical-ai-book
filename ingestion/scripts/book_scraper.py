"""
Book scraper script for the RAG chatbot system.
Scrapes content from Docusaurus sites and indexes it.
"""
import asyncio
import sys
import os
import argparse
from urllib.parse import urljoin, urlparse
from typing import Set, List, Dict, Any
import requests
from bs4 import BeautifulSoup
import time

# Add the backend src directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'backend', 'src'))

from services.ingestion_service import BookIngestionService
from utils.logging import get_logger


logger = get_logger(__name__)


class BookScraper:
    """
    Scrapes content from book websites (especially Docusaurus-based sites).
    """

    def __init__(self, base_url: str, max_pages: int = 100):
        """
        Initialize the scraper.

        Args:
            base_url: Base URL of the book site
            max_pages: Maximum number of pages to scrape
        """
        self.base_url = base_url
        self.max_pages = max_pages
        self.visited_urls: Set[str] = set()
        self.to_visit: List[str] = [base_url]
        self.session = requests.Session()
        self.session.headers.update({
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
        })

    def is_valid_url(self, url: str) -> bool:
        """
        Check if a URL is valid for scraping.

        Args:
            url: URL to check

        Returns:
            bool: True if URL is valid for scraping
        """
        parsed_base = urlparse(self.base_url)
        parsed_url = urlparse(url)

        # Same domain check
        if parsed_base.netloc != parsed_url.netloc:
            return False

        # Exclude certain file types
        excluded_extensions = ['.pdf', '.jpg', '.jpeg', '.png', '.gif', '.zip', '.exe']
        if any(url.lower().endswith(ext) for ext in excluded_extensions):
            return False

        # Exclude certain paths
        excluded_paths = ['/api/', '/auth/', '/admin/']
        url_lower = url.lower()
        if any(path in url_lower for path in excluded_paths):
            return False

        return True

    def extract_links(self, html_content: str, current_url: str) -> List[str]:
        """
        Extract all valid links from HTML content.

        Args:
            html_content: HTML content to extract links from
            current_url: Current page URL for relative link resolution

        Returns:
            List of extracted URLs
        """
        soup = BeautifulSoup(html_content, 'html.parser')
        links = []

        for link in soup.find_all('a', href=True):
            href = link['href']

            # Resolve relative URLs
            absolute_url = urljoin(current_url, href)

            if self.is_valid_url(absolute_url):
                # Normalize URL (remove fragments, ensure consistent format)
                normalized_url = absolute_url.split('#')[0]  # Remove fragment
                normalized_url = normalized_url.split('?')[0]  # Remove query params for now

                if normalized_url not in self.visited_urls:
                    links.append(normalized_url)

        return links

    def extract_content(self, html_content: str) -> str:
        """
        Extract main content from HTML.

        Args:
            html_content: HTML content to extract text from

        Returns:
            Extracted text content
        """
        soup = BeautifulSoup(html_content, 'html.parser')

        # Remove script and style elements
        for script in soup(["script", "style", "nav", "header", "footer", "aside"]):
            script.decompose()

        # Try to find main content areas (common in Docusaurus sites)
        main_content = (
            soup.find('main') or
            soup.find('article') or
            soup.find('div', class_=lambda x: x and 'main' in x.lower()) or
            soup.find('div', class_=lambda x: x and 'content' in x.lower()) or
            soup.find('div', class_=lambda x: x and 'container' in x.lower()) or
            soup.body
        )

        if main_content:
            text = main_content.get_text()
        else:
            text = soup.get_text()

        # Clean up text (remove extra whitespace)
        lines = (line.strip() for line in text.splitlines())
        chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
        clean_text = ' '.join(chunk for chunk in chunks if chunk)

        return clean_text

    async def scrape_page(self, url: str) -> Dict[str, Any]:
        """
        Scrape a single page.

        Args:
            url: URL to scrape

        Returns:
            Dict with page content and metadata
        """
        try:
            logger.info(f"Scraping: {url}")
            response = self.session.get(url, timeout=10)
            response.raise_for_status()

            content = self.extract_content(response.text)
            links = self.extract_links(response.text, url)

            page_data = {
                "url": url,
                "content": content,
                "links": links,
                "scraped_at": time.time()
            }

            logger.info(f"Scraped {len(content)} characters from {url}")
            return page_data

        except Exception as e:
            logger.error(f"Error scraping {url}: {str(e)}")
            return {
                "url": url,
                "content": "",
                "links": [],
                "scraped_at": time.time(),
                "error": str(e)
            }

    async def scrape_book(self) -> List[Dict[str, Any]]:
        """
        Scrape the entire book site.

        Returns:
            List of scraped page data
        """
        all_pages = []

        while self.to_visit and len(self.visited_urls) < self.max_pages:
            # Get next URL to visit
            current_url = self.to_visit.pop(0)

            if current_url in self.visited_urls:
                continue

            # Scrape the page
            page_data = await self.scrape_page(current_url)

            # Mark as visited
            self.visited_urls.add(current_url)

            # Add to results if we got content
            if page_data.get("content") and not page_data.get("error"):
                all_pages.append(page_data)

                # Add new links to visit list
                for link in page_data["links"]:
                    if link not in self.visited_urls and link not in self.to_visit:
                        self.to_visit.append(link)

            # Be respectful with requests
            await asyncio.sleep(0.5)

        logger.info(f"Scraping completed: {len(all_pages)} pages scraped from {len(self.visited_urls)} URLs")
        return all_pages

    def combine_pages_content(self, pages: List[Dict[str, Any]]) -> str:
        """
        Combine all page content into a single text string.

        Args:
            pages: List of page data

        Returns:
            Combined content string
        """
        combined_content = []
        for page in pages:
            if page.get("content"):
                # Add a separator between pages
                combined_content.append(f"\n\n--- PAGE BREAK ---\nURL: {page['url']}\n\n{page['content']}\n")

        return "".join(combined_content)


async def main():
    """
    Main function to run the book scraper.
    """
    parser = argparse.ArgumentParser(description='Scrape book content for RAG system')
    parser.add_argument('url', help='Base URL of the book site to scrape')
    parser.add_argument('--max-pages', type=int, default=50, help='Maximum number of pages to scrape (default: 50)')
    parser.add_argument('--output', help='Output file to save combined content')
    parser.add_argument('--ingest', action='store_true', help='Ingest content directly into the RAG system')

    args = parser.parse_args()

    # Create scraper
    scraper = BookScraper(args.url, args.max_pages)

    # Scrape the book
    logger.info(f"Starting to scrape book from: {args.url}")
    pages = await scraper.scrape_book()

    # Combine all content
    combined_content = scraper.combine_pages_content(pages)

    logger.info(f"Combined content length: {len(combined_content)} characters from {len(pages)} pages")

    # Save or ingest content
    if args.output:
        with open(args.output, 'w', encoding='utf-8') as f:
            f.write(combined_content)
        logger.info(f"Content saved to: {args.output}")

    if args.ingest:
        logger.info("Ingesting content into RAG system...")
        ingestion_service = BookIngestionService()

        # Ingest the combined content
        result = await ingestion_service.ingest_from_text(
            text=combined_content,
            source_metadata={
                "url": args.url,
                "source_type": "web_scraped",
                "page_count": len(pages)
            }
        )

        logger.info(f"Ingestion result: {result}")

    # Print summary
    print(f"\nScraping Summary:")
    print(f"- Base URL: {args.url}")
    print(f"- Pages scraped: {len(pages)}")
    print(f"- Total content length: {len(combined_content)} characters")
    print(f"- Output: {'Saved to file' if args.output else 'Not saved'}")
    print(f"- Ingestion: {'Completed' if args.ingest else 'Skipped'}")


if __name__ == "__main__":
    asyncio.run(main())