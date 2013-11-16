#!/usr/bin/env python

from book_db import BookDB
from book import Book

db = BookDB()
codes = db.getAllBookCodes()

for book in codes.keys():
    print codes.get(book).getTitle()
    print codes.get(book).getGenre()
    print codes.get(book).getYear()
    print codes.get(book).getSummary()
    print
