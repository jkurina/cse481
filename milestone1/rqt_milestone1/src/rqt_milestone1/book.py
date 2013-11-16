#!/usr/bin/env python

class Book:

    title = None
    year = None
    genre  = None
    summary = None

    def __init__(self, book_data):
        self.title = book_data["title"]
        self.year = book_data["year"]
        self.genre = book_data["genre"]
        self.summary = book_data["summary"]

    def getTitle(self):
        return self.title

    def getYear(self):
        return self.year
        
    def getGenre(self):
        return self.genre

    def getSummary(self):
        return self.summary
