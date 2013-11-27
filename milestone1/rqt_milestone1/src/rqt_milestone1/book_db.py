#!/usr/bin/env python

import json
import os
from book import Book

class BookDB:

    instance = None

    class Singleton:

        book_codes = None
        book_dict = dict()
        dbFile = None
        filePath = (str(os.path.dirname(os.path.realpath(__file__))) + 
                "/book.db")

        def __init__(self):
            if self.book_codes == None:
                self.loadBookCodes()
                self.buildBookDict()

        def loadBookCodes(self):
            try:
                self.dbFile = open(self.filePath, 'r')
                content = self.dbFile.read()
                if content:
                    self.book_codes = json.loads(content)
                self.dbFile.close()
            except IOError:
                self.book_codes = {"books":{}}
        
        # Populates a dictionary, from title->Book(Object)
        def buildBookDict(self):
            for key in self.book_codes["books"].keys():
                book_data = self.book_codes["books"].get(key)
                book_object = Book(book_data)
                self.book_dict[key] = book_object

        def getAllBookCodes(self):
            return self.book_dict

        def getBookIdByTitle(self, book_title):
	    if book_title is None:
	        return None
	    else:
	        book_id = None
	        for key in self.book_dict.keys():
		    if self.book_dict.get(key).getTitle() == book_title:
		        book_id = key
	        return book_id

    def __init__(self):
        if BookDB.instance is None:
            BookDB.instance = BookDB.Singleton()

    # return a dict, from name to book_attributes
    def getAllBookCodes(self):
        return BookDB.instance.getAllBookCodes()

    # returns the book_id for the given title, None if does not exist
    def getBookIdByTitle(self, book_title):
	return BookDB.instance.getBookIdByTitle(book_title)
