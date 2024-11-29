import csv


class Csvaction:
    def __init__(self, csv_filename):
        self.csv_filename = csv_filename
        self.fp = None
        self.writer = None
        self.reader = None

    def open_writer(self):
        self.fp = open(self.csv_filename, "w")
        self.writer = csv.writer(self.fp, delimiter=",")
        self.writer.writerow("header")

    def open_reader(self):
        self.fp = open(self.csv_filename, "r")
        self.reader = csv.reader(self.fp)
        lines = len(list(self.reader)) - 1
        self.fp.close()
        self.fp = open(self.csv_filename, "r")
        self.reader = csv.reader(self.fp)
        next(self.reader)
        return lines

    def close(self):
        self.fp.close()
