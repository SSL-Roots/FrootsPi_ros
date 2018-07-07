#encoding: utf8

def add_hello(text):
    return text + 'hello'

def enclose_hello(text):
    return 'he' + text + 'llo'


class Calculator(object):
    def __init__(self):

        self._buffer = None

    def add(self, a, b):
        return a + b

    def set_value(self, value):
        self._buffer = value

    def get_value(self):
        return self._buffer

    def delete_value(self):
        self._init_buffer()
        return True

    def _init_buffer(self):
        self._buffer = None


