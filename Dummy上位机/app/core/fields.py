"""Small value objects for controller tests and UI adapters.

Qt widgets bind through the same get/set contract. No widget or Tk dependency
is needed to use robot operations in a test or another client.
"""


class ValueField:
    def __init__(self, value="", values=None):
        self.value = value
        self.options = list(values or [])
        self.index = 0
        self.properties = {}

    def get(self):
        return self.value

    def set(self, value):
        self.value = value

    def delete(self, *_):
        self.value = ""

    def insert(self, _index, value):
        self.value = str(value)

    def config(self, **kwargs):
        self.properties.update(kwargs)
        if "text" in kwargs:
            self.value = kwargs["text"]

    configure = config

    def current(self, index=None):
        if index is None:
            return self.index
        self.index = int(index)
        if self.options:
            self.value = self.options[self.index]

    def __setitem__(self, name, value):
        if name == "values":
            self.options = list(value)
        else:
            self.properties[name] = value


class ListField:
    def __init__(self):
        self.items = []
        self.selected = None

    def insert(self, index, text):
        if index == "end":
            self.items.append(text)
        else:
            self.items.insert(int(index), text)

    def delete(self, index, end=None):
        if end == "end":
            self.items.clear()
            self.selected = None
        else:
            del self.items[int(index)]

    def curselection(self):
        return () if self.selected is None else (self.selected,)

    def size(self):
        return len(self.items)
