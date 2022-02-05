class Pillar:
    def __init__(self, name, lowers=[], uppers=[], min_area=400) -> None:
        self.name = name
        self.lowers = lowers
        self.uppers = uppers
        self.min_area = min_area
    
    def get_info(self):
        return f'Name: {self.name}, Min-Area: {self.min_area}'
    
    def add_lower(self, lower):
        self.lowers.append(lower)

    def add_upper(self, upper):
        self.lowers.append(upper)