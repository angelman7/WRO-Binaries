class Pillar:
    def __init__(self, name='red', hsv_bounds=None, min_area=100):
        if hsv_bounds is None:
            hsv_bounds = [((0, 0, 0), (179, 255, 255))]
        self.name = name
        self.hsv_bounds = hsv_bounds
        self.min_area = min_area

    def get_info(self):
        return f'Name: {self.name}, min area: {self.min_area}, hsv_bounds: {self.hsv_bounds}'

    def add_bounds(self, bounds):
        self.hsv_bounds.append(bounds)
