class Point:
    def __init__(self,x,y):
        self.x = x
        self.y = y
    def __repr__(self):
        return f"Point: ({self.x}, {self.y})"
    def __getitem__(self, key):
        assert key == 0 or key == 1
        if key == 0:
            return self.x
        elif key == 1:
            return self.y
        