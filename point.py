

class Point:

    goal = None

    def __init__(self, x=0, y=0, priority=100) -> None:
        self.x = x 
        self.y = y
        if priority == 100:
            self.priority = self.dist_to(Point.goal)
        else: 
            self.priority = 0
        
    def is_empty(self, image):
        return image[self.x][self.y] != 0
    
    def dist_to(self, p):
        return ((self.x-p.x)**2 + (self.y - p.y)**2)**0.5
    
    def __eq__(self, __value: object) -> bool:
        if __value is None:
            return False
        return self.x == __value.x and self.y == __value.y
    # def eq(self, p):
    #     return self.x == p.x and self.y == p.y

    def __str__(self) -> str:
        return f"{self.x}, {self.y}"