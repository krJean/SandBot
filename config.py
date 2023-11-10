from obstacles import Circle, Square

BOX_WIDTH = 10
BOX_LENGTH = 10

OBSTACLES = [
    Circle(center=(3.,4.), radius=1.),
    Square(center=(6.,6.), side_length=2.)
]

WAY_POINTS = [
    (1.,1.),
    (9.,9.)
]