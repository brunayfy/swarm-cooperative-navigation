class Floorplan(object):
    """
    The floorplan the DotBots move in.
    """

    def __init__(self, drawing):
        (self.width, self.height, self.obstacles) = self._parseDrawing(drawing)

    def getJSON(self):
        return {
            'width': self.width,
            'height': self.height,
            'obstacles': self.obstacles,
        }

    @staticmethod
    def _parseDrawing(drawing):
        lines = [line for line in drawing.splitlines() if line]
        width = max([len(line) for line in lines])
        height = len(lines)
        obstacles = []
        for (y, line) in enumerate(lines):
            for (x, c) in enumerate(line):
                if c == '#':
                    obstacles += [{'x': x, 'y': y, 'width': 1, 'height': 1}]
        return width, height, obstacles
