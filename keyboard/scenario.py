import csv

class AxesPos:
    """
    Encapsulates information about position of multiple axes and
    an optional comment describing that position
    """

    def __init__(self, pos, nline=None, comment=None):
        self.nline = nline
        self.pos = list(pos)
        self.comment = comment
    
    def clone(self):
        return AxesPos(self.pos)

    def apply_delta(self, axis, delta):
        new_pos = self.pos[axis] + delta
        if new_pos >= -45 and new_pos <= 45:
            self.pos[axis] = new_pos
        else:
            # limit exceeded
            print('\a')

    def write(self, writer):
        writer.writerow(self.pos)

    def __str__(self):
        """
        Returns a string specifying coordinates for all axes
        """
        return str(self.pos)

class Scenario:
    """
    Encapsulates information about sequence of positions we want the manipulator to move between
    """

    def __init__(self, file):
        self.file = file
        self.load()

    def load(self):
        self.positions = []
        with open(self.file, 'rt') as f:
            data = csv.reader(f)
            for row in data:
                row = [int(i) for i in row]
                self.add_pos(AxesPos(row))

    def get_next_nline(self):
        return len(self.positions)

    def add_pos(self, new_pos):
        new_pos.nline = self.get_next_nline()
        self.positions.append(new_pos)

    def get_pos(self, nline):
        return self.positions[nline]

    def get_first_pos(self):
        return self.positions[0]

    def get_last_pos(self):
        return self.positions[-1]

    def print(self):
        print("=== %s ===" % self.file)
        for i in range(len(self.positions)):
            print("%3d: %s" % (i, self.positions[i].pos))
        print("=== END ===")
    
    def save(self, file=None):
        if file is None:
            file = self.file

        with open(file, mode='w', newline='') as f:
            writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            for pos in self.positions:
                pos.write(writer)
        
        print("# scenario saved in %s" % file)

TEST_SCENARIO1 = 'scenario-test1.csv'
TEST_SCENARIO2 = 'scenario-test2.csv'

if __name__ == "__main__":
    s1 = Scenario(TEST_SCENARIO1)
    s1.print()
    s1.get_pos(1).apply_delta(1, 1)
    s1.save(TEST_SCENARIO2)

    s2 = Scenario(TEST_SCENARIO2)
    s1.print()
