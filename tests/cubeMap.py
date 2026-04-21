import numpy as np

class CubeMap:
    def __init__(self):
        self.map = np.zeros((6,1), dtype=int)

    #Initialixe map with known map
    def initialize_map(self, map):
        self.map = map
    
    #Identify the position of the cube base on the top face and one side face, specify if the side face is clockwise or counterclockwise from the top face
    def identify_dice_location(self, top, side = 0, clockwise = False):
        #clear map before filling in new values
        self.map = np.zeros((6,1), dtype=int)
        
        self.map[0] = top
        if clockwise:
            self.map[1] = side
        else:
            self.map[3] = side
        
        self.infer_missing_sides()

    #Claude function: Fills the remaining unknown faces of the cube map based on the known faces and standard dice orientation.
    def infer_missing_sides(self):
        copy = self.map.copy()
        # Opposite face pairs: (top/bottom), (front/back), (right/left)
        opposite_pairs = [(0, 2), (1, 3), (4, 5)]

        # Fill in any known opposites (sum to 7)
        for a, b in opposite_pairs:
            if copy[a] == 0 and copy[b] != 0:
                copy[a] = 7 - copy[b]
            elif copy[b] == 0 and copy[a] != 0:
                copy[b] = 7 - copy[a]

        # If left/right axis is still unknown, use standard dice chirality.
        # On a standard (right-handed) die: 1 top, 2 front → 3 is right.
        # All faces follow a fixed cyclic order around each axis.
        if copy[4] == 0 or copy[5] == 0:
            top, front = copy[0], copy[1]
            # Standard dice: right-hand rule cycles for each top/front combo
            # Build all 24 orientations implicitly via the known chirality table.
            # Each row: [top, front, right]
            chirality_table = [
                (1,2,3),(1,3,5),(1,5,4),(1,4,2),
                (2,1,4),(2,4,6),(2,6,3),(2,3,1),
                (3,1,2),(3,2,6),(3,6,5),(3,5,1),
                (4,1,5),(4,5,6),(4,6,2),(4,2,1),
                (5,1,3),(5,3,6),(5,6,4),(5,4,1),
                (6,2,4),(6,4,5),(6,5,3),(6,3,2),
            ]
            for t, f, r in chirality_table:
                if t == top and f == front:
                    copy[4] = r
                    copy[5] = 7 - r
                    break

        self.map = copy

    #Rotate the internal cube on the X axis by 90 degrees, default counterclockwise
    def rotate_cube_x_axis(self,clockwise=False):
        copy = self.map.copy()
        if clockwise:
            copy[0] = self.map[3]
            copy[3] = self.map[2]
            copy[2] = self.map[1]
            copy[1] = self.map[0]
        else:
            copy[0] = self.map[1]
            copy[1] = self.map[2]
            copy[2] = self.map[3]
            copy[3] = self.map[0]
        self.map = copy

    #Rotate the internal cube on the Y axis by 90 degrees, default clockwise
    def rotate_cube_z_axis(self, clockwise=True):
        copy = self.map.copy()
        if clockwise:
            copy[1] = self.map[4]
            copy[4] = self.map[3]
            copy[3] = self.map[5]
            copy[5] = self.map[1]
        else:
            copy[1] = self.map[5]
            copy[5] = self.map[3]
            copy[3] = self.map[4]
            copy[4] = self.map[1]
        self.map = copy

    #Find the sequence of rotations needed to get to a specific face, specify preference rotation on x axis clockwise or counterclockise
    #Returns sequence of moves. 0 = rotate z clockwise, 1 = rotate z counterclockwise, 2 = rotate x clockwise, 3 = rotate x counterclockwise
    def find_face_with_pip(self, pip, prefer_clockwise=False):
        move_sequence = []
        #Check if pip is on annoying sides first
        if self.map[4] == pip:
            #Rotate clockwise Z axis
            self.rotate_cube_z_axis(clockwise=not prefer_clockwise)
            move_sequence.append(1 if prefer_clockwise else 0)
        elif self.map[5] == pip:
            #roate counterclockwise Z axis
            self.rotate_cube_z_axis(clockwise=prefer_clockwise)
            move_sequence.append(0 if prefer_clockwise else 1)

        #Check if pip is on top
        if self.map[0] == pip:
            return move_sequence

        #check if pip is on the back, if so rotate 180 degrees on Z axis
        if self.map[3] == pip and not prefer_clockwise:
            self.rotate_cube_z_axis(clockwise=True)
            self.rotate_cube_z_axis(clockwise=True)
            move_sequence.extend([0,0])
        elif self.map[1] == pip and prefer_clockwise:
            self.rotate_cube_z_axis(clockwise=True)
            self.rotate_cube_z_axis(clockwise=True)
            move_sequence.extend([0,0])

        #Rotate on X axis until pip is on top, prefer clockwise for right robot and counterclockwise for left robot
        for i in range(4):
            if self.map[0] == pip:
                return move_sequence
            if prefer_clockwise:
                self.rotate_cube_x_axis(clockwise=True)
                move_sequence.append(2)
            else:
                self.rotate_cube_x_axis(clockwise=False)
                move_sequence.append(3)

    #Print the dice in a pretty way
    def print_cube(self):
        print(f"    {self.map[4]}")
        print(f"{self.map[3]} {self.map[0]} {self.map[1]} {self.map[2]}")
        print(f"    {self.map[5]}")

if __name__ == "__main__":
    cube = CubeMap()
    cube.identify_dice_location(top=4, side=1, clockwise=False)
    cube.print_cube()
    print("0 = rotate z clockwise, 1 = rotate z counterclockwise, 2 = rotate x clockwise, 3 = rotate x counterclockwise")
    print("How to find 3:", cube.find_face_with_pip(3, prefer_clockwise=False))
