from enum import Enum
import numpy as np


class Instruction(Enum):
    STRAIGHT = "straight"
    LEFT_TURN = "left"
    RIGHT_TURN = "right"
    U_TURN = "u_turn"
    STOP = "stop"

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return :wq!self.value == other.value
        return False


class Direction(Enum):
    NORTH = "negative_x"
    SOUTH = "positive_x"
    EAST = "positive_y"
    WEST = "negative_y"


class Grid:
    def __init__(self, n, m, direction=Direction.SOUTH):
        self.n = n
        self.m = m
        self.grid = np.full((n, m, 3), False)
        self.c_x = -1
        self.c_y = 0
        self.c_dir = direction
        self.stack = []

    def reset_grid(self):
        self.grid = np.full((self.n, self.m, 3), False)
        self.stack = []

    def __mark_node(self, is_obstacle):
        if self.c_dir == Direction.SOUTH:
            self.c_x += 1
        elif self.c_dir == Direction.NORTH:
            self.c_x -= 1
        elif self.c_dir == Direction.EAST:
            self.c_y += 1
        else:
            self.c_y -= 1

        self.stack.append((self.c_x, self.c_y))
        self.grid[self.c_x][self.c_y][1] = True
        self.grid[self.c_x][self.c_y][0] = is_obstacle

    def __turn180(self):
        if self.c_dir == Direction.SOUTH:
            self.c_dir = Direction.NORTH

        elif self.c_dir == Direction.NORTH:
            self.c_dir = Direction.SOUTH

        elif self.c_dir == Direction.EAST:
            self.c_dir = Direction.WEST

        else:
            self.c_dir = Direction.EAST

    def __go_straight(self):
        return self.c_dir == Direction.NORTH \
               and self.c_x > 0 \
               and not self.grid[self.c_x - 1][self.c_y][0] \
               and not self.grid[self.c_x - 1][self.c_y][1] \
               or \
               self.c_dir == Direction.SOUTH \
               and self.c_x < self.n - 1 \
               and not self.grid[self.c_x + 1][self.c_y][0] \
               and not self.grid[self.c_x + 1][self.c_y][1] \
               or \
               self.c_dir == Direction.EAST \
               and self.c_y < self.m - 1 \
               and not self.grid[self.c_x][self.c_y + 1][0] \
               and not self.grid[self.c_x][self.c_y + 1][1] \
               or \
               self.c_dir == Direction.WEST \
               and self.c_y > 0 \
               and not self.grid[self.c_x][self.c_y - 1][0] \
               and not self.grid[self.c_x][self.c_y - 1][1]

    def __go_left(self):
        if self.c_dir == Direction.NORTH and self.c_y > 0 \
                and not self.grid[self.c_x][self.c_y - 1][0] \
                and not self.grid[self.c_x][self.c_y - 1][1]:
            self.c_dir = Direction.WEST
            return True

        if self.c_dir == Direction.SOUTH and self.c_y < self.m - 1 \
                and not self.grid[self.c_x][self.c_y + 1][0] \
                and not self.grid[self.c_x][self.c_y + 1][1]:
            self.c_dir = Direction.EAST
            return True

        if self.c_dir == Direction.EAST and self.c_x > 0 \
                and not self.grid[self.c_x - 1][self.c_y][0] \
                and not self.grid[self.c_x - 1][self.c_y][1]:
            self.c_dir = Direction.NORTH
            return True

        if self.c_dir == Direction.WEST and self.c_x < self.n - 1 \
                and not self.grid[self.c_x + 1][self.c_y][0] \
                and not self.grid[self.c_x + 1][self.c_y][1]:
            self.c_dir = Direction.SOUTH
            return True

        return False

    def __go_right(self):
        if self.c_dir == Direction.NORTH and self.c_y < self.m - 1 \
                and not self.grid[self.c_x][self.c_y + 1][0] \
                and not self.grid[self.c_x][self.c_y + 1][1]:
            self.c_dir = Direction.EAST
            return True

        if self.c_dir == Direction.SOUTH and self.c_y > 0 \
                and not self.grid[self.c_x][self.c_y - 1][0] \
                and not self.grid[self.c_x][self.c_y - 1][1]:
            self.c_dir = Direction.WEST
            return True

        if self.c_dir == Direction.EAST and self.c_x < self.n - 1 \
                and not self.grid[self.c_x + 1][self.c_y][0] \
                and not self.grid[self.c_x + 1][self.c_y][1]:
            self.c_dir = Direction.SOUTH
            return True

        if self.c_dir == Direction.WEST and self.c_x > 0 \
                and not self.grid[self.c_x - 1][self.c_y][0] \
                and not self.grid[self.c_x - 1][self.c_y][1]:
            self.c_dir = Direction.NORTH
            return True
        return False

    def get_navigating_instruction_a_2_b(self, x, y, to_go_x, to_go_y):
        if self.c_dir == Direction.NORTH:
            if to_go_x > x:
                self.c_dir = Direction.SOUTH
                return [Instruction.U_TURN]
            if to_go_x < x:
                self.c_dir = Direction.NORTH
                return [Instruction.STRAIGHT]
            if to_go_y < y:
                self.c_dir = Direction.WEST
                return [Instruction.LEFT_TURN]
            if to_go_y > y:
                self.c_dir = Direction.EAST
                return [Instruction.RIGHT_TURN]

        if self.c_dir == Direction.SOUTH:
            if to_go_x > x:
                self.c_dir = Direction.SOUTH
                return [Instruction.STRAIGHT]
            if to_go_x < x:
                self.c_dir = Direction.NORTH
                return [Instruction.U_TURN]
            if to_go_y < y:
                self.c_dir = Direction.WEST
                return [Instruction.RIGHT_TURN]
            if to_go_y > y:
                self.c_dir = Direction.EAST
                return [Instruction.LEFT_TURN]

        if self.c_dir == Direction.EAST:
            if to_go_x > x:
                self.c_dir = Direction.SOUTH
                return [Instruction.RIGHT_TURN]
            if to_go_x < x:
                self.c_dir = Direction.NORTH
                return [Instruction.LEFT_TURN]
            if to_go_y < y:
                self.c_dir = Direction.WEST
                return [Instruction.U_TURN]
            if to_go_y > y:
                self.c_dir = Direction.EAST
                return [Instruction.STRAIGHT]

        if self.c_dir == Direction.WEST:
            if to_go_x > x:
                self.c_dir = Direction.SOUTH
                return [Instruction.LEFT_TURN]
            if to_go_x < x:
                self.c_dir = Direction.NORTH
                return [Instruction.RIGHT_TURN]
            if to_go_y < y:
                self.c_dir = Direction.WEST
                return [Instruction.STRAIGHT]
            if to_go_y > y:
                self.c_dir = Direction.EAST
                return [Instruction.U_TURN]

    def get_next_instruction(self, is_obstacle):
        self.__mark_node(is_obstacle)

        if is_obstacle:
            self.stack.pop()
            self.stack.pop()
            self.__turn180()
            return [Instruction.U_TURN]

        if self.__go_straight():
            return [Instruction.STRAIGHT]

        if self.__go_left():
            return [Instruction.LEFT_TURN]

        if self.__go_right():
            return [Instruction.RIGHT_TURN]

        # If I reach here i have no where to go so i return
        x, y = self.stack.pop()
        if len(self.stack) == 0:
            return Instruction.STOP
        to_go_x, to_go_y = self.stack.pop()

        return self.get_navigating_instruction_a_2_b(x, y, to_go_x, to_go_y)


if __name__ == "__main__":
    grid = Grid(5, 5)
    grid.grid = np.full((5, 5, 3), False)

    while True:
        ins = grid.get_next_instruction(False)
        print(ins)
        if Instruction.STOP == ins:
            break
