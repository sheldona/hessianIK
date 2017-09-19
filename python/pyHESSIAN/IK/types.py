# Copyright (C) 2017    Kenny Erleben
#
# Permission to use and modify in any way, and for any purpose, this
# software, is granted by the author.  Permission to redistribute
# unmodified copies is also granted.  Modified copies may only be
# redistributed with the express written consent of:
#   Kenny Erleben (kenny@di.ku.dk)
#
import Math.quaternion as Q
import Math.vector3 as V3


class Bone:

    def __init__(self):
        self.idx = 0                # Bone index
        self.euler_code = 'ZYZ'     # Euler angle convention
        self.alpha = 0.0            # Euler angles of bone
        self.beta = 0.0
        self.gamma = 0.0
        self.q_wcs = Q.identity()   # Joint frame orientation in WCS
        self.t_wcs = V3.zero()      # Joint origin in WCS
        self.t = V3.zero()          # Joint vector in parent frame
        self.parent = -1            # Index to parent bone or -1 if bone is root
        self.children = []          # Indices to children bones

    def is_root(self):
        if self.parent == -1:
            return True
        return False

    def is_end_effector(self):
        if len(self.children) == 0:
            return True
        return False

    def has_parent(self):
        return not self.is_root()

    def get_axis(self, idx):
        if self.euler_code[idx] == 'X':
            return V3.i()
        if self.euler_code[idx] == 'Y':
            return V3.j()
        if self.euler_code[idx] == 'Z':
            return V3.k()

    def get_rotation(self, idx, radians):
        if self.euler_code[idx] == 'X':
            return Q.Rx(radians)
        if self.euler_code[idx] == 'Y':
            return Q.Ry(radians)
        if self.euler_code[idx] == 'Z':
            return Q.Rz(radians)

    def get_axis_alpha(self):
        return self.get_axis(0)

    def get_axis_beta(self):
        return self.get_axis(1)

    def get_axis_gamma(self):
        return self.get_axis(2)

    def get_rotation_alpha(self):
        return self.get_rotation(0, self.alpha)

    def get_rotation_beta(self):
        return self.get_rotation(1, self.beta)

    def get_rotation_gamma(self):
        return self.get_rotation(2, self.gamma)


class Skeleton:

    def __init__(self):
        self.bones = []       # Bones of the skeleton.

    def has_root(self):
        if len(self.bones) > 0:
            return True
        return False

    def has_bone(self, idx):
        if len(self.bones) > idx >= 0:
            return True
        return False


class Chain:

    def __init__(self):
        self.bones = []                # Indices of all bones that are part of the chain.
        self.skeleton = None           # Reference to skeleton holding the bones
        self.goal = V3.make(10, 0, 0)  # A default goal position in world coordinates
        self.tool = V3.zero()          # A tool vector in end-effector coordinates

