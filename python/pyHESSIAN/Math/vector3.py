# Copyright (C) 2017    Kenny Erleben
#
# Permission to use and modify in any way, and for any purpose, this
# software, is granted by the author.  Permission to redistribute
# unmodified copies is also granted.  Modified copies may only be
# redistributed with the express written consent of:
#   Kenny Erleben (kenny@di.ku.dk)
#
import numpy as np


def zero():
    return np.zeros((3, ), dtype=np.float64)


def make(x, y, z):
    return np.array([x, y, z], dtype=np.float64)


def i():
    return np.array([1.0, 0.0, 0.0], dtype=np.float64)


def j():
    return np.array([0.0, 1.0, 0.0], dtype=np.float64)


def k():
    return np.array([0.0, 0.0, 1.0], dtype=np.float64)


def cross(a, b):
    return np.cross(a, b, axis=0)


def norm(a):
    return np.linalg.norm(a)
