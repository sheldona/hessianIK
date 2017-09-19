# Copyright (C) 2017    Kenny Erleben
#
# Permission to use and modify in any way, and for any purpose, this
# software, is granted by the author.  Permission to redistribute
# unmodified copies is also granted.  Modified copies may only be
# redistributed with the express written consent of:
#   Kenny Erleben (kenny@di.ku.dk)
#
import numpy as np
from math import cos, sin


def from_array(data):
    return np.array(data[0:4], dtype=np.float64)


def identity():
    return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


def Rx(radians):
    c = cos(radians/2.0)
    s = sin(radians/2.0)
    return from_array([c, s, 0.0, 0.0])


def Ry(radians):
    c = cos(radians/2.0)
    s = sin(radians/2.0)
    return from_array([c, 0.0, s, 0.0])


def Rz(radians):
    c = cos(radians/2.0)
    s = sin(radians/2.0)
    return from_array([c, 0.0, 0.0, s])


def conjugate(Q):
    return from_array([Q[0], -Q[1], -Q[2], -Q[3]])


def prod(Qa, Qb):
    a = Qa[0]
    A = Qa[1:]
    b = Qb[0]
    B = Qb[1:]
    qs = a * b - np.dot(A, B)
    qv = a * B + A * b + np.cross(A, B, axis=0)
    return from_array([qs, qv[0], qv[1], qv[2]])


def rotate(q, r):
    qr = from_array([0.0, r[0], r[1], r[2]])
    return prod(prod(q, qr), conjugate(q))[1:]
