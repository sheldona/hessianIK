# Copyright (C) 2017    Kenny Erleben
#
# Permission to use and modify in any way, and for any purpose, this
# software, is granted by the author.  Permission to redistribute
# unmodified copies is also granted.  Modified copies may only be
# redistributed with the express written consent of:
#   Kenny Erleben (kenny@di.ku.dk)
#

import IK.api as IK
import Math.vector3 as V3
import numpy as np


def test_update_skeleton():
    skeleton = IK.create_skeleton()
    B0 = IK.create_root(skeleton, alpha=IK.degrees_to_radians(90), beta=0.0, gamma=0.0, tx=1.0, ty=0.0, tz=0.0)
    B1 = IK.add_bone(skeleton, parent_idx=B0.idx, alpha=IK.degrees_to_radians(90), beta=0.0, gamma=0.0, tx=1.0, ty=0.0, tz=0.0)
    B2 = IK.add_bone(skeleton, parent_idx=B1.idx, alpha=IK.degrees_to_radians(-90), beta=0.0, gamma=0.0, tx=1.0, ty=0.0, tz=0.0)
    B3 = IK.add_bone(skeleton, parent_idx=B0.idx, alpha=IK.degrees_to_radians(-90), beta=0.0, gamma=0.0, tx=-1.0, ty=0.0, tz=0.0)
    IK.update_skeleton(skeleton)
    if V3.norm(B0.t_wcs - V3.make(1.0, 0.0, 0.0)) < 0.001:
        print('success')
    else:
        print('failure')
    if V3.norm(B1.t_wcs - V3.make(1.0, 1.0, 0.0)) < 0.001:
        print('success')
    else:
        print('failure')
    if V3.norm(B2.t_wcs - V3.make(0.0, 1.0, 0.0)) < 0.001:
        print('success')
    else:
        print('failure')
    if V3.norm(B3.t_wcs - V3.make(1.0, -1.0, 0.0)) < 0.001:
        print('success')
    else:
        print('failure')
    IK.print_skeleton(skeleton)

    skeleton = IK.create_skeleton()
    B0 = IK.create_root(skeleton, alpha=IK.degrees_to_radians(0), beta=0.0, gamma=0.0, tx=1.0, ty=0.0, tz=0.0)
    B1 = IK.add_bone(skeleton, parent_idx=B0.idx, alpha=IK.degrees_to_radians(0), beta=0.0, gamma=0.0, tx=1.0, ty=0.0, tz=0.0)

    IK.update_skeleton(skeleton)
    if V3.norm(B0.t_wcs - V3.make(1.0, 0.0, 0.0)) < 0.001:
        print('success')
    else:
        print('failure')

    if V3.norm(B1.t_wcs - V3.make(2.0, 0.0, 0.0)) < 0.001:
        print('success')
    else:
        print('failure')

    B0.alpha = IK.degrees_to_radians(90)
    IK.update_skeleton(skeleton)
    if V3.norm(B0.t_wcs - V3.make(1.0, 0.0, 0.0)) < 0.001:
        print('success')
    else:
        print('failure')

    if V3.norm(B1.t_wcs - V3.make(1.0, 1.0, 0.0)) < 0.001:
        print('success')
    else:
        print('failure')


def test_make_chains():
    skeleton = IK.create_skeleton()
    B0 = IK.create_root(skeleton, alpha=IK.degrees_to_radians(0), beta=0.0, gamma=0.0, tx=1.0, ty=0.0, tz=0.0)
    B1 = IK.add_bone(skeleton, parent_idx=B0.idx, alpha=IK.degrees_to_radians(0), beta=0.0, gamma=0.0, tx=1.0, ty=0.0, tz=0.0)
    chains = IK.make_chains(skeleton)
    if len(chains) == 1:
        print('success')
    else:
        print('failure')
    if len(chains[0].bones) == 2:
        print('success')
    else:
        print('failure')
    if chains[0].bones[0] == 0:
        print('success')
    else:
        print('failure')
    if chains[0].bones[1] == 1:
        print('success')
    else:
        print('failure')
    IK.print_chains(chains)

    skeleton = IK.create_skeleton()
    B0 = IK.create_root(skeleton, alpha=IK.degrees_to_radians(0), beta=0.0, gamma=0.0, tx=1.0, ty=0.0, tz=0.0)
    B1 = IK.add_bone(skeleton, parent_idx=B0.idx, alpha=IK.degrees_to_radians(0), beta=0.0, gamma=0.0, tx=1.0, ty=0.0, tz=0.0)
    B2 = IK.add_bone(skeleton, parent_idx=B0.idx, alpha=IK.degrees_to_radians(90), beta=0.0, gamma=0.0, tx=1.0, ty=0.0, tz=0.0)
    B3 = IK.add_bone(skeleton, parent_idx=B2.idx, alpha=IK.degrees_to_radians(-90), beta=0.0, gamma=0.0, tx=1.0, ty=0.0, tz=0.0)

    chains = IK.make_chains(skeleton)
    if len(chains) == 2:
        print('success')
    else:
        print('failure')
    if len(chains[0].bones) == 2:
        print('success')
    else:
        print('failure')
    if chains[0].bones[0] == 0:
        print('success')
    else:
        print('failure')
    if chains[0].bones[1] == 1:
        print('success')
    else:
        print('failure')

    if len(chains[1].bones) == 3:
        print('success')
    else:
        print('failure')
    if chains[1].bones[0] == 0:
        print('success')
    else:
        print('failure')
    if chains[1].bones[1] == 2:
        print('success')
    else:
        print('failure')
    if chains[1].bones[2] == 3:
        print('success')
    else:
        print('failure')
    IK.print_chains(chains)


def test_angles():
    skeleton = IK.create_skeleton()
    B0 = IK.create_root(skeleton, alpha=IK.degrees_to_radians(90), beta=0.0, gamma=0.0, tx=1.0, ty=0.0, tz=0.0)
    B1 = IK.add_bone(skeleton, parent_idx=B0.idx, alpha=IK.degrees_to_radians(-90), beta=0.0, gamma=0.0, tx=1.0, ty=0.0, tz=0.0)
    B2 = IK.add_bone(skeleton, parent_idx=B1.idx, alpha=IK.degrees_to_radians(90), beta=0.0, gamma=0.0, tx=1.0, ty=0.0, tz=0.0)
    B3 = IK.add_bone(skeleton, parent_idx=B0.idx, alpha=IK.degrees_to_radians(-90), beta=0.0, gamma=0.0, tx=1.0, ty=0.0, tz=0.0)

    angles = IK.get_joint_angles(skeleton)

    if len(angles) == 12:
        print('success')
    else:
        print('failure')

    if abs(angles[0] - IK.degrees_to_radians(90)) < 0.001:
        print('success')
    else:
        print('failure')
    if abs(angles[1] - IK.degrees_to_radians(0)) < 0.001:
        print('success')
    else:
        print('failure')
    if abs(angles[2] - IK.degrees_to_radians(0)) < 0.001:
        print('success')
    else:
        print('failure')

    if abs(angles[3] - IK.degrees_to_radians(-90)) < 0.001:
        print('success')
    else:
        print('failure')
    if abs(angles[4] - IK.degrees_to_radians(0)) < 0.001:
        print('success')
    else:
        print('failure')
    if abs(angles[5] - IK.degrees_to_radians(0)) < 0.001:
        print('success')
    else:
        print('failure')

    angles = [ n for n in range(0,12) ]
    IK.set_joint_angles(skeleton, angles)
    if B3.alpha == 9:
        print('success')
    else:
        print('failure')
    if B3.beta == 10:
        print('success')
    else:
        print('failure')
    if B3.gamma == 11:
        print('success')
    else:
        print('failure')


def test_jacobian():
    skeleton = IK.create_skeleton()
    B0 = IK.create_root(skeleton, alpha=IK.degrees_to_radians(0), beta=0.0, gamma=0.0, tx=1.0, ty=0.0, tz=0.0)
    B1 = IK.add_bone(skeleton, parent_idx=B0.idx, alpha=IK.degrees_to_radians(0), beta=0.0, gamma=0.0, tx=1.0, ty=0.0, tz=0.0)
    B2 = IK.add_bone(skeleton, parent_idx=B1.idx, alpha=IK.degrees_to_radians(0), beta=0.0, gamma=0.0, tx=1.0, ty=0.0, tz=0.0)
    IK.update_skeleton(skeleton)
    chains = IK.make_chains(skeleton)
    Jtst = IK.compute_jacobian(chains, skeleton)
    J = np.array(
        [[0., 0., 0., 0., 0., 0., 0., 0., 0.],
         [2., 0., 2., 1., 0., 1., 0., 0., 0.],
         [0., -2., 0., 0., -1., 0., 0., 0., 0.]]
    )
    if (np.max(np.abs(J-Jtst))) > 0.0:
        print('failure')
    else:
        print('success')


def test_hessian():
    skeleton = IK.create_skeleton()
    B0 = IK.create_root(skeleton, alpha=IK.degrees_to_radians(0), beta=0.0, gamma=0.0, tx=1.0, ty=0.0, tz=0.0)
    B1 = IK.add_bone(skeleton, parent_idx=B0.idx, alpha=IK.degrees_to_radians(0), beta=0.0, gamma=0.0, tx=1.0, ty=0.0, tz=0.0)
    B2 = IK.add_bone(skeleton, parent_idx=B1.idx, alpha=IK.degrees_to_radians(0), beta=0.0, gamma=0.0, tx=1.0, ty=0.0, tz=0.0)
    B3 = IK.add_bone(skeleton, parent_idx=B2.idx, alpha=IK.degrees_to_radians(0), beta=0.0, gamma=0.0, tx=1.0, ty=0.0, tz=0.0)
    IK.update_skeleton(skeleton)
    chains = IK.make_chains(skeleton)
    chains[0].goal = V3.make(10, 0, 0)

    J = IK.compute_jacobian(chains, skeleton)
    H_tst = IK.compute_hessian(chains, skeleton, J)
    print(H_tst)

    H_app = IK.finite_difference_hessian(chains, skeleton, 0.00001)
    print(H_app)


def test_gradient():
    skeleton = IK.create_skeleton()
    B0 = IK.create_root(skeleton, alpha=IK.degrees_to_radians(0), beta=0.0, gamma=0.0, tx=1.0, ty=0.0, tz=0.0)
    B1 = IK.add_bone(skeleton, parent_idx=B0.idx, alpha=IK.degrees_to_radians(0), beta=0.0, gamma=0.0, tx=1.0, ty=0.0, tz=0.0)
    B2 = IK.add_bone(skeleton, parent_idx=B1.idx, alpha=IK.degrees_to_radians(0), beta=0.0, gamma=0.0, tx=1.0, ty=0.0, tz=0.0)
    B3 = IK.add_bone(skeleton, parent_idx=B2.idx, alpha=IK.degrees_to_radians(0), beta=0.0, gamma=0.0, tx=1.0, ty=0.0, tz=0.0)
    IK.update_skeleton(skeleton)
    chains = IK.make_chains(skeleton)
    chains[0].goal = V3.make(10, 0, 0)

    J = IK.compute_jacobian(chains, skeleton)
    g = IK.compute_gradient(chains, skeleton, J)
    print(g)

    g_app = IK.finite_difference_gradient(chains, skeleton, h=0.00001)
    print(g_app)


if __name__ == '__main__':
    test_update_skeleton()
    test_make_chains()
    test_angles()
    test_jacobian()
    test_gradient()
    test_hessian()
