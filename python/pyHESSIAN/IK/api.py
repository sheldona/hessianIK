# Copyright (C) 2017    Kenny Erleben
#
# Permission to use and modify in any way, and for any purpose, this
# software, is granted by the author.  Permission to redistribute
# unmodified copies is also granted.  Modified copies may only be
# redistributed with the express written consent of:
#   Kenny Erleben (kenny@di.ku.dk)
#
import numpy as np
import Math.vector3 as V3
import Math.quaternion as Q
from IK.types import *


def is_valid_euler_code(euler_code):

    proper_euler_anlges = ['XZX','XYX','YXY','YZY','ZYZ','ZXZ']
    tait_bryan_angles = ['XZY','XYZ','YXZ','YZX','ZYX','ZXY']

    if euler_code in proper_euler_anlges:
        return True
    if euler_code in tait_bryan_angles:
        return True
    return False


def radians_to_degrees(radians):
    return 180.0*radians/np.pi


def degrees_to_radians(degrees):
    return degrees*np.pi/180.0


def create_skeleton():
    S = Skeleton()
    return S


def create_root(skeleton, alpha, beta, gamma, tx, ty, tz, euler_code='ZYZ'):
    if skeleton.has_root():
        raise RuntimeError('create_root(): Internal error, root already existed')
    if not is_valid_euler_code(euler_code):
        raise RuntimeError('create_root(): Internal error, invalid Euler angle code given')
    root = Bone()
    root.t = V3.make(tx, ty, tz)
    root.alpha = alpha
    root.beta = beta
    root.gamma = gamma
    root.euler_code = euler_code
    skeleton.bones.append(root)
    return root


def add_bone(skeleton, parent_idx, alpha, beta, gamma, tx, ty, tz, euler_code='ZYZ'):
    if not skeleton.has_bone(parent_idx):
        raise RuntimeError('add_bone(): Internal error, parent did not exist')
    if not is_valid_euler_code(euler_code):
        raise RuntimeError('create_root(): Internal error, invalid Euler angle code given')

    end_effector = Bone()
    end_effector.idx = len(skeleton.bones)
    end_effector.parent = parent_idx
    end_effector.t = V3.make(tx, ty, tz)
    end_effector.alpha = alpha
    end_effector.beta = beta
    end_effector.gamma = gamma
    end_effector.euler_code = euler_code

    skeleton.bones[parent_idx].children.append(end_effector.idx)
    skeleton.bones.append(end_effector)
    return end_effector


def __print_bone(indent, bone, skeleton):
    print(indent,
          'Bone idx :', bone.idx,
          'Parent idx :', bone.parent,
          'Euler angles :', V3.make(bone.alpha, bone.beta, bone.gamma),
          'Euler code :', bone.euler_code,
          ' Origin :', bone.t_wcs
          )
    for idx in bone.children:
        __print_bone(indent + '\t',  skeleton.bones[idx], skeleton)


def print_skeleton(skeleton):
    print('skeleton:')
    __print_bone('\t', skeleton.bones[0], skeleton)


def __update_bone(bone, skeleton):

    q_alpha = bone.get_rotation_alpha()
    q_beta = bone.get_rotation_beta()
    q_gamma = bone.get_rotation_gamma()
    q_bone = Q.prod(q_alpha, Q.prod(q_beta, q_gamma))

    t_parent = V3.zero()
    q_parent = Q.identity()
    if bone.has_parent():
        t_parent = skeleton.bones[bone.parent].t_wcs
        q_parent = skeleton.bones[bone.parent].q_wcs
    bone.t_wcs = t_parent + Q.rotate(q_parent, bone.t)
    bone.q_wcs = Q.prod(q_parent, q_bone)

    for idx in bone.children:
        __update_bone(skeleton.bones[idx], skeleton)


def update_skeleton(skeleton):
    __update_bone(skeleton.bones[0], skeleton)


def __make_chain(end_effector, skeleton):
    chain = Chain()
    chain.skeleton = skeleton
    bone = end_effector
    while not bone.is_root():
        chain.bones.append(bone.idx)
        bone = skeleton.bones[bone.parent]
    chain.bones.append(bone.idx)
    chain.bones.reverse()
    return chain


def make_chains(skeleton):
    chains = []
    for bone in skeleton.bones:
        if bone.is_end_effector():
            chain = __make_chain(bone, skeleton)
            chains.append(chain)
    return chains


def print_chains(chains):
    for chain in chains:
        print('chain:')
        for idx in chain.bones:
            print('\t', idx)
        print()


def get_joint_angles(skeleton):
    angles = np.zeros((len(skeleton.bones)*3, ), dtype=np.float64)
    for bone in skeleton.bones:
        angles[bone.idx*3 + 0] = bone.alpha
        angles[bone.idx*3 + 1] = bone.beta
        angles[bone.idx*3 + 2] = bone.gamma
    return angles


def set_joint_angles(skeleton, angles):
    for bone in skeleton.bones:
        bone.alpha = angles[bone.idx*3 + 0]
        bone.beta = angles[bone.idx*3 + 1]
        bone.gamma = angles[bone.idx*3 + 2]


def get_end_effector(chain, skeleton):
    t_wcs = skeleton.bones[chain.bones[-1]].t_wcs
    q_wcs = skeleton.bones[chain.bones[-1]].q_wcs
    tool = chain.tool
    return t_wcs + Q.rotate(q_wcs, tool)


def compute_jacobian(chains, skeleton):
    num_angles = len(skeleton.bones)*3
    num_coords = len(chains)*3
    J = np.zeros((num_coords, num_angles), dtype=np.float64)
    row_offset = 0
    for chain in chains:
        e = get_end_effector(chain, skeleton)
        for idx in chain.bones:
            bone = skeleton.bones[idx]
            q_parent = Q.identity()
            if bone.has_parent():
                q_parent = skeleton.bones[bone.parent].q_wcs

            # In the 5 lines of code below the ZYZ Euler angles are hardwired into the code.
            q_alpha = bone.get_rotation_alpha()
            q_alpha_beta = Q.prod(q_alpha, bone.get_rotation_beta())

            u = Q.rotate(q_parent, bone.get_axis_alpha())
            v = Q.rotate(q_parent, Q.rotate(q_alpha, bone.get_axis_beta()))
            w = Q.rotate(q_parent, Q.rotate(q_alpha_beta, bone.get_axis_gamma()))

            delta_p = e - bone.t_wcs

            J_alpha = V3.cross(u, delta_p)
            J_beta = V3.cross(v, delta_p)
            J_gamma = V3.cross(w, delta_p)

            J[row_offset: row_offset+3, idx*3+0] = J_alpha
            J[row_offset: row_offset+3, idx*3+1] = J_beta
            J[row_offset: row_offset+3, idx*3+2] = J_gamma
        row_offset += 3
    return J


def compute_hessian(chains, skeleton, J):
    H = np.dot(np.transpose(J), J)
    row_offset = 0
    for chain in chains:
        e = get_end_effector(chain, skeleton)
        r = chain.goal - e
        for k in chain.bones:
            for h in chain.bones:
                if h > k:
                    continue

                Bh = skeleton.bones[h]

                q_parent = Q.identity()
                if Bh.has_parent():
                    q_parent = skeleton.bones[Bh.parent].q_wcs

                # In the 7 lines of code below the ZYZ Euler angles are hardwired into the code.
                q_alpha = Bh.get_rotation_alpha()
                q_beta = Bh.get_rotation_beta()
                q_gamma = Bh.get_rotation_gamma()
                q_alpha_beta = Q.prod(q_alpha, q_beta)

                u = Q.rotate(q_parent, Bh.get_axis_alpha())
                v = Q.rotate(q_parent, Q.rotate(q_alpha, Bh.get_axis_beta()))
                w = Q.rotate(q_parent, Q.rotate(q_alpha_beta, Bh.get_axis_gamma()))

                k_offset = k*3
                h_offset = h*3

                J_a = J[row_offset:row_offset+3, k_offset]
                J_b = J[row_offset:row_offset+3, k_offset+1]
                J_c = J[row_offset:row_offset+3, k_offset+2]

                ua = np.dot(V3.cross(u, J_a), r)
                va = np.dot(V3.cross(v, J_a), r)
                wa = np.dot(V3.cross(w, J_a), r)

                ub = np.dot(V3.cross(u, J_b), r)
                vb = np.dot(V3.cross(v, J_b), r)
                wb = np.dot(V3.cross(w, J_b), r)

                uc = np.dot(V3.cross(u, J_c), r)
                vc = np.dot(V3.cross(v, J_c), r)
                wc = np.dot(V3.cross(w, J_c), r)

                dH = np.array(
                    [[ua, va, wa],
                     [ub, vb, wb],
                     [uc, vc, wc]]
                )

                H[h_offset:h_offset+3, k_offset:k_offset+3] -= dH
                if h != k:
                    H[k_offset:k_offset+3, h_offset:h_offset+3] -= np.transpose(dH)

        row_offset += 3
    return H


def compute_gradient(chains, skeleton, J):
    r = np.zeros((len(chains)*3, ), dtype=np.float64)
    row_offset = 0
    for chain in chains:
        e = get_end_effector(chain, skeleton)
        r[row_offset:row_offset+3] = chain.goal - e
        row_offset += 3
    g = - np.dot(np.transpose(J), r)
    return g


def compute_objective(chains, skeleton):
    f = 0
    for chain in chains:
        e = get_end_effector(chain, skeleton)
        r = chain.goal - e
        f += np.dot(r, r)
    return f*0.5


def set_angle(idx, value, skeleton):
    k = idx // 3
    offset = idx % 3
    if offset == 0:
        skeleton.bones[k].alpha = value
    elif offset == 1:
        skeleton.bones[k].beta = value
    elif offset == 2:
        skeleton.bones[k].gamma = value
    else:
        raise RuntimeError('set_angle(): no such offset exist')


def get_angle(idx, skeleton):
    k = idx // 3
    offset = idx % 3
    if offset == 0:
        return skeleton.bones[k].alpha
    elif offset == 1:
        return skeleton.bones[k].beta
    elif offset == 2:
        return skeleton.bones[k].gamma
    else:
        raise RuntimeError('get_angle(): no such offset exist')


def __numerical_differentiation_second_derivative(chains, skeleton, i, j, h):
    if i == j:
        update_skeleton(skeleton)
        f_i = compute_objective(chains, skeleton)
        theta_i = get_angle(i, skeleton)
        set_angle(i, theta_i + h, skeleton)
        update_skeleton(skeleton)
        f_iph = compute_objective(chains, skeleton)
        set_angle(i, theta_i - h, skeleton)
        update_skeleton(skeleton)
        f_imh = compute_objective(chains, skeleton)
        set_angle(i, theta_i, skeleton)
        return (f_iph - 2*f_i + f_imh) / (h * h)
    else:
        theta_i = get_angle(i, skeleton)
        theta_j = get_angle(j, skeleton)
        set_angle(i, theta_i + h, skeleton)
        set_angle(j, theta_j + h, skeleton)
        update_skeleton(skeleton)
        f_iph_jph = compute_objective(chains, skeleton)
        set_angle(i, theta_i - h, skeleton)
        set_angle(j, theta_j + h, skeleton)
        update_skeleton(skeleton)
        f_imh_jph = compute_objective(chains, skeleton)
        set_angle(i, theta_i + h, skeleton)
        set_angle(j, theta_j - h, skeleton)
        update_skeleton(skeleton)
        f_iph_jmh = compute_objective(chains, skeleton)
        set_angle(i, theta_i - h, skeleton)
        set_angle(j, theta_j - h, skeleton)
        update_skeleton(skeleton)
        f_imh_jmh = compute_objective(chains, skeleton)
        set_angle(i, theta_i, skeleton)
        set_angle(j, theta_j, skeleton)
        return (f_iph_jph - f_iph_jmh - f_imh_jph + f_imh_jmh) / (4 * h * h)


def __numerical_differentiation_first_derivative(chains, skeleton, i, h):
    theta_i = get_angle(i, skeleton)
    set_angle(i, theta_i + h, skeleton)
    update_skeleton(skeleton)
    f_iph = compute_objective(chains, skeleton)
    set_angle(i, theta_i - h, skeleton)
    update_skeleton(skeleton)
    f_imh = compute_objective(chains, skeleton)
    set_angle(i, theta_i, skeleton)
    return (f_iph - f_imh) / (2 * h)


def finite_difference_gradient(chains, skeleton, h=0.1):
    N = len(skeleton.bones)*3
    g = np.zeros((N, ), dtype=np.float64)
    for i in range(N):
        g[i] = __numerical_differentiation_first_derivative(chains, skeleton, i, h)
    return g


def finite_difference_hessian(chains, skeleton, h=0.1):
    N = len(skeleton.bones)*3
    H = np.zeros((N, N), dtype=np.float64)
    for i in range(N):
        for j in range(N):
            H[i, j] = __numerical_differentiation_second_derivative(chains, skeleton, i, j, h)
    return H
