"""Sphere-swept link model + analytic point-to-obstacle signed distance (v2).

Why this exists (cause #2 of the "jumpy robot" in tutorial 05):
    FCL returns ONE nearest point per (link primitive, obstacle) pair. When a
    long primitive (e.g. the link7 or the hand cylinder) lies almost
    parallel to a wall / the floor, every point along it is (almost) equally
    close, and a 0.1 deg tilt moves the nearest point from one end of the link
    to the other (0.2 m jump). The point Jacobian built at that point jumps
    too, so the QP answer (qd_des) flips sign from one plan to the next.

    Here each link primitive is covered by a short chain of spheres. A sphere
    has a UNIQUE nearest point to a convex obstacle, and that point moves
    continuously with q, so the rows of the QP change continuously.

Conventions
    * Cylinders are along the local z axis of their geom frame (FCL / trimesh).
    * Boxes use FULL extents (same as ``get_fcl_geom``).
    * Sphere centres are returned in the LINK frame (the geom ``offset`` is
      already applied), so ``p_world = R_link @ c + p_link``.
    * ``signed_distance_*`` returns (sd, p_obs, n) with n the unit vector that
      points from the obstacle surface towards the query point. sd < 0 means
      the point is inside the obstacle.
"""

from __future__ import annotations

import numpy as np


# --------------------------------------------------------------------------
# Link primitive -> spheres
# --------------------------------------------------------------------------
def spheres_from_primitive(
    geom_type: str, size, offset, spacing_ratio: float = 1.0
) -> tuple[np.ndarray, np.ndarray]:
    """Cover one primitive with spheres.

    Args:
        geom_type: "cylinder" | "box" | "sphere".
        size: cylinder (radius, length) / box full extents (x, y, z) / sphere (r,).
        offset: 4x4 pose of the primitive in the link frame.
        spacing_ratio: centre spacing as a fraction of the sphere radius
            (1.0 -> neighbouring spheres overlap by one radius).

    Returns:
        centres (N, 3) in the link frame, radii (N,).
    """
    size = np.asarray(size, dtype=np.float64).reshape(-1)
    offset = np.asarray(offset, dtype=np.float64).reshape(4, 4)

    if geom_type == "sphere":
        local = np.zeros((1, 3))
        radii = np.array([size[0]])
    elif geom_type == "cylinder":
        r, length = float(size[0]), float(size[1])
        n = max(2, int(np.ceil(length / (spacing_ratio * r))) + 1)
        z = np.linspace(-0.5 * length, 0.5 * length, n)
        local = np.stack([np.zeros(n), np.zeros(n), z], axis=1)
        radii = np.full(n, r)
    elif geom_type == "box":
        half = 0.5 * size
        axis = int(np.argmax(half))
        others = [i for i in range(3) if i != axis]
        # radius that covers the box cross-section perpendicular to the long axis
        r = float(np.linalg.norm(half[others]))
        length = 2.0 * half[axis]
        n = max(2, int(np.ceil(length / (spacing_ratio * r))) + 1)
        s = np.linspace(-half[axis], half[axis], n)
        local = np.zeros((n, 3))
        local[:, axis] = s
        radii = np.full(n, r)
    else:
        raise ValueError(f"Unsupported primitive for sphere model: {geom_type}")

    centres = local @ offset[:3, :3].T + offset[:3, 3]
    return centres, radii


# --------------------------------------------------------------------------
# Analytic signed distance: points -> obstacle (vectorised)
# --------------------------------------------------------------------------
def signed_distance_box(points_w, T_w_box, full_size):
    """Signed distance from points to a box surface.

    Args:
        points_w: (N, 3) world points.
        T_w_box: 4x4 world pose of the box centre.
        full_size: (3,) full extents.

    Returns:
        sd (N,), p_obs_w (N, 3) closest surface point, n_w (N, 3) unit normal
        (obstacle -> point).
    """
    P = np.atleast_2d(np.asarray(points_w, dtype=np.float64))
    R = T_w_box[:3, :3]
    t = T_w_box[:3, 3]
    h = 0.5 * np.asarray(full_size, dtype=np.float64)
    pb = (P - t) @ R  # into the box frame (R^T p)

    q = np.abs(pb) - h
    outside = np.any(q > 0.0, axis=1)

    sd = np.empty(len(P))
    p_obs_b = np.empty_like(pb)
    n_b = np.empty_like(pb)

    if np.any(outside):
        po = pb[outside]
        clamp = np.clip(po, -h, h)
        d = po - clamp
        dist = np.linalg.norm(d, axis=1)
        sd[outside] = dist
        p_obs_b[outside] = clamp
        n_b[outside] = d / np.maximum(dist, 1e-12)[:, None]

    inside = ~outside
    if np.any(inside):
        pi = pb[inside]
        qi = q[inside]
        axis = np.argmax(qi, axis=1)  # face with the least penetration
        rows = np.arange(len(pi))
        sd[inside] = qi[rows, axis]  # <= 0
        sgn = np.where(pi[rows, axis] >= 0.0, 1.0, -1.0)
        proj = pi.copy()
        proj[rows, axis] = sgn * h[axis]
        p_obs_b[inside] = proj
        nb = np.zeros_like(pi)
        nb[rows, axis] = sgn
        n_b[inside] = nb

    return sd, p_obs_b @ R.T + t, n_b @ R.T


def signed_distance_cylinder(points_w, T_w_cyl, size):
    """Signed distance from points to a z-axis cylinder (radius, full length)."""
    P = np.atleast_2d(np.asarray(points_w, dtype=np.float64))
    R = T_w_cyl[:3, :3]
    t = T_w_cyl[:3, 3]
    radius, half_len = float(size[0]), 0.5 * float(size[1])
    pc = (P - t) @ R

    rho = np.linalg.norm(pc[:, :2], axis=1)
    radial = pc[:, :2] / np.maximum(rho, 1e-12)[:, None]
    radial[rho < 1e-12] = np.array([1.0, 0.0])

    dr = rho - radius
    dz = np.abs(pc[:, 2]) - half_len
    sgn_z = np.where(pc[:, 2] >= 0.0, 1.0, -1.0)

    sd = np.empty(len(P))
    p_obs = np.empty_like(pc)
    n = np.empty_like(pc)

    outside = (dr > 0.0) | (dz > 0.0)
    if np.any(outside):
        o = outside
        rho_c = np.minimum(rho[o], radius)
        z_c = np.clip(pc[o, 2], -half_len, half_len)
        cp = np.concatenate([radial[o] * rho_c[:, None], z_c[:, None]], axis=1)
        d = pc[o] - cp
        dist = np.linalg.norm(d, axis=1)
        sd[o] = dist
        p_obs[o] = cp
        n[o] = d / np.maximum(dist, 1e-12)[:, None]

    inside = ~outside
    if np.any(inside):
        i = inside
        use_side = dr[i] > dz[i]  # closer to the side than to a cap
        sd[i] = np.maximum(dr[i], dz[i])
        cp = pc[i].copy()
        nn = np.zeros_like(cp)
        side_idx = np.where(use_side)[0]
        cap_idx = np.where(~use_side)[0]
        cp[side_idx, :2] = radial[i][side_idx] * radius
        nn[side_idx, :2] = radial[i][side_idx]
        cp[cap_idx, 2] = sgn_z[i][cap_idx] * half_len
        nn[cap_idx, 2] = sgn_z[i][cap_idx]
        p_obs[i] = cp
        n[i] = nn

    return sd, p_obs @ R.T + t, n @ R.T


def signed_distance_sphere(points_w, T_w_sph, size):
    P = np.atleast_2d(np.asarray(points_w, dtype=np.float64))
    c = T_w_sph[:3, 3]
    r = float(size[0])
    d = P - c
    dist = np.linalg.norm(d, axis=1)
    n = d / np.maximum(dist, 1e-12)[:, None]
    n[dist < 1e-12] = np.array([0.0, 0.0, 1.0])
    return dist - r, c + n * r, n


def signed_distance(points_w, obstacle: dict):
    """Dispatch on ``obstacle = {"type", "size", "T"}`` (world pose)."""
    kind = obstacle["type"]
    if kind == "box":
        return signed_distance_box(points_w, obstacle["T"], obstacle["size"])
    if kind == "cylinder":
        return signed_distance_cylinder(points_w, obstacle["T"], obstacle["size"])
    if kind == "sphere":
        return signed_distance_sphere(points_w, obstacle["T"], obstacle["size"])
    raise ValueError(f"Unsupported obstacle type: {kind}")
