import numpy as np
from scipy.spatial import HalfspaceIntersection, ConvexHull
from scipy.optimize import linprog

def find_feasible_point(halfspaces, margin=0.01):
    """
    Solve Linear Program to find a feasible point strictly inside the polyhedron.
    halfspaces: (N, 4) array where each row is [a, b, c, d] representing a*x + b*y + c*z + d <= 0
    """
    A = halfspaces[:, :3]
    b = -halfspaces[:, 3] - margin  # Shrink polytope a bit to ensure strict feasibility

    # print("A is ", A)
    # print("b is ", b)

    n_vars = 3
    n_halfspaces = A.shape[0]

    c = np.zeros(n_vars)  # Objective is arbitrary (we just need a feasible point)

    res = linprog(c, A_ub=A, b_ub=b, bounds=(None, None))
    if res.success:
        return res.x
    else:
        raise RuntimeError("Failed to find a feasible point inside the inflated polyhedron.")

def hrep_to_vertices(halfspaces):
    """
    Convert H-representation polyhedron to V-representation (list of vertices).
    halfspaces: (N, 4) array where each row is [a, b, c, d] representing a*x + b*y + c*z + d <= 0
    """
    feasible_point = find_feasible_point(halfspaces)
    hs = HalfspaceIntersection(halfspaces, feasible_point)
    return hs.intersections

# Example usage:
# halfspaces = np.array([...])  # Nx4 array with [a, b, c, d]
# vertices = hrep_to_vertices(halfspaces)
# print(vertices)
