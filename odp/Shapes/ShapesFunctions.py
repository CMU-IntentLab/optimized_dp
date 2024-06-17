import numpy as np
import torch.nn as nn
import torch
import torch.nn.functional as F
from networks.mlp import MLP

def learnedShape(grid, path = '/home/kensuke/latent-safety/logs/failure_set.pth'):
    assert grid.dims==3

    #for i in range()
    x = np.array(grid.vs[0]).squeeze()
    y = np.array(grid.vs[1]).squeeze()
    z = np.array(grid.vs[2]).squeeze()/(2*np.pi)+0.5
    X, Y, Z = np.meshgrid(x, y, z, indexing='ij')
    grid_points = np.column_stack((X.flatten(), Y.flatten(), Z.flatten()))
    model = MLP(3, 1, 256)
    print(path)
    model.load_state_dict(torch.load(path))
    model.eval()  # Set the model to evaluation mode
    with torch.no_grad():  # Disable gradient calculation
        inputs = torch.tensor(grid_points, dtype=torch.float32)
        outputs = model(inputs)

    # Reshape the outputs to match the shape of the grid
    output_grid = outputs.reshape(X.shape).detach().numpy()

    return output_grid

def CylinderShape(grid, ignore_dims, center, radius):
    """Creates an axis align cylinder implicit surface function

    Args:
        grid (Grid): Grid object
        ignore_dims (List) : List  specifing axis where cylindar is aligned (0-indexed)
        center (List) :  List specifying the center of cylinder
        radius (float): Radius of cylinder

    Returns:
        np.ndarray: implicit surface function of the cylinder
    """
    data = np.zeros(grid.pts_each_dim)
    for i in range(grid.dims):
        if i not in ignore_dims:
            # This works because of broadcasting
            data = data + np.power(grid.vs[i] - center[i], 2)
    #data = np.sqrt(data) - radius
    return data - radius*radius

# Range is a list of list of ranges
# def Rectangle4D(grid, range):
#     data = np.zeros(grid.pts_each_dim)
#
#     for i0 in (0, grid.pts_each_dim[0]):
#         for i1 in (0, grid.pts_each_dim[1]):
#             for i2 in (0, grid.pts_each_dim[2]):
#                 for i3 in (0, grid.pts_each_dim[3]):
#                     x0 = grid.xs[i0]
#                     x1 = grid.xs[i1]
#                     x2 = grid.xs[i2]
#                     x3 = grid.xs[i3]
#                     range_list = [-x0 + range[0][0], x0 - range[0][1],
#                                   -x1 + range[1][0], x1 - range[1][1],
#                                   -x2 + range[2][0], x2 - range[2][1],
#                                   -x3 + range[3][0], x3 - range[3][1]]
#                     data[i0, i1, i2, i3] = min(range_list)
#     return data

def ShapeRectangle(grid, target_min, target_max):
    data = np.maximum(grid.vs[0] - target_max[0], -grid.vs[0] + target_min[0])

    for i in range(grid.dims):
        data = np.maximum(data,  grid.vs[i] - target_max[i])
        data = np.maximum(data, -grid.vs[i] + target_min[i])

    return data

def Rect_Around_Point(grid, target_point):
    return ShapeRectangle(grid, target_point - 1.5 * grid.dx, target_point + 1.5 * grid.dx)


def Lower_Half_Space(grid, dim, value):
    """Creates an axis aligned lower half space 

    Args:
        grid (Grid): Grid object
        dim (int): Dimension of the half space (0-indexed)
        value (float): Used in the implicit surface function for V < value

    Returns:
        np.ndarray: implicit surface function of the lower half space
                    of size grid.pts_each_dim
    """
    data = np.zeros(grid.pts_each_dim)
    for i in range(grid.dims):
        if i == dim:
            data += grid.vs[i] - value
    return data


def Upper_Half_Space(grid, dim, value):
    """Creates an axis aligned upper half space 

    Args:
        grid (Grid): Grid object
        dim (int): Dimension of the half space (0-indexed)
        value (float): Used in the implicit surface function for V > value

    Returns:
        np.ndarray: implicit surface function of the lower half space
                    of size grid.pts_each_dim
    """
    data = np.zeros(grid.pts_each_dim)
    for i in range(grid.dims):
        if i == dim:
            data += -grid.vs[i] + value
    return data

def Union(shape1, shape2):
    """ Calculates the union of two shapes

    Args:
        shape1 (np.ndarray): implicit surface representation of a shape
        shape2 (np.ndarray): implicit surface representation of a shape

    Returns:
        np.ndarray: the element-wise minimum of two shapes
    """
    return np.minimum(shape1, shape2)

def Intersection(shape1, shape2):
    """ Calculates the intersection of two shapes

    Args:
        shape1 (np.ndarray): implicit surface representation of a shape
        shape2 (np.ndarray): implicit surface representation of a shape

    Returns:
        np.ndarray: the element-wise minimum of two shapes
    """
    return np.maximum(shape1, shape2)

def ShapeEllipsoid(grid, center, semiAxLen):
    data = np.zeros(grid.pts_each_dim)  
    for i in range(grid.dims):
        data = data + np.power(grid.vs[i] - center[i],2)/np.power(semiAxLen[i],2)
    data = np.sqrt(data) - 1
    return data
