import warnings
import numpy as np


def Rt2Trans(R, t):
    """Constructs a 4x4 transformation matrix from a rotation matrix 
    and a translation vector.

    Args:
        R (np.ndarray): 3x3 rotation matrix.
        t (np.ndarray): (3,) translation vector.

    Returns:
        np.ndarray: 4x4 transformation vector (SE(3))
    """
    return np.vstack((np.hstack((R, t)), [[0, 0, 0, 1]]))

def Trans2Rt(T):
    """Unpacks a transformation matrix into its rotation matrix and 
    translation vector. 

    Args:
        T (np.ndarrar): 4x4 transformation matrix.

    Returns:
        R and t
    """
    return T[0: 3, 0: 3], T[0: 3, 3]


def K_inverse(K):
    """Computes the inverse of a 3x3 camera matrix.

    Args:
        K (np.ndarray): A 3x3 camera matrix.

    Returns: 
        Kinv (np.ndarray): The inverse of K.
    """

    fx, fy, s, cx, cy = K[0,0], K[1,1], K[0, 1], K[0,2], K[1,2]

    Kinv = np.array([[1/fx, -s/(fx*fy), -cx/fx + cy*s/(fx*fy)],
                    [   0,       1/fy,                -cy/fy],
                    [   0,          0,                     1]])
    
    return Kinv

def T_inverse(T):
    """Computes the inverse of a 4x4 transformation matrix (SE(3))

    Example: 
        If $T_{ab}$ is the transformation matrix which projects homogeneous
        points in frame $\{b\}$ to frame $\{a\}$, then this function computes
        $T_{ba}$ which projects points in the opposite direction.
    
    Args:
        T (np.ndarray): A 4x4 transformation matrix.

    Returns: 
        Tinv (np.ndarray): The inverse of T.
    """    

    R, t = T[0: 3, 0: 3], T[0: 3, 3]
    Rinv = R.T 
    tinv = -Rinv @ t
    return np.vstack((np.hstack((Rinv, tinv)), [[0, 0, 0, 1]]))


def compute_extrinsic(K, H): 
    """Compute the extrinsic camera matrix from the camera matrix and
    an homography.

    Source: Zhang, Zhengyou. "A flexible new technique for camera calibration."

    Args:
        K (np.ndarray): 
            A 3x3 camera matrix.
        H (np.ndarray): 
            A 3x3 matrix describing an homography. 
            E.g. the homography returned by `cv2.findHomography()`
    Returns: 
        T (np.ndarray):
            A 4x4 transformation matrix (SE(3)) describing the transformation
            from world space to camera space.
    """    

    Kinv = K_inverse(K)
    
    # Pg. 6, Z. Zhang "A Flexible New Technique for Camera Calibration"
    h1, h2, h3 = H.T
    lambd = 1/np.linalg.norm(Kinv @ h1)
    r1 = lambd * Kinv @ h1
    r2 = lambd * Kinv @ h2
    r3 = np.cross(r1, r2)
    t = lambd * Kinv @ h3

    R = np.vstack((r1, r2, r3)).T
    # We refine the rotation matrix to ensure det(R) = 1
    # Appendix C, Z. Zhang "A Flexible New Technique for Camera Calibration"
    U, _, Vt = np.linalg.svd(R) 
    R = U @ Vt

    return np.vstack((np.hstack((R, t)), [[0, 0, 0, 1]])) 

def homogeneize(points):
    """Converts cartesian coordinates into homogeneous coordinates. I.e. appends
    an additional dimension to the list of points.

    Args:
        points (np.ndarray): An MxN matrix of N points with M dimensions.
    
    Returns
        np.ndarray: An (M+1)xN matrix.

    """
    return np.vstack([points, np.ones(points.shape[1])]) 



def dehomogeneize(points):
    """Converts homogeneous coordinates back to cartesian coordinates by 
    element-wise division by the last row.

    Args:
        points (np.ndarray): An MxN matrix of N points with M dimension. 
        The M'th row is assumed to be the additional dimension.

    Returns: 
        np.ndarray: Cartesian coordinates.
    """
    return points[:-1, :]/points[-1, :]    


def proj_points(p_w, K, T_cw, flip=False):
    """Project points using 

    Args:
        p_w (np.ndarray): (n, 3) or (n, 4) np.ndarray. 
        K (np.ndarray): A 3x3 camera matrix
        Tcw (np.ndarray): An extrinsic camera matrix. Default: Identity matrix.
        flip (bool, optional): [description]. Defaults to False.

    Returns:
        p_i (np.ndarray): 
    """    
    pass 
    p_cam = T_cw @ p_w.T
    if flip:
        p_img = K @ (p_cam[:3, :] * np.array([[-1, 1, 1]]).T)
    else: 
        p_img = K @ p_cam[:3, :]
    return (p_img/p_img[2, :]).T

def link(uri, label=None, parameters=None):
    '''
    Generate a string that is a clickable URI when printed in the terminal. 

    As described by here: 
    https://gist.github.com/egmontkob/eb114294efbcd5adb1944c9f3cb5feda
    
    Args:
        uri (string): The URI to be linked to. 
        label (string): Label of the URI. If no label is give, the URI is used as the label.
            Default: None. 
        parameters (string): Not used. For future extendability. Default: None

    Example usage:

        > print(link("http://asdf.com", "click me"))
        click me # This is a clickable link labeled "click me"
    '''
    if label is None: 
        label = uri
    if parameters is None: 
        parameters = ''

    # OSC 8 ; params ; URI ST <name> OSC 8 ;; ST 
    escape_mask = '\033]8;{};{}\033\\{}\033]8;;\033\\'

    return escape_mask.format(parameters, uri, label)
