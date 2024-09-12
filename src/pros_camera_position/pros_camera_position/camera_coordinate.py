import numpy as np
    
class convert_camera_coordinates:
    def __init__(self, camera_matrix, extrinsic_matrix, rotation_matrix, camera_transfrom):
        self.camera_matrix = camera_matrix
        self.extrinsic_matrix = extrinsic_matrix
        self.rotation_matrix = rotation_matrix
        self.camera_transfrom = camera_transfrom

    def unity_to_reality_coordinates(self, coords):
        """
        Converts Unity coordinates to reality coordinates.

        Parameters:
        coords (np.ndarray): A NumPy array with shape (n, 3), where n is the number of coordinates.
                            Each row should contain (x, y, z) coordinates.

        Returns:
        np.ndarray: A NumPy array with shape (n, 3) containing the converted reality coordinates (x, -y, -z).
        """            
        if coords.shape[1] == 4:        
            converted_coords = np.empty_like(coords)
            converted_coords[:, 0] = coords[:, 0]  
            converted_coords[:, 1] = -coords[:, 1]  
            converted_coords[:, 2] = coords[:, 2]  
            converted_coords[:, 3] = coords[:, 3]  
            return converted_coords

        if coords.shape[1] == 3:
            converted_coords = np.empty_like(coords)
            converted_coords[:, 0] = coords[:, 0]  
            converted_coords[:, 1] = -coords[:, 1]  
            converted_coords[:, 2] = coords[:, 2]

        return converted_coords

    def world_point_to_screen_point(self, world_points, camera_matrix, extrinsic_matrix):
        camera_matrix = np.hstack((camera_matrix, np.zeros((camera_matrix.shape[0], 1))))
        world_points = self.unity_to_reality_coordinates(world_points)
        image_points = camera_matrix @ extrinsic_matrix @ world_points.T
        print("depth",image_points[2])
        image_points = image_points / image_points[2]
        return image_points[:2]
    
    def screen_point_to_world_point(self, scream_points, depth):
        world_points = np.linalg.inv(self.rotation_matrix) @ (np.linalg.inv(self.camera_matrix) @ scream_points.T * depth - self.camera_transfrom.T)
        return world_points.T[0]
  
    
    def check(self):
        _3d_path_points_list = [[0,0,0], [0,1,0], [0,2,0], [0,3,0], [0,4,0], [0,5,0]]
        for _3d_points in _3d_path_points_list:
            _2d_points = self.camera_matrix @ self.extrinsic_matrix[:3, :] @ np.append(_3d_points, 1)
            _2d_points = _2d_points / _2d_points[2]
            print(_2d_points)