# Parameters

``bb_dim_x``
    Dimension in x (forward) direction of the car/robot  
``bb_dim_y``
    Dimension in y direction of the car/robot  
``bb_dim_z``
    Dimension in z direction (height) of the car/robot  
``wheelbase``
    The wheelbase of the car/robot  
``track``
    Track width of the car/robot  
``simplify_solution``
    Configure OMPL to prune a solution path  
``distance_between_poses``
    Distance between the pose samples of the conversion step of the OMPL path to a ROS path  
``search_radius_nn``
    Nearest neighbor search radius in the traversability point cloud  
``ratio_traversables``
    Ratio of traversable neighboring points in the traversability point cloud  
``relax_neighbor_search``
    When projecting sampled points onto the surface, check the more points if the nearest point is not traversable  
``clearance``
    Clearance between collision object and ground  
``car_angle_delta_deg``
    The rectangle of all four contact points is divided into two triangles, this parameter defines the maximum angle between them  
``state_validity_resolution``
    Resolution used by OMPL to discretize a motion during motion validation step  


