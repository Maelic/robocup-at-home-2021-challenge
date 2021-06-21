import numpy as np
import trimesh

class ErrorComputingRayTriangleIntersection(Exception):
    """Error when computing the intersection between a ray and a mesh triangle"""
    pass

def mesh_to_grasps(mesh, gripper, MAX_MESH_SIDE_LENGTH=0.2, TOTAL_RAYS_TO_USE=512, use_only_external_points=True):
    # Version 1: Use the face normals for grasp axes
    #            Face normals start at the centroid of their face triangles.
    # Formula: https://en.wikipedia.org/wiki/Centroid#Of_a_triangle
    # mesh_triangles = [[mesh.vertices[triangle[0]], mesh.vertices[triangle[1]], mesh.vertices[triangle[2]]] for triangle in mesh.faces]
    mesh_triangles = np.array(mesh.vertices[mesh.faces]) # Create triplets of triangle points
    face_centroids = np.array([np.mean(triangle, axis=0) for triangle in mesh_triangles])
    # http://jwilson.coe.uga.edu/EMAT6680/Dunbar/Assignment4/Assignment4_KD.htm

    ### Compute the rays: origins and directions (using face normals)
    ray_origins = []
    ray_directions = []

    # Limit to a random TOTAL_RAYS_TO_USE rays
    randomSelectionIndexes = np.arange(len(mesh.face_normals)) # ray_directions = normals
    np.random.shuffle(randomSelectionIndexes)
    # randomSelectionIndexes = randomSelectionIndexes[:TOTAL_RAYS_TO_USE] #
    randomSelectionIndexes = randomSelectionIndexes[:min(TOTAL_RAYS_TO_USE, len(mesh.face_normals))]

    # print(randomSelectionIndexes)

    # print("Ray origins #: {0}".format(ray_origins))
    # print("Ray directions #: {0}".format(ray_directions))
    ray_origins = np.array(face_centroids[randomSelectionIndexes])
    ray_directions = np.array(mesh.face_normals)[randomSelectionIndexes] # Use face normals

    # Face normals have norm 1.0
    # The bounding box of the mesh has side of 0.2 (as defined in the mesh generation file)
    ray_origins = ray_origins - (MAX_MESH_SIDE_LENGTH*ray_directions)
    ray_directions *= 2 * (MAX_MESH_SIDE_LENGTH)

    # Compute intersection points for all these rays
    successfully_computed_intersection_points = False
    while (not successfully_computed_intersection_points):
        try:
            # run the mesh-ray query
            # Returns:
            #     locations ((n) sequence of (m,3) float) - Intersection points
            #     index_ray ((n,) int) - Array of ray indexes
            #     index_tri ((n,) int) - Array of triangle (face) indexes
            locations, index_ray, index_tri = mesh.ray.intersects_location(
                                                    ray_origins=ray_origins,
                                                    ray_directions=ray_directions) # multiple_hits=False)
            successfully_computed_intersection_points = True
        except:
            print("Mesh faces: {}".format(len(mesh.face_normals)))
            # import pdb; pdb.set_trace()
            raise ErrorComputingRayTriangleIntersection

            # print("Retrying computing grasps with fewer rays")
            # successfully_computed_grasps = False
            #     # This solution may fail (if the bad pair of points is at the beginning of the array)
            #     if (len(randomSelectionIndexes) > 32):
            #         randomSelectionIndexes = randomSelectionIndexes[0:(len(randomSelectionIndexes)//2)]
            #         #
            #         ray_origins = np.array(face_centroids[randomSelectionIndexes])
            #         ray_directions = np.array(mesh.face_normals)[randomSelectionIndexes] # Use face normals
            #         #
            #         # Face normals have norm 1.0
            #         ray_origins = ray_origins - (MAX_MESH_SIDE_LENGTH*ray_directions)
            #         ray_directions *= 2 * (MAX_MESH_SIDE_LENGTH)
            #     # else: any other way to solve it?


    # Onwards, compute the Grasp structures:
    #     - group the contact points by rays
    #     - contact points
    #     - faces (IDs) on which these points are located

    # print("Total intersections: {}".format(len(locations)))
    # print("Intersection locations:\n{}".format(locations))
    # print("Ray IDs:\n{}".format(index_ray))
    # print("Triangle IDs:\n{}".format(index_tri))

    # Group the intersection points by the rays that generated them
    # We want to create a structure here of the type:
    # Ray ID: array [intersection points]
    rays_intersection_points = {}
    rays_intersection_faces = {}
    for i in range(len(index_ray)):
        if index_ray[i] not in rays_intersection_points:
            # Create a new entry for this ray
            rays_intersection_points[index_ray[i]] = []
            rays_intersection_faces[index_ray[i]] = []
            # Add the corresponding intersection point
        rays_intersection_points[index_ray[i]].append(locations[i])
        rays_intersection_faces[index_ray[i]].append(index_tri[i])



    #     ### Visualise the rays
    #     # stack rays into line segments for visualization as Path3D
    #     ray_visualize = trimesh.load_path(np.hstack((ray_origins,
    #                                                  ray_origins+ray_directions)).reshape(-1, 2, 3))

    #     # Visualise only the mesh:
    #     # mesh.show()

    #     # unmerge so viewer doesn't smooth
    #     mesh.unmerge_vertices()
    #     # make mesh white-ish
    #     mesh.visual.face_colors = [255,255,255,255]
    #     mesh.visual.face_colors[index_tri] = [255, 0, 0, 255]

    #     # Create a visualization scene with rays, hits, and mesh
    #     #scene = trimesh.Scene([mesh, ray_visualize, mesh.bounding_sphere])
    #     scene = trimesh.Scene([mesh, ray_visualize])

    #     # show the visualization
    #     scene.show()



    # CHECK WHICH INTERSECTION POINTS ARE CLOSER THAN THE GRIPPER WIDTH
    # The container that will hold grasps in format:
    # rayID: (point1, point2)
    grasps = []
    total_grasps=0

    for rayId in rays_intersection_points: # loop through the keys of the dictionary
        # print("\nRay {}:".format(i))
        # for j in range(len(rays_intersections[i])):
        #     print(rays_intersections[i][j])

        # Assumption: each ray intersects the mesh in at least 2 points (false when it touches a face)
        # Try: (max width)
        # If that is wider than maxGripperWidth,
        # then try smaller and smaller until first success
        # or until we run out of points

        # Constraint:
        # Distance between two successive points in empty space
        # should be bigger than finger thickness (X&Y)

        totalRayIntersectionPoints = len(rays_intersection_points[rayId])
        # print("Total intersection points: {}".format(totalRayIntersectionPoints))

        # Ensure you have an even number of intersection points (not true for objects with 0-volume surfaces)
        # assert (totalRayIntersectionPoints%2==0),"Ray #{} has an odd number of intersections: {}".format(rayId,totalRayIntersectionPoints)


        # If you want to use only the external points (to avoid problems with grasp reachability)
        if (use_only_external_points):
            startID = 0
            endID = totalRayIntersectionPoints - 1
            distance = np.linalg.norm(rays_intersection_points[rayId][startID] - rays_intersection_points[rayId][endID])
            # print("Distance: {}".format(distance))
            # Reject pair of points if they are too far apart
            # and if there is only 1 intersection point (i.e. P1 = P2)
            if ((distance < gripper['width']) and (totalRayIntersectionPoints > 1)):
                grasp = {}
                grasp["point_locations"] = (rays_intersection_points[rayId][startID], rays_intersection_points[rayId][endID])
                grasp["point_faces"] = (rays_intersection_faces[rayId][startID], rays_intersection_faces[rayId][endID])
                #
                # Additional info (just in case)
                grasp["ray_id"] = rayId
                grasp["grasp_id"] = total_grasps
                # grasps.append((rays_intersections[rayId][startID],rays_intersections[rayId][endID]))
                grasps.append(grasp)
                total_grasps += 1

        # If you want to
        # Generate grasps at all point pairs on a ray intersecting a mesh (event interior ones)
        else: #if (not use_only_external_points):
            # WARNING: If you do not want to work with pairs of contact points,
            # then you have to check the presence of intersections between the two contact points and the mesh.
            for startID in range(0,totalRayIntersectionPoints,2):
                for endID in range(startID+1,totalRayIntersectionPoints,2):
                    # Test if points are closer than max gripper width
                    # print(rays_intersections[i][startID])
                    # print(rays_intersections[i][endID])
                    distance = np.linalg.norm(rays_intersection_points[rayId][startID] - rays_intersection_points[rayId][endID])
                    # print("Distance: {}".format(distance))
                    if (distance < gripper['width']):
                        grasp = {}
                        grasp["point_locations"] = (rays_intersection_points[rayId][startID], rays_intersection_points[rayId][endID])
                        grasp["point_faces"] = (rays_intersection_faces[rayId][startID], rays_intersection_faces[rayId][endID])
                        #
                        # Additional info (just in case)
                        grasp["ray_id"] = rayId
                        grasp["grasp_id"] = total_grasps
                        # grasps.append((rays_intersections[rayId][startID],rays_intersections[rayId][endID]))
                        grasps.append(grasp)
                        total_grasps += 1

    # Check
    print("Total rays: {}".format(len(rays_intersection_points)))
    print("Total grasps: {} ({})".format(len(grasps), total_grasps))
    print("Mesh volume: {}".format(mesh.volume))
    # print(grasps)
    if (total_grasps == 0):
        # Debugging
        mesh.show()


    return grasps
