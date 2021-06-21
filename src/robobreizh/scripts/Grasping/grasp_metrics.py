import numpy as np
import math
import mesh_to_grasps

class ErrorGraspScore(Exception):
	"""Base class for exceptions when calculating grasp score"""
	pass


class ErrorGraspAngularDistanceScore(ErrorGraspScore):
	"""Error when calculating the grasp angular distance score"""
	pass

class ErrorGraspCentroidDistanceScore(ErrorGraspScore):
	"""Error when calculating the grasp centroid distance score"""
	pass

### Metric #1: Distance between grasp centroid and mesh centroid
def _getScoreCentroidDistance(mesh, grasp):

    meshCentroid = mesh.centroid
    graspCentroid = np.mean(grasp, axis=0)

    score = np.linalg.norm(graspCentroid - meshCentroid)
    score = np.around(score, decimals=4)

    # The furthest that a point in the mesh can be from the mesh centroid is
    # (not half, but the full length of) the diagonal of the bounding box
    normalisation_factor = np.linalg.norm(mesh.extents) # the diagonal of the bounding box
    score /= normalisation_factor
    return score


### Metric #2: Angular distance between grasp axis and face normals
# Compute the angle between two vectors in 3D (in radians)
# Source: https://www.youtube.com/watch?v=QWIZXRjMspI
# ray_direction, face_normal
def angular_distance_in_rad(vector1, vector2):
	precisionInDigitsAfterDot = 6
	return angular_distance_in_rad_with_precision(vector1, vector2, precisionInDigitsAfterDot)

def angular_distance_in_rad_with_precision(vector1, vector2, precisionInDigitsAfterDot):
    # Make sure that the ray and the normal point into the same direction (or are orthogonal)
    # If the ray and the normal are pointing into opposite directions, then change the direction of the ray.
	# Effect: all measured angles are between [0, 90] degrees or [0, Pi/2] radians
    if (np.dot(vector1, vector2) < 0.0):
        vector1 = np.array(vector1) * -1
    #         print("Before: {}\nAfter:{}".format(-vector1, vector1))
    #     else:
    #         print("No change: {}".format(vector1))

    # Step 1: compute the projection of vector1 on vector2, times the length of that vector (i.e. the dot product)
    dot_product = np.dot(vector1, vector2)

    # Step 2: compute the norm of each vector (vector1 and vector2)
    vector1_norm = np.linalg.norm(vector1)
    vector2_norm = np.linalg.norm(vector2)
    # print("Norm: {} {}".format(vector1_norm, vector2_norm))

    # Step 3: compute the arc cosinus of the vectors,
    # This should have worked, but breaks the code with some value close to 1.0 but different from 1.0
    # angle = np.arccos(dot_product / (vector1_norm * vector2_norm)) # returns the angle in radians
    dot_product = round(dot_product, precisionInDigitsAfterDot)
    norm_product = round(vector1_norm * vector2_norm, precisionInDigitsAfterDot)

    if (norm_product == 0.0):
        print("Norm product is zero! Caused by the following values:")
        print("vector1: {}".format(vector1))
        print("vector2: {}".format(vector2))
        print("vector1_norm: {}".format(vector1_norm))
        print("vector2_norm: {}".format(vector2_norm))
        return -1

    # WARNING: division by zero possible here.
    cos_angle = dot_product / norm_product
    angle = np.arccos(cos_angle)

    # TODO: error generated here, apparently because the NORM and the DOT PRODUCT are computed in different ways.
    if (math.isnan(angle)):
        print("NaN error")
        print("Value passed to arccos({} / {} = {})".format(dot_product, (vector1_norm * vector2_norm), dot_product / (vector1_norm * vector2_norm)))
		# With a small chance (1/100k), it may happen that the
        # assert(dot_product / (vector1_norm * vector2_norm) == 1.0), ("{} != 1.0".format(dot_product / (vector1_norm * vector2_norm)))
        print("Lowering precision after dot to {} decimals".format(precisionInDigitsAfterDot))
        if (precisionInDigitsAfterDot > 1):
            return angular_distance_in_rad_with_precision(vector1, vector2, precisionInDigitsAfterDot-1)
        else:
            print("Error calculating the arccos of the angle (grasp_metrics.py)")
            return -1
    #     else:
    #         print("Value passed to arccos({} / {} = {})".format(dot_product, (vector1_norm * vector2_norm), dot_product / (vector1_norm * vector2_norm)))

    # Return the result
    return angle


# Compute the angular distance of the ray defining the grasp axis, and the normals to the faces that it intersects.
def angular_distance_graspAxis_vs_meshFaces(grasp, mesh):
    # Plan:
    # 1. Compute the faces that the grasp axis intersects
    # 2. Extract the normals of those faces
    # 3a. Compute the angular distance between the grasp axis and faceNormal1
    # 3b. Compute the angular distance between the grasp axis and faceNormal2
    # 4. Sum the two distances, and return the result. (you can use sum of squares here)

    # Implementation:
    # 1. Compute the faces that the grasp axis intersects
        # grasp["contact_points"]
        # grasp["contact_faces_id"]
    # 2. Extract the normals of those faces
    # triangles = mesh.faces
    # triangles_pierced = triangles[index_tri]
    # print("Triangle IDs:\n{}".format(grasp["point_faces"]))
    triangles_pierced_normals = mesh.face_normals[np.array(grasp["point_faces"])]
    # print("Triangle normals:\n{}".format(triangles_pierced_normals))

    # Reminder:
    #     grasp = {}
    #     grasp["point_locations"] = (rays_intersection_points[rayId][startID], rays_intersection_points[rayId][endID])
    #     grasp["point_faces"] = (rays_intersection_faces[rayId][startID], rays_intersection_faces[rayId][endID])
    #     #
    #     # Additional info (just in case)
    #     grasp["ray_id"] = rayId
    #     grasp["grasp_id"] = total_grasps
    ray_direction = grasp["point_locations"][0] - grasp["point_locations"][1]
    # print("Ray direction:{}".format(ray_direction))

    # Make sure you compute the right distance here: it should always be between [0, Pi/2]
    # 3a. Compute the angular distance between the grasp axis and faceNormal1
    distance1_rad = angular_distance_in_rad(ray_direction, triangles_pierced_normals[0])
    # 3b. Compute the angular distance between the grasp axis and faceNormal2
    distance2_rad = angular_distance_in_rad(ray_direction, triangles_pierced_normals[1])

    # To normalise these values, divide by 90 degrees = (Pi/2) radians (max possible value)
    distance1_normalised = distance1_rad / (math.pi / 2.0)
    distance2_normalised = distance2_rad / (math.pi / 2.0)
    # Max value is 1 for the normalised angular distance

    # WARNING: Pi * radian = 180 degrees
    # 1 radian = (180 / Pi) ~= 57 degrees

    # 4. Sum the two distances, and return the result. (you can use sum of squares here)
    angular_distance = distance1_normalised**2 + distance2_normalised**2

    # Normalise the value (what is the maximum value possible?)
    #     v1 = [1,0,0]
    #     v2 = [-1,0,0]
    #     print(angular_distance_in_rad(v1, v2)) = 3.14... (Pi) = max value
    # In fact, the max distance between normal and grasp axis is 90 deg = Pi/2
    # Therefore, the max value here is: 2*(Pi/2)**2 = Pi**2 / 2
    angular_distance_normalised = angular_distance / 2.0
    # angular_distance_normalised = angular_distance / ((math.pi**2) / 2.0)

    # [HACK] the values we get are too low,
    # because it is rare to see the best grasp on an object
    # being at almost 90 degrees from the surface normal.
    # We assume here that it is at most
    # # angular_distance_denormalised = angular_distance_normalised * 2.0

    # Linearize a bit the error: (to better spread it over the [0,1] interval)
    # Holds any value between [0,1] within [0,1] still.
    # angular_distance_normalised_spread = np.sqrt(angular_distance_normalised)

    # return angular_distance_normalised_spread
    return angular_distance_normalised



### Metrics defined. Computing the metric scores for each mesh, and all grasps generated for it.
# 2020.01.09
# 1. Compute grasps for the given mesh, using TOTAL_RAYS_TO_USE
# 2. For each grasp, compute its grasp metrics
# 3. Compute the minimum score for each grasp metric, and return it.
# def compute_grasp_metrics(mesh, gripper, MAX_MESH_SIDE_LENGTH=0.2, TOTAL_RAYS_TO_USE=10, metric1_error_threshold, metric2_error_threshold):
def compute_grasp_metrics(mesh,
                          gripper,
                          MAX_MESH_SIDE_LENGTH,
                          TOTAL_RAYS_TO_USE,
                          percentile): # Score percentile to compute
                          # some meshes will have 0% grasps below this threshold
                          # metric1_error_threshold_centroid_distance=0.3,
                          # metric2_error_threshold_sqrt_angular_distance=0.44): # ang distance = 0.44^2 = 0.2
    # Metrics on grasp-list statistics gather general data about the object shape:
    # - how easy is it to grasp the object?

    try:
        grasps = mesh_to_grasps.mesh_to_grasps(
                    mesh,
                    gripper,
                    MAX_MESH_SIDE_LENGTH,
                    TOTAL_RAYS_TO_USE,
                    use_only_external_points=True)
    except mesh_to_grasps.ErrorComputingRayTriangleIntersection:
        # Failed to measure the grasp metrics of this mesh,
        print("Warning: Failed to measure the grasp metrics of this mesh. Returning NaN (to skip it)")
        result = {
            "percentile_value_centroid_distance" : math.nan,
            "percentile_value_angular_distance" : math.nan,
        }
        return result

    # Reminder:
    #     grasp = {}
    #     grasp["point_locations"] = (rays_intersection_points[rayId][startID], rays_intersection_points[rayId][endID])
    #     grasp["point_faces"] = (rays_intersection_faces[rayId][startID], rays_intersection_faces[rayId][endID])
    #     #
    #     # Additional info (just in case)
    #     grasp["ray_id"] = rayId
    #     grasp["grasp_id"] = total_grasps

    # Use the distance between the mesh centroid and the grasp contact centroid
    # Compute the grasp scores for all the grasps
    # grasp_scores = []

    grasps_scores_angular_distance = []
    grasps_scores_centroid_distance = []

    print("Grasps: {}".format(len(grasps)))
    for grasp in grasps:
        # Metric 1
        #grasp_score_centroid_distance = _getScoreCentroidDistance(mesh.centroid, grasp["point_locations"])
        grasp_score_centroid_distance = _getScoreCentroidDistance(mesh, grasp["point_locations"])
        # print("Grasp score: {}".format(getScoreCentroidDistance(mesh.centroid, grasp)))
        grasp_score_centroid_distance_spread = np.sqrt(np.sqrt(grasp_score_centroid_distance))

        # Metric 2
        # Apparently, the may be a division by zero in some rare cases,
        # when the product of the face_normals' norms is equal to 0.0
        # In this case, just drop this grasp
        try:
            grasp_score_angular_distance = angular_distance_graspAxis_vs_meshFaces(grasp, mesh)

            # (!) Ad-hoc measure, to better spread the values inside the [0,1] interval
            # https://docs.scipy.org/doc/numpy/reference/generated/numpy.sqrt.html
            grasp_score_angular_distance_spread = np.sqrt(np.sqrt(grasp_score_angular_distance))

            # grasp_scores.append((grasp_score_centroid_distance,grasp_score_angular_distance))
            # grasp_scores.append((grasp_score_centroid_distance_spread, grasp_score_angular_distance_spread))

            grasps_scores_centroid_distance.append(grasp_score_centroid_distance_spread)
            grasps_scores_angular_distance.append(grasp_score_angular_distance_spread)


        except ZeroDivisionError:
            print("Warning: Zero Division Error when computing grasp angular distance. Skipping grasp.")

    # Compute the max grasp score
    # print("Grasp scores ({}):".format(len(grasp_scores)))
    # grasp_scores = np.array(grasp_scores)
    grasps_scores_centroid_distance = np.array(grasps_scores_centroid_distance)
    grasps_scores_angular_distance = np.array(grasps_scores_angular_distance)

    print("grasps_scores_centroid_distance:")
    print(grasps_scores_centroid_distance)
    percentile_value_centroid_distance = np.percentile(grasps_scores_centroid_distance, percentile)
    percentile_value_angular_distance = np.percentile(grasps_scores_angular_distance, percentile)

    result = {
        "percentile_value_centroid_distance" : percentile_value_centroid_distance,
        "percentile_value_angular_distance" : percentile_value_angular_distance,
    }

    # Compute the minimum value for each column (each metric)
    print("Total grasps: {}\nTotal grasp scores: {}".format(len(grasps), len(grasps_scores_angular_distance)))
    # min_grasp_scores = np.amin(grasp_scores, axis=0)
    return result

def selectBestGrasp(mesh, grasps):
	"""
	Selects the best grasp among several,
	based on computed grasp metrics.
	"""

	grasps_scores_angular_distance = []
	grasps_scores_centroid_distance = []

	print("Grasps: {}".format(len(grasps)))
	for grasp in grasps:
		# Metric 1
		#grasp_score_centroid_distance = _getScoreCentroidDistance(mesh.centroid, grasp["point_locations"])
		grasp_score_centroid_distance = _getScoreCentroidDistance(mesh, grasp["point_locations"])
		# print("Grasp score: {}".format(getScoreCentroidDistance(mesh.centroid, grasp)))
		grasp_score_centroid_distance_spread = np.sqrt(np.sqrt(grasp_score_centroid_distance))

		# Metric 2
		# Apparently, the may be a division by zero in some rare cases,
		# when the product of the face_normals' norms is equal to 0.0
		# In this case, just drop this grasp
		try:
			grasp_score_angular_distance = angular_distance_graspAxis_vs_meshFaces(grasp, mesh)

			# (!) Ad-hoc measure, to better spread the values inside the [0,1] interval
			# https://docs.scipy.org/doc/numpy/reference/generated/numpy.sqrt.html
			grasp_score_angular_distance_spread = np.sqrt(np.sqrt(grasp_score_angular_distance))

			# grasp_scores.append((grasp_score_centroid_distance,grasp_score_angular_distance))
			# grasp_scores.append((grasp_score_centroid_distance_spread, grasp_score_angular_distance_spread))

			grasps_scores_centroid_distance.append(grasp_score_centroid_distance_spread)
			grasps_scores_angular_distance.append(grasp_score_angular_distance_spread)


		except ZeroDivisionError:
			print("Warning: Zero Division Error when computing grasp angular distance. Skipping grasp.")

	# Compute the max grasp score
	# print("Grasp scores ({}):".format(len(grasp_scores)))
	# grasp_scores = np.array(grasp_scores)
	# Intuiton: The smaller the distances, the better
	grasps_scores_centroid_distance = np.array(grasps_scores_centroid_distance)
	grasps_scores_angular_distance = np.array(grasps_scores_angular_distance)
	print(len(grasps_scores_centroid_distance))
	print(len(grasps_scores_angular_distance))
	print("Grasp scores: {0} centroid and {1} angular distances".format(len(grasps_scores_centroid_distance), len(grasps_scores_angular_distance)))

	# Compute the combined scores
	grasp_scores_combined = [grasps_scores_centroid_distance[i] * grasps_scores_angular_distance[i] for i in range(0,len(grasps_scores_angular_distance))]

	index_of_grasp_with_best_score = np.argmin(grasp_scores_combined)
	best_grasp = grasps[index_of_grasp_with_best_score]
	return best_grasp
