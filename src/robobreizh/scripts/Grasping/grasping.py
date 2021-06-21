#!/usr/bin/env python
# -*- coding: utf-8 -*-

import trimesh
import mesh_to_grasps
import grasp_metrics
import numpy as np
import rtree 
import glob
import sys
import os
from scipy.spatial import cKDTree
from scipy import stats
from open3d_ros_helper import open3d_ros_helper as orh
import open3d as o3d

class ProcessPCD(object):

	def use_ransac(self, points):
		o3dpc = orh.rospc_to_o3dpc(points) 

		plane_model, inliers = o3dpc.segment_plane(distance_threshold=0.01,
                                         ransac_n=3,
                                         num_iterations=1000)
		[a, b, c, d] = plane_model
		print("Plane equation: {}x + {}y + {}z + {} = 0".format(a, b, c, d))

		inlier_cloud = o3dpc.select_by_index(inliers)
		inlier_cloud.paint_uniform_color([1.0, 0, 0])
		outlier_cloud = o3dpc.select_by_index(inliers, invert=True)
		o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
		                                  zoom=0.8,
		                                  front=[-0.4999, -0.1659, -0.8499],
		                                  lookat=[2.1813, 2.0619, 2.0999],
		                                  up=[0.1204, -0.9852, 0.1215])

	def statistical_outilier_removal(self, kdtree, k=8, z_max=2 ):
		""" Compute a Statistical Outlier Removal filter on the given KDTree.

		Parameters
		----------                        
		kdtree: scipy's KDTree instance
		    The KDTree's structure which will be used to
		    compute the filter.
		    
		k(Optional): int
		    The number of nearest neighbors wich will be used to estimate the 
		    mean distance from each point to his nearest neighbors.
		    Default : 8
		    
		z_max(Optional): int
		    The maximum Z score wich determines if the point is an outlier or 
		    not.
		    
		Returns
		-------
		sor_filter : boolean array
		    The boolean mask indicating wherever a point should be keeped or not.
		    The size of the boolean mask will be the same as the number of points
		    in the KDTree.
		    
		Notes
		-----    
		The 2 optional parameters (k and z_max) should be used in order to adjust
		the filter to the desired result.

		A HIGHER 'k' value will result(normally) in a HIGHER number of points trimmed.

		A LOWER 'z_max' value will result(normally) in a HIGHER number of points trimmed.

		"""

		distances, i = kdtree.query(kdtree.data, k=k, n_jobs=-1) 

		z_distances = stats.zscore(np.mean(distances, axis=1))

		sor_filter = abs(z_distances) < z_max

		return sor_filter


	def PCA(self, data, correlation = False, sort = True):
		""" Applies Principal Component Analysis to the data
		Parameters
		----------        
		data: array
		    The array containing the data. The array must have NxM dimensions, where each
		    of the N rows represents a different individual record and each of the M columns
		    represents a different variable recorded for that individual record.
		        array([
		        [V11, ... , V1m],
		        ...,
		        [Vn1, ... , Vnm]])

		correlation(Optional) : bool
		        Set the type of matrix to be computed (see Notes):
		            If True compute the correlation matrix.
		            If False(Default) compute the covariance matrix. 
		            
		sort(Optional) : bool
		        Set the order that the eigenvalues/vectors will have
		            If True(Default) they will be sorted (from higher value to less).
		            If False they won't.   
		Returns
		-------
		eigenvalues: (1,M) array
		    The eigenvalues of the corresponding matrix.
		    
		eigenvector: (M,M) array
		    The eigenvectors of the corresponding matrix.

		Notes
		-----
		The correlation matrix is a better choice when there are different magnitudes
		representing the M variables. Use covariance matrix in other cases.

		"""

		mean = np.mean(data, axis=0)

		data_adjust = data - mean

		#: the data is transposed due to np.cov/corrcoef syntax
		if correlation:
		    
		    matrix = np.corrcoef(data_adjust.T)
		    
		else:
		    matrix = np.cov(data_adjust.T) 

		eigenvalues, eigenvectors = np.linalg.eig(matrix)

		if sort:
		    #: sort eigenvalues and eigenvectors
		    sort = eigenvalues.argsort()[::-1]
		    eigenvalues = eigenvalues[sort]
		    eigenvectors = eigenvectors[:,sort]

		return eigenvalues, eigenvectors

	def best_fitting_plane(self, points, equation=False):
		""" Computes the best fitting plane of the given points

		Parameters
		----------        
		points: array
		    The x,y,z coordinates corresponding to the points from which we want
		    to define the best fitting plane. Expected format:
		        array([
		        [x1,y1,z1],
		        ...,
		        [xn,yn,zn]])
		        
		equation(Optional) : bool
		        Set the oputput plane format:
		            If True return the a,b,c,d coefficients of the plane.
		            If False(Default) return 1 Point and 1 Normal vector.    
		Returns
		-------
		a, b, c, d : float
		    The coefficients solving the plane equation.

		or

		point, normal: array
		    The plane defined by 1 Point and 1 Normal vector. With format:
		    array([Px,Py,Pz]), array([Nx,Ny,Nz])
		    
		"""

		w, v = self.PCA(points)

		#: the normal of the plane is the last eigenvector
		normal = v[:,2]
		   
		#: get a point from the plane
		point = np.mean(points, axis=0)


		if equation:
		    a, b, c = normal
		    d = -(np.dot(normal, point))
		    return a, b, c, d
		    
		else:
		    return point, normal

class Grasping(object):
	def __init__(self):
		self.models = {}
		BASE_DIR = "/home/maelic/Documents/Object_Detection/ycb-tools/models/ycb"
		sys.path.append(BASE_DIR)

		for model in glob.glob(os.path.join(BASE_DIR, "*/")):
			self.models[model.split("/")[-2]] = model+"tsdf/nontextured.stl"

	def load_model(self, object_name):

		mesh_filename = self.models[object_name]
		mesh = trimesh.load(mesh_filename, use_embree=False)
		# mesh_areas.append(mesh.area) 
		# mesh_volumes.append(mesh.volume)
		mesh.show()

		return mesh

	def compute_grasping_point(self, object_name):
		mesh = self.load_model(object_name)

		GRIPPER = {
		    # Define the size of the gripper fingers (thickness in X&Y)
		    "fingertip_x": 0.01,
		    "fingertip_y": 0.01,
		    "palm_depth": 0.05, #0.07, # depth between fingers (NOT YET TAKEN INTO ACCOUNT)
		    "width": 0.07
		}

		try:
		    # def mesh_to_grasps(mesh, gripper, MAX_MESH_SIDE_LENGTH=0.2, TOTAL_RAYS_TO_USE=512, use_only_external_points=True)
		    grasps = mesh_to_grasps.mesh_to_grasps(
		            mesh, 
		            GRIPPER, 
		            1.0, 
		            256, 
		            True)
		    print(np.array(grasps))

		except mesh_to_grasps.ErrorComputingRayTriangleIntersection:
		    # Failed to measure the grasp metrics of this mesh,
		    print("Warning: Failed to measure the grasp metrics of this mesh. Returning NaN (to skip it)")


		mesh_triangles = np.array(mesh.vertices[mesh.faces]) # Create triplets of triangle points
		face_centroids = np.array([np.mean(triangle, axis=0) for triangle in mesh_triangles])
		# http://jwilson.coe.uga.edu/EMAT6680/Dunbar/Assignment4/Assignment4_KD.htm

		### Compute the rays: origins and directions (using face normals)
		ray_origins = [grasps[i]["point_locations"][0] for i in range(0, len(grasps))]
		ray_directions = [grasps[i]["point_locations"][1]*2 for i in range(0, len(grasps))]

		### Visualise the rays
		# stack rays into line segments for visualization as Path3D
		# ray_visualize = trimesh.load_path(np.hstack((ray_origins, ray_origins+ray_directions)).reshape(-1, 2, 3))
		ray_visualize = trimesh.load_path(np.hstack((ray_origins, ray_directions)).reshape(-1, 2, 3))
		scene = trimesh.Scene([mesh, ray_visualize])
		# show the visualization
		scene.show()


		TOTAL_RAYS_TO_USE = 256
		MAX_MESH_SIDE_LENGTH = 1.0

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


		### Visualise the rays
		# stack rays into line segments for visualization as Path3D
		ray_visualize = trimesh.load_path(np.hstack((ray_origins,
		                                             ray_origins+ray_directions)).reshape(-1, 2, 3))
		scene = trimesh.Scene([mesh, ray_visualize])
		# show the visualization
		scene.show()

		grasp = grasp_metrics.selectBestGrasp(mesh, grasps)

		### Compute the rays: origins and directions (using face normals)
		ray_origins = grasp["point_locations"][0] - grasp["point_locations"][1]
		ray_directions = grasp["point_locations"][1] * 3.0

		### Visualise the rays
		# stack rays into line segments for visualization as Path3D
		# ray_visualize = trimesh.load_path(np.hstack((ray_origins, ray_origins+ray_directions)).reshape(-1, 2, 3))
		ray_visualize = trimesh.load_path(np.hstack((ray_origins, ray_directions)).reshape(-1, 2, 3))
		scene = trimesh.Scene([mesh, ray_visualize])
		# show the visualization
		scene.show()


if __name__ == "__main__":
	g = Grasping()
	g.compute_grasping_point("008_pudding_box")