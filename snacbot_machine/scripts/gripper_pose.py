import numpy as np
import cv2

class gripper_pose
    #RETURNS:   The distance of a point (x,y,z) from a plane (Ax+By+C=z)
    #REQUIRES:  x, y, and z are values of a coordinate (x,y,z) and A,B,C are the
    #           coefficients of a plane Ax+By+C=z
    def dist_to_plane(self,x,y,z,A,B,C):
        return abs(-A*x-B*y+z-C)/np.sqrt(pow(A,2)+pow(B,2)+1)

    #RETURNS:   Distance of plane from camera at the center of object, pixel coordinate of object in frame, and angle orientation of object
    #REQUIRES:  depth_map is a (HxWx1) array of int16 where H and W are the EXACT
    #           DIMENSIONS of the RGB image as well, and bounding_box is not equal to None.
    def pose_estimate(self, bounding_box, depth):
        #Extract valuable info from the bounding box input
        x1=bounding_box[0]
        y1=bounding_box[1]
        x2=bounding_box[2]
        y2=bounding_box[3]
        
        #Crop the array by the bounding box
        depth_arr = depth_arr[y1:y2,x1:x2]
        
        #Find nonzero-valued coordinate pairs (x,y,depth_arr[y,x]) and generate a plane        
        shape = depth_arr.shape
        y_lim = shape[1]
        x_lim = shape[0]
        
        #Values at the edges of the image. Keep in mind: y axis is along the top edge of the image, x axis along left edge
        top_edge = depth_arr[0,:]
        left_edge = depth_arr[:,0]
        bottom_edge = depth_arr[shape[0]-1,:]
        right_edge = depth_arr[:,shape[1]-1]
        
        #Create a set of coordinates to fit a plane to
        plane_coords = set()
        thresh = 20 # If the value of the edge cell is over this threshold, its coordinate will be counted on the plane.
        for y in range(y_lim):
            if top_edge[y] > thresh:
                plane_coords.add((0,y,top_edge[y]))
            if bottom_edge[y] > thresh:
                plane_coords.add((x_lim - 1,y,bottom_edge[y]))
        for x in range(x_lim):
            if left_edge[x] > thresh:
                plane_coords.add((x,0,left_edge[x]))
            if right_edge[x] > thresh:
                plane_coords.add((x,y_lim - 1,right_edge[x]))
        
        #Fit a plane to the set of points on the edge of the bounding box
        A = np.matrix(list(plane_coords))
        A[:,2]=1
        B = np.matrix(np.array(list(plane_coords))[:,2]).T
        X = (A.T*A).I*A.T*B
        
        #ax+by+c=z
        a = X[0,0]
        b = X[1,0]
        c = X[2,0]
        
        #Create the object mask
        bool_array = np.full((x_lim, y_lim), False)
        height_thresh = 10
        for x in range(x_lim):
            for y in range(y_lim):
                dist = self.dist_to_plane(x,y,depth_arr[x,y],a,b,c)
                if depth_arr[x,y] > thresh and dist > height_thresh:
                    bool_array[x,y] = True
                    
        #Get coordinates of nonzero elements in bool array
        p_x,p_y=np.nonzero(bool_array)
        
        #center of mass
        c_x = (x2-x1)/2
        c_y = (y2-y1)/2
        
        #arrays of points
        p_x = p_x - np.mean(p_x)
        p_y = p_y - np.mean(p_y)
        coords = np.vstack([p_x, p_y])
        
        #Find a vector in the direction of the object length wise using covariance matrix and eigenvectors/values
        cov = np.cov(coords)
        evals, evecs = np.linalg.eig(cov)
        sort_indices = np.argsort(evals)[::-1]
        y_v1, x_v1 = evecs[:, sort_indices[0]]
        
        #Return: depth of plane at (c_x,x_y), coord of object center relative to image top right corner , image orientation
        depth_of_plane_at_obj = a*c_x+b*c_y+c
        center_of_obj_in_image_coords = (x1+int(np.round(c_x)), y1+int(np.round(c_y)))
        theta_of_obj_to_image_x = np.arctan2(y_v1, x_v1)
        return (depth_of_plane_at_obj, center_of_obj_in_image_coords, theta_of_obj_to_image_x)