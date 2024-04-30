import numpy as np
from vehicle_model import vehicle_position, KinematicVehicleModel
from path_creation import global_path, path


class shadow_vehicle:
    def __init__(self,vehicle,GlobalPath):    
        self.v_pose = vehicle.position
        self.path = GlobalPath.path
        self.v_refs = GlobalPath.v_ref
        self.path_len = self.path.x.shape[0]
        self.shadow_vehicle = vehicle_position(self.path.x[0],self.path.y[0],self.path.psi[0])
        self.HeadingErr = self.shadow_vehicle.psi - vehicle.position.psi
        self.HeadingErrLA = self.shadow_vehicle.psi - vehicle.position.psi
        self.xte = 0
        self.idx = 0
        self.v_ref = 0

    def get_shadow_position(self,vehicle):        
        self.v_pose = vehicle.position
        path_xy = np.array([self.path.x,self.path.y])
        path_len = path_xy.shape[1]
        vehicle_xy = np.array([[self.v_pose.x], [self.v_pose.y]])
        pose_mat = np.tile(vehicle_xy, (1, path_len))
        
        diff_mat = path_xy - pose_mat
        dists = np.sqrt(np.sum(diff_mat**2, axis=0))
        min_idx = np.argmin(dists)
     
        if(min_idx == 0):
            close_pts = np.stack((path_xy[:,min_idx],path_xy[:,min_idx+1]))
            max_iter = 1
            idxs = np.array([min_idx, min_idx+1, min_idx])
        elif(min_idx == path_len-1):
            close_pts = np.stack((path_xy[:,min_idx-1],path_xy[:,min_idx]))
            max_iter = 1
            idxs = np.array([min_idx-1, min_idx, min_idx-1])
        else:
            close_pts = np.stack((path_xy[:,min_idx-1],path_xy[:,min_idx],path_xy[:,min_idx+1]))
            max_iter = 2
            idxs = np.array([min_idx-1, min_idx, min_idx+1, min_idx-1, min_idx])
        
        norms = np.full((max_iter*2+1, 1), np.inf)
        pts = np.zeros((max_iter*2+1, 2))
        psis = self.path.psi[idxs]
        
        pts[0:max_iter+1,:] = close_pts
        norms[0:max_iter+1] = np.reshape(np.sqrt(np.sum((close_pts - np.tile(vehicle_xy.T, (max_iter+1, 1)))**2,axis=1)),(norms[0:max_iter+1].shape))
       
        for i in range(max_iter):
            b = (close_pts[i+1, :] - close_pts[i, :])
            a = (np.reshape(vehicle_xy,close_pts[i, :].shape) - close_pts[i, :])
            
            apn = np.dot(a,b)/np.dot(b,b)
       
            ap = apn * b
            pts[max_iter+1+i, :] = ap + close_pts[i, :]
            ar = a - ap
            if apn >= 0 and apn <= 1:
                norms[max_iter+1+i] = np.linalg.norm(ar)
      
    
        min_pt_i = np.argmin(norms)
        shadow_pose = pts[min_pt_i, :]
        self.shadow_vehicle.x = shadow_pose[0]
        self.shadow_vehicle.y = shadow_pose[1]
        self.shadow_vehicle.psi = psis[min_pt_i]
        self.idx = idxs[min_pt_i]  
        if(self.v_refs is not None):
            self.v_ref = self.v_refs[self.idx]
        else:
            self.v_ref = 0.5

    def crossTrackCalc(self,vehicle):
        #print('here')
        self.v_pose = vehicle.position
        v_vec = np.array((self.v_pose.x,self.v_pose.y))
        s_vec = np.array((self.shadow_vehicle.x,self.shadow_vehicle.y))
        
        # vector pointing from shadow vehicle to real vehicle
        diff_vec = v_vec - s_vec
        
        # heading of shadow vehicle
        psi_s = self.shadow_vehicle.psi

        #print(psi_s)
        R = np.array([[np.cos(psi_s), -np.sin(psi_s)],[np.sin(psi_s), np.cos(psi_s)]])
        Ri = R.T

        # rotate diff vec around shadow vehicle
        dist_vec = Ri @ diff_vec

        # get cross track error as y coordinate of dist_vec multiplied by -1
        cross_track = -dist_vec[1]
        self.xte = cross_track
        #print(cross_track)
    
    def headingErrCalc(self,vehicle):
        self.v_pose = vehicle.position
        error = self.shadow_vehicle.psi - self.v_pose.psi
        aux_err = np.array([error, error - 2 * np.pi, error + 2 * np.pi])
        abs_aux_err = abs(aux_err)
        min_abs_err = np.min(abs_aux_err)
        min_i = np.argmin(abs_aux_err)
        error = aux_err[min_i]
        
        if(abs(error) < 10.0 * (np.pi/12)):
            if(abs(self.HeadingErr - error) > np.pi/2):
                error = error - np.sign(error) * 2 * np.pi
           
        e_out = error
        #print(e_out)
        self.HeadingErr = e_out

    def headingErrCalcLA(self,vehicle,velocity,LAconst):
        self.v_pose = vehicle.position
        LA_index = min(self.idx + LAconst,self.path_len-1)
        LApsi = self.path.psi[LA_index]
        error = LApsi - self.v_pose.psi
        aux_err = np.array([error, error - 2 * np.pi, error + 2 * np.pi])
        abs_aux_err = abs(aux_err)
        min_abs_err = np.min(abs_aux_err)
        min_i = np.argmin(abs_aux_err)
        error = aux_err[min_i]
        
        if(abs(error) < 10.0 * (np.pi/12)):
            if(abs(self.HeadingErrLA - error) > np.pi/2):
                error = error - np.sign(error) * 2 * np.pi
           
        e_out = error
        #print(e_out)
        self.HeadingErrLA = e_out
    
    def updatePath(self, LocalPath):
        self.path.x = LocalPath[:,0]
        self.path.y = LocalPath[:,1]
        self.path.psi = LocalPath[:,2]
        self.path.d = 0
        self.v_refs = None
        self.path_len = len(self.path.x)
            
