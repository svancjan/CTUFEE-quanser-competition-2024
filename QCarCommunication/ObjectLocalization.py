import numpy as np

class ObjectLokalization:
    def __init__(self, maximalLocalisationDistance):
        self.maximalLocalisationDistance_ = maximalLocalisationDistance
        
        self.objectLocations_ = None
        self.objectVerifications_ = None
        self.objectIsDriveEnabled_ = None
        
        self.min_dist = None
        self.minarg_dist = None
        
    def localize(self, vehiclePosition, objectPosition, isDriveEnabled):
        if objectPosition is not None:
            if self.objectLocations_ is None:
                self.objectLocations_ = np.array([objectPosition])
                self.objectVerifications_ = np.array([False])
                self.objectIsDriveEnabled_ = np.array([False])
            else:
                if (np.linalg.norm(vehiclePosition-objectPosition) < self.maximalLocalisationDistance_):
                    # Get closest object from the list to new object
                    distances = np.linalg.norm(self.objectLocations_ - objectPosition, axis = 1)
                    min_dist = np.min(distances)
                    minarg_dist = np.argmin(distances)
                    # If there is close object, verify it, and adjust position, otherwise add new object to the list
                    if min_dist < 1:
                        self.objectVerifications_[minarg_dist] = True
                        self.objectLocations_[minarg_dist] = (objectPosition+self.objectLocations_[minarg_dist])/2
                        if isDriveEnabled is not None:
                            self.objectIsDriveEnabled_[minarg_dist] = isDriveEnabled
                    else:
                        self.objectLocations_ = np.vstack([self.objectLocations_,objectPosition])
                        self.objectVerifications_ = np.append(self.objectVerifications_, False)
                        if isDriveEnabled is not None:
                            self.objectIsDriveEnabled_ = np.append(self.objectIsDriveEnabled_, isDriveEnabled)
                        else:
                            self.objectIsDriveEnabled_ = np.append(self.objectIsDriveEnabled_, False)
    
    def getClosest(self, vehicle_position):
        if self.objectLocations_ is not None and self.objectVerifications_ is not None:
            distances = np.linalg.norm(self.objectLocations_[self.objectVerifications_] - vehicle_position, axis = 1)
            if (len(distances) > 0):
                self.min_dist = np.min(distances)
                self.minarg_dist = np.argmin(distances)
                return (self.min_dist, self.objectIsDriveEnabled_[self.objectVerifications_][self.minarg_dist])
            
    def setClosestState(self, isDriveEnabled):
        tmp = self.objectIsDriveEnabled_[self.objectVerifications_]
        tmp[self.minarg_dist] = isDriveEnabled
        self.objectIsDriveEnabled_[self.objectVerifications_] = tmp