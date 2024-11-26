
from mapUtilities import *
from a_star import *

POINT_PLANNER=0; TRAJECTORY_PLANNER=1

class planner:
    def __init__(self, type_, mapName="room"):

        self.type=type_
        self.mapName=mapName

    
    def plan(self, startPose, endPose):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(endPose)
        
        elif self.type==TRAJECTORY_PLANNER:
            self.costMap=None
            self.initTrajectoryPlanner()
            return self.trajectory_planner(startPose, endPose)


    def point_planner(self, endPose):
        return endPose

    def initTrajectoryPlanner(self):


        # TODO PART 5 Create the cost-map, the laser_sig is 
        # the standard deviation for the gausiian for which
        # the mean is located on the occupant grid. 
        self.m_utilites=mapManipulator(laser_sig=0.3)
            
        self.costMap=self.m_utilites.make_likelihood_field()
        

    def trajectory_planner(self, startPoseCart, endPoseCart):


        # This is to convert the cartesian coordinates into the 
        # the pixel coordinates of the map image, remmember,
        # the cost-map is in pixels. You can by the way, convert the pixels
        # to the cartesian coordinates and work by that index, the a_star finds
        # the path regardless. 
        startPose=self.m_utilites.position_2_cell(startPoseCart)
        endPose=self.m_utilites.position_2_cell(endPoseCart)
        
        # TODO PART 5 convert the cell pixels into the cartesian coordinates
        path= search(self.costMap, startPose, endPose)
        pathCart = list(map(self.m_utilites.cell_2_position, path))

        # Plot the costmap and path
        import matplotlib.pyplot as plt
        
        plt.figure(figsize=(10,10))
        plt.imshow(self.costMap, cmap='gray')
        
        # Extract x and y coordinates from path for plotting
        path_x = [p[0] for p in path]
        path_y = [p[1] for p in path]
        
        # Plot path on top of costmap
        plt.plot(path_x, path_y, 'r-', linewidth=2, label='Planned Path')
        plt.scatter([startPose[0]], [startPose[1]], c='g', marker='o', s=100, label='Start')
        plt.scatter([endPose[0]], [endPose[1]], c='r', marker='x', s=100, label='Goal')
        
        plt.colorbar(label='Cost')
        plt.legend()
        plt.title('Costmap with Planned Path')
        plt.xlabel('X (pixels)')
        plt.ylabel('Y (pixels)') 
        plt.show()


        # TODO PART 5 return the path as list of [x,y]
        return pathCart




if __name__=="__main__":

    m_utilites=mapManipulator()
    
    map_likelihood=m_utilites.make_likelihood_field()

    # you can use this part of the code to test your 
    # search algorithm regardless of the ros2 hassles
    
