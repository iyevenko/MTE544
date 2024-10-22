from math import atan2, asin, sqrt

M_PI=3.1415926535

class Logger:
    
    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):
        
        self.filename = filename

        with open(self.filename, 'w') as file:
            
            header_str=""

            for header in headers:
                header_str+=header
                header_str+=", "
            
            header_str+="\n"
            
            file.write(header_str)


    def log_values(self, values_list):

        with open(self.filename, 'a') as file:
            
            vals_str=""
            
            for value in values_list:
                vals_str+=f"{value}, "
            
            vals_str+="\n"
            
            file.write(vals_str)
            

    def save_log(self):
        pass

class FileReader:
    def __init__(self, filename):
        
        self.filename = filename
        
        
    def read_file(self):
        
        read_headers=False

        table=[]
        headers=[]
        with open(self.filename, 'r') as file:

            if not read_headers:
                for line in file:
                    values=line.strip().split(',')

                    for val in values:
                        if val=='':
                            break
                        headers.append(val.strip())

                    read_headers=True
                    break
            
            next(file)
            
            # Read each line and extract values
            for line in file:
                values = line.strip().split(',')
                
                row=[]                
                
                for val in values:
                    if val=='':
                        break
                    row.append(float(val.strip()))

                table.append(row)
        
        return headers, table
    
    

# TODO Part 3: Implement the conversion from Quaternion to Euler Angles
def euler_from_quaternion(quat):
    #gets the variables from the quaternion vector
    qx= quat[0]
    qy = quat[1]
    qz = quat[2]
    qw = quat[3]
    
    #calculates roll, pitch and yaw euler angles
    roll = atan2(2*(qw*qx+qy*qz),1-2*(qx**2+qy**2))
    pitch = -M_PI + 2*atan2(sqrt(1+2*(qw*qy-qx*qz)),sqrt(1-2*(qw*qy-qx*qz)))
    yaw = atan2(2*(qw*qz+qx*qy),1-2*(qy**2+qz**2))
    #returns yaw to odom_log function
    return yaw


#TODO Part 4: Implement the calculation of the linear error
def calculate_linear_error(current_pose, goal_pose):
        
    # Compute the linear error in x and y
    # Remember that current_pose = [x,y, theta, time stamp] and goal_pose = [x,y]
    # Remember to use the Euclidean distance to calculate the error.
    error_x = current_pose[0]-goal_pose[0]
    error_y = current_pose[1]-goal_pose[1]
    error_linear= sqrt(error_x**2 + error_y**2)

    return error_linear

#TODO Part 4: Implement the calculation of the angular error
def calculate_angular_error(current_pose, goal_pose):

    # Compute the linear error in x and y
    # Remember that current_pose = [x,y, theta, time stamp] and goal_pose = [x,y]
    # Use atan2 to find the desired orientation
    # Remember that this function returns the difference in orientation between where the robot currently faces and where it should face to reach the goal
    error_x = current_pose[0]-goal_pose[0]
    error_y = current_pose[1]-goal_pose[1]
    error_angular = atan2(error_y,error_x)

    # Remember to handle the cases where the angular error might exceed the range [-π, π]
    if error_angular > M_PI:
        error_angular = error_angular % (2*M_PI) - 2*M_PI
    elif error_angular < -M_PI:
        error_angular = error_angular % (2*M_PI) + 2*M_PI
    
    return error_angular
