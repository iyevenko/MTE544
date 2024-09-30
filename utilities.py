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

            #read values in list to string

            for i in list:
                vals_str+= str(i)+','
            
            vals_str+="\n"

            #write string to file
            
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
            # Skip the header line

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


# TODO Part 5: Implement the conversion from Quaternion to Euler Angles
def euler_from_quaternion(quat):
    qx= quat[0]
    qy = quat[1]
    qz = quat[2]
    qw = quat[3]

    roll = atan2(2*(qw*qx+qy*qz),1-2*(qx^2+qy^2))
    pitch = -M_PI + 2*atan2(sqrt(1+2*(qw*qy-qx*qz)),sqrt(1-2*(qw*qy-qx*qz)))
    yaw = atan2(2*(qw*qz+qx*qy),1-2*(qy^2+qz^2))
    ... # just unpack yaw
    return yaw


