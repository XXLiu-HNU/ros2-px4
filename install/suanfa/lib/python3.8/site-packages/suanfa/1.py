import rclpy

from rclpy.node import Node
from std_msgs.msg import String,UInt32,UInt32MultiArray,Float32MultiArray
from math import sqrt



class Calculate_node(Node):
    def __init__(self,name):
        super().__init__(name)

        self.corrdinate_pub = self.create_publisher(UInt32MultiArray,'uav1_obj_position',10)
        self.position_sub = self.create_subscription(Float32MultiArray,'uav_position',self.uav1_position_process,10)

        self.z_height = 10
        self.obj_radius = 1.0   # uav arrive at position radius
        self.uav1_position = [] # now uav1 position corrdinate
        self.uav1_obj_position = [[10,0,5],[10,5,5],[6,5,5],[6,0,5],[0,0,1]] #uav1 obj position corrdinate

        self.uav1_hj_point = len(self.uav1_obj_position)
        self.i =0


        self.timer = self.create_timer(1,self.timer_callback)

    def uav1_position_process(self,position):
        self.uav1_position = position.data
        print(self.uav1_position)
        self.get_logger().info('订阅到无人机1的位置')


    def timer_callback(self):

        if len(self.uav1_position)!=0:
            s = (self.uav1_obj_position[self.i][0]+self.uav1_position[0])**2 + (self.uav1_obj_position[self.i][1]+self.uav1_position[1])**2 + (self.uav1_obj_position[self.i][2]+self.uav1_position[2])**2
            oula_instance = sqrt(s)

            if oula_instance < self.obj_radius:
                if self.i < self.uav1_hj_point-1:
                    self.i +=1
                else:
                    #self.i = 0
                    pass
            msg = UInt32MultiArray()
            msg.data = self.uav1_obj_position[self.i]
            self.get_logger().info('发送无人机1目标位置')
            print(msg.data)
            self.corrdinate_pub.publish(msg)
            if self.i >= self.uav1_hj_point-1 and oula_instance < 0.5:
                self.i = 0
            	


        

        







def main(args = None):

    rclpy.init(args=args)

    node = Calculate_node('calcluate')

    rclpy.spin(node)
    rclpy.shutdown()





