import rclpy

from rclpy.node import Node
from std_msgs.msg import String,UInt32,UInt32MultiArray,Float32MultiArray
from math import sqrt


from rclpy.parameter import Parameter


class Calculate_node(Node):
    def __init__(self,name):
        super().__init__(name)

        self.corrdinate_pub = self.create_publisher(UInt32MultiArray,'uav1_obj_position',10)
        self.position_sub = self.create_subscription(Float32MultiArray,'uav_position',self.uav1_position_process,10)

        self.z_height = 10
        self.obj_radius = 0.5   # uav arrive at position radius 无人机到达的半径范围，在这里区域内则认为到了设定点
        self.uav1_position = [] # now uav1 position corrdinate
        
        self.declare_parameter("obj_position",[20, 0, 5, 0, 5, 5, 20, 20, 5, 0, 0, 20, 0, 0, 1])

        self.uav1_obj_position = [[0,0,1]] #uav1 obj position corrdinate

        self.last_hj_point = [[0,0,0]]

        # self.uav1_obj_position = self.get_parameter('obj_position').get_parameter_value()
        # self.uav1_obj_position = self.obj_position_to_uav_obj_position()

        self.uav1_hj_point = len(self.uav1_obj_position)
        self.i =0


        self.timer = self.create_timer(1,self.timer_callback)
        self.timer2 = self.create_timer(2,self.timer_callback2)


    """
    def uav1_position_process(self,position):
    用来处理获得到的无人机的位置信息
    将位置信息存储到 uav1_position 
    """
    def uav1_position_process(self,position):
        self.uav1_position = position.data
        #print(self.uav1_position)
        #self.get_logger().info('订阅到无人机1的位置')

#-----------------------------------------------------------------------
#简单的一个分割，将多个目标点分割出来为单组的数据。
#-----------------------------------------------------------------------
    def obj_position_to_uav_obj_position(self):

        #self.get_logger().info('接受无人机1外部坐标参数设置')
        corr = self.get_parameter('obj_position').get_parameter_value().integer_array_value
        #print(corr)
        obj_cor_list = []
        if len(corr)%3 == 0:
            m = 0
            while m < len(corr):
                one_corr = [corr[m],corr[m+1],corr[m+2]]
                m+=3
                obj_cor_list.append(one_corr)
        else:
            self.get_logger().info('坐标数量输入数量不对 请保持x,y,z为一组')
        return obj_cor_list
    
#-----------------------------------------------------------------------
#这个是我现在不懂的部分！！！
#-----------------------------------------------------------------------

    def timer_callback2(self):
        # last_hj_point = [[0,0,0]]
        new_hj_point = self.obj_position_to_uav_obj_position()
        print('航迹点: ',new_hj_point) #[20, 0, 5, 0, 5, 5, 20, 20, 5, 0, 0, 20, 0, 0, 1]
        if self.last_hj_point  == new_hj_point:
            pass
        else:
            self.get_logger().info('更新无人机航迹点')
            self.uav1_obj_position = new_hj_point
            print(new_hj_point)
            self.uav1_hj_point = len(self.uav1_obj_position)
            self.i = 0
            self.last_hj_point  = new_hj_point
        pass

#-----------------------------------------------------------------------
#-----------------------------------------------------------------------
    """
     def timer_callback(self):
     
    通过计算现在位置与设定位置的距离，判断是否到达设定点
    若到达 则i+1 目标点更新
    """
    def timer_callback(self):
        # self.uav1_obj_position = self.obj_position_to_uav_obj_position()
        # self.uav1_hj_point = len(self.uav1_obj_position)

        if len(self.uav1_position)!=0:
            s = (self.uav1_obj_position[self.i][0]+self.uav1_position[0])**2 + (self.uav1_obj_position[self.i][1]+self.uav1_position[1])**2 + (self.uav1_obj_position[self.i][2]+self.uav1_position[2])**2
            oula_instance = sqrt(s)
            print('距离设定点距离: ',oula_instance)
            if oula_instance < self.obj_radius:
                if self.i < self.uav1_hj_point-1:
                    self.i +=1
                    print("到达地 %d 个设定点"%self.i)
                else:
                    pass

            msg = UInt32MultiArray()
            msg.data = self.uav1_obj_position[self.i]
            self.corrdinate_pub.publish(msg)


            # a = self.obj_position_to_uav_obj_position()
            # print(a)

            if self.i >= self.uav1_hj_point-1 and oula_instance < 0.5:
                 self.i = 0
            	


        

        







def main(args = None):

    rclpy.init(args=args)

    node = Calculate_node('calcluate')

    rclpy.spin(node)
    rclpy.shutdown()






