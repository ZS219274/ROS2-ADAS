import rclpy
from rclpy.node import Node
import numpy as np 
import matplotlib.pyplot as plt


class PlotData(Node):
    def __init__(self):
        super().__init__('plot_data')
        self.get_logger().info('Starting plot node')
        

def main():
    rclpy.init()
    plot_node = PlotData()
    
    try:
        rclpy.spin(plot_node)
    except KeyboardInterrupt:
        plot_node.get_logger().info('Keyboard Interrupt by user')
    finally:
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
