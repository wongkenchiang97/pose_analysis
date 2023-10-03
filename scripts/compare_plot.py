#!/usr/bin python3 
# import the required library
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns 
import rospy
import rospkg
from std_msgs.msg import Bool

class CompareVisualizer:
    
    def __init__(self):
        rospy.init_node('compare_plot')
        rospy.loginfo("pose error plot started.")
        self.field_name = rospy.get_param('field_name')
        rospack = rospkg.RosPack()
        self.time_series_pose_error_dir = rospack.get_path('pose_analysis')+"/data/analysis/time_series_compare_data.csv"
        self.showPoseAnalysisResult()

    def __del__(self):
        plt.close('all')

    def showPoseAnalysisResult(self):
        print("showing pose error result...")
        self.time_series_data = pd.read_csv(self.time_series_pose_error_dir)

        box_rgb = {"x_error(m)":"r","y_error(m)":"g","yaw_error(°)":"b"}

        plt.figure("Histogram Plot")
        sns.set(style="darkgrid")
        sns.histplot(self.time_series_data, x=" yaw_error(°)",
                    stat="percent",
                    hue=self.field_name,
                    element="step")
        plt.title("Rotation Error Distribution",size=20)

        sns.set(style="darkgrid")
        g = sns.JointGrid(data=self.time_series_data
                        ,x=self.time_series_data[' x_error(m)']
                        ,y=self.time_series_data[' y_error(m)']
                        ,hue=self.field_name
                        ,marginal_ticks='True')
        g.plot_marginals(sns.histplot,stat='percent',kde='True')
        g.plot_joint(sns.kdeplot, color="b",fill='True')
        g.plot_joint(sns.scatterplot,s=20)
        g.figure.suptitle('Translation error distribution',y=1)

        plt.show()

if __name__ == '__main__':

    pose_error_visualizer = CompareVisualizer()
    rospy.spin()