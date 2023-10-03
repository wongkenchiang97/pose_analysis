#!/usr/bin python3 
# import the required library
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns 
import rospy
import rospkg
from std_msgs.msg import Bool

class PoseAnalysisVisualizer:
    
    def __init__(self):
        rospy.init_node('pose_error_visualizer')
        self.show_error_flag_sub = rospy.Subscriber("/show_error_flag",Bool,self.flag_callback)
        rospack = rospkg.RosPack()
        self.time_series_pose_error_dir = rospack.get_path('pose_analysis')+"/data/analysis/time_series_pose_error_data.csv"

    def __del__(self):
        plt.close('all')

    def flag_callback(self,msg):
        if msg.data == True:
            self.showPoseAnalysisResult()

    def showPoseAnalysisResult(self):
        print("showing pose error result...")
        self.time_series_data = pd.read_csv(self.time_series_pose_error_dir)

        plt.figure("Histogram Plot")
        sns.set(style="darkgrid")
        sns.histplot(self.time_series_data, x=" yaw_error(°)",
                    stat="percent",
                    element="step")
        plt.title("Rotation Error Distribution",size=20)

        sns.set(style="darkgrid")
        g = sns.JointGrid(data=self.time_series_data
                        ,x=self.time_series_data[' x_error(m)']
                        ,y=self.time_series_data[' y_error(m)']
                        ,marginal_ticks='True')
        g.plot_marginals(sns.histplot,stat='percent',kde='True')
        g.plot_joint(sns.kdeplot, color="b",fill='True')
        g.plot_joint(sns.scatterplot,s=20,linewidth=0.8)
        g.figure.suptitle('Translation error distribution',y=1)


        plt.figure("Time Series Plot(translation)")
        sns.set(style="darkgrid")
        # Plot multiple lines
        ax= sns.lineplot(x='duration', y=' x_error(m)'
                        ,data=self.time_series_data,label="x_error(m)"
                        ,color='r')
        ax1 = sns.lineplot(x='duration', y=' y_error(m)'
                        ,data=self.time_series_data,label="y_error(m)"
                        ,color='g')
        plt.ylabel("translation error(m)",size=16)

        plt.figure("Time Series Plot(rotation)")
        sns.set(style="darkgrid")
        ax2 = sns.lineplot(x='duration', y=' yaw_error(°)', data=self.time_series_data,label="yaw_error(°)",color='b')
        plt.ylabel("rotation error(°)",size=16)

        plt.show()

if __name__ == '__main__':

    pose_error_visualizer = PoseAnalysisVisualizer()
    rospy.spin()