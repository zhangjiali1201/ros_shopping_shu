#  超市机器人项目  
**1.简介**  
中国机器人大赛、服务机器人专项赛上的超市机器人项目（Shopping)
要求机器人启动后跟随引导者至各个货架，并记住货架上指定物品的位置。到达收银台后，假设引导者忘记购买某物品，机器人应当替引导者拿回该物品。比赛开始后，任何人不得接触机器人，指令全部通过语音下达。
  
**2.环境**  
ubuntu14.04, ros indigo
  
***ps.以下操作基于shu自强队的家庭服务机器人danial，本项目仅包含超市机器人项目主框架部分和语音部分。***  
**3.仿真操作**  
*1.启动仿真环境*  
cd /home/ros/robocup  
source devel/setup.bash  
roslaunch bringup simulate-amada.launch  
*2.模拟follow过程（三种方式）*  
cd /home/ros/robocup  
source devel/setup.bash  
roslaunch utility keyboard_teleop.launch `键盘`  
  
cd ~/catkin_ws  
source devel/setup.bash  
roslaunch rbx1_apps follower.launch `kinect深度相机`  
  
roslaunch freenect_launch freenect.launch  //启动kinect  
rosrun image_view disparity_view image:=/camera/depth/disparity   //启动深度图像  
cd ~/catkin_ws  
source devel/setup.bash  
roslaunch rbx2_ar_tags ar_large_markers_kinect.launch  //识别ar大标签  
cd ~/catkin_ws  
source devel/setup.bash  
roslaunch rbx2_ar_tags ar_follower.launch `ar follow`   
*3.启动shopping主框架*  
cd /home/ros/robocup  
source devel/setup.bash  
roslaunch shopping_2018 shopping2.launch  
*4.启动语音识别*  
cd /home/ros/robocup  
source devel/setup.bash  
roslaunch shopping_2018 recognizer.launch  
  
**4.实体机器人操作**  
*1.启动底盘*  
sudo su  
cd /home/ros/catkin_ws  
source devel/setup.bash  
roslaunch base_controller su_bringuphardware.launch  
*2.建图*  
cd /home/ros/robocup  
source devel/setup.bash  
roslaunch bringup nav_with_gmap.launch  
*3.follow（两种方式）*  
cd /home/ros/robocup  
source devel/setup.bash  
roslaunch turtlebot_follower follower.launch    `深度相机点云follow`  
  
roslaunch freenect_launch freenect.launch   //kinect启动  
rosrun image_view disparity_view image:=/camera/depth/disparity  
cd ~/catkin_ws  
source devel/setup.bash  
roslaunch rbx2_ar_tags ar_large_markers_kinect.launch   //识别大标签  
cd ~/catkin_ws  
source devel/setup.bash  
roslaunch rbx2_ar_tags ar_follower.launch   `ar标签follow`  
*4.启动主框架*  
cd /home/ros/robocup  
source devel/setup.bash  
roslaunch shopping_2018 shopping2.launch  
*5.语音识别*  
cd /home/ros/robocup  
source devel/setup.bash  
roslaunch shopping_2018 recognizer.launch
