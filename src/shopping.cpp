//for 2018 shanghai
#include <ros/ros.h>
//ros messages
#include <std_msgs/String.h>            
#include <geometry_msgs/PoseStamped.h>
//ros services
#include <topic_tools/MuxSelect.h>
//sound client
#include <sound_play/sound_play.h>
//stl
#include <map>
#include <set>
//tf
#include <tf/transform_listener.h>	//tf :: TransformListener
//move_base
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_client_goal_state.h>
//clear costmaps
#include <std_srvs/Empty.h>


typedef  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient ;	

class Shopping
{
private:
    ros::NodeHandle node_handle_;
    ros::Subscriber voice_cmd_sub;      //subscribe to /recognize/output topic
    ros::Publisher nav_goal_pub;	         //publish to topic : /move_base_simple/goal
    ros::Publisher permit_pub_static_map ;	//permit  map_server_for_shopping to pub static map
    ros::ServiceClient mux_select_client;           //mux_select client
    ros::ServiceClient clear_costmaps_client;          //clear costmaps client
    std::string  map_frame = "map"; 	//the name of the map frame 
    std::map<std::string , geometry_msgs::PoseStamped> goods_mapto_pose;    
    bool pause_listenning = true;  //whether pause listenning
    sound_play::SoundClient sc;
    std::set<std::string> goods{"milk","chips","safeguard","toothpaste","laoganma","cola","icetea","coffee","noodle","water","porridge","napkin","floralwater","terminal"};
    enum Flag {FOLLOW,NAV,NO};
    Flag flag = NO;
    tf::TransformListener listener;	//  tf transform listener 
    //subscribe callbacks
    MoveBaseClient *move_base_client ;		//move_base client
    void voice_cmd_sub_cb(const std_msgs::StringConstPtr &msg) ;
    void send_nav_goal(const geometry_msgs::PoseStamped goalPose) ;		//send goal to move_base node 
    void goal_done_cb( const actionlib::SimpleClientGoalState&  state, const move_base_msgs::MoveBaseResultConstPtr&  result, const std::string otherArg, geometry_msgs::PoseStamped& goal) ;
public:
    Shopping(ros::NodeHandle nh);
    //~Shopping();
};

//constructor
Shopping::Shopping(ros::NodeHandle nh)
    : node_handle_(nh)
{
    voice_cmd_sub = node_handle_.subscribe<std_msgs::String>("voice_cmd",1,&Shopping::voice_cmd_sub_cb,this);
    nav_goal_pub = node_handle_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
   // permit_pub_static_map = node_handle_.advertise<std_msgs::String>("permit_pub_map_flag", 1) ;
    mux_select_client = node_handle_.serviceClient<topic_tools::MuxSelect>("mux/select");        //////   cmd_vel_select_mux/select
    clear_costmaps_client = node_handle_.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");
    move_base_client = new MoveBaseClient("move_base", true) ; 
    bool connect_service_before_timeout =  mux_select_client.waitForExistence(ros::Duration(2.0)); 
    bool connect_move_base_before_timeout = move_base_client->waitForServer(ros::Duration(2.0));
    if(!connect_move_base_before_timeout)
    {
    	ROS_ERROR("failed connecting move_base service...");
    	exit(0);
    }
    if(!connect_service_before_timeout)
    {
        ROS_ERROR("failed connecting mux/select service...");
        exit(0);
    }
    ROS_INFO("===================shopping_node===================");
    ROS_INFO("connected mux/select service...");
    ROS_INFO("connected move_base service...");
    ROS_INFO("node initialized.....");
    ROS_INFO("===================shopping_node===================");
}  



//voice_cmd_sub_cb
void Shopping::voice_cmd_sub_cb(const std_msgs::StringConstPtr &msg)
{
    	auto iter = goods.find(msg->data);
	auto iter2 = goods_mapto_pose.find(msg->data);
        if(msg->data == "follow")
        {
            topic_tools::MuxSelect srv;
            srv.request.topic = "follow_cmd_vel";
            if(mux_select_client.call(srv) == true)
            {
                ROS_INFO("selecting follow_cmd_vel successfully!");
                ROS_INFO("I will follow you");
                sc.say("I will follow you");
                pause_listenning = true;
                flag = FOLLOW;
            }
            else
                ROS_ERROR("selecting follow_cmd_vel was not successfully!");
        }


        else if ( flag == FOLLOW && iter != goods.end() )
        {
            geometry_msgs::PoseStamped pose;
            tf::StampedTransform transform;	
            try
            {
            	listener.lookupTransform("/map","/base_footprint",ros::Time(0),transform);
            }catch ( tf::TransformException ex){
            	ROS_ERROR("%s",ex.what() );
            	exit(0);
            }
             pose.header.frame_id = "map";
             pose.pose.position.x = transform.getOrigin().getX();
             pose.pose.position.y = transform.getOrigin().getY();
             pose.pose.position.z = transform.getOrigin().getZ();
             pose.pose.orientation.x = transform.getRotation().x();
             pose.pose.orientation.y = transform.getRotation().y();
             pose.pose.orientation.z = transform.getRotation().z();
             pose.pose.orientation.w = transform.getRotation().w();
            auto res = goods_mapto_pose.emplace(*iter,pose);
            if(res.second)
            {
                std::cout << "succeeded rememberring " << *iter << std::endl;
                char str[50];
                sprintf(str,"I remember the %s",(*iter).c_str());
                sc.say(str);
                /*debug*/
                std::cout << "current rememberred goods: " << std::endl;
                for(auto elem : goods_mapto_pose)
                {
                    std::cout << "- " << elem.first << std::endl ;
                    std::cout << "'  -(" << elem.second.pose.position.x << ","<<elem.second.pose.position.y<<","<<elem.second.pose.position.z<<"),("
                    		<<elem.second.pose.orientation.x<<","<<elem.second.pose.orientation.y<<","<<elem.second.pose.orientation.z<<","
                    		<<elem.second.pose.orientation.w<<")."<<std::endl;
                }
                /*debug*/

                ///当到达柜台的时候，设置参数permit_save_map为true， 允许map_saver_shopping节点保存地图
  /*              if(*iter == "home")
					{  
                    ROS_INFO("permit to save the map");
                   node_handle_.setParam("permit_save_map",true);      	//permit to save the map 
                    node_handle_.setParam("switch_map_server",true);		//switch to staitc_map 
                }
    */
                pause_listenning = true;
            }
            else
            {
                std::cout << "failed, rememberred " << *iter << " before "<< std::endl;
            }
        }
        else if(msg->data == "navigation")
        {
           // permit_pub_static_map.publish("YES");
            topic_tools::MuxSelect srv;
            srv.request.topic = "nav_cmd_vel";
            if(mux_select_client.call(srv) == true)
            {
                ROS_INFO("selecting nav_cmd_vel successfully.");
                ROS_INFO("I will start navigation");
                sc.say("I will start navigation.");
                flag = NAV;
            }
            else
            {
                ROS_INFO("selecting nav_cmd_vel was not successfully.");
            }
        }
        else if(flag == NAV && iter2 != goods_mapto_pose.end())
        {
            std::cout << "I will go to get " << iter2->first<< std::endl;
            char str[50];
            sprintf(str,"I will go to get %s",( (iter2->first).c_str() ));
            iter2->second.header.stamp  = ros::Time::now();
            //nav_goal_pub.publish(iter2->second);
            sc.say(str);
            pause_listenning = true;
            send_nav_goal(iter2->second);
            
        }    
  //  }   
}

 void Shopping::goal_done_cb( const actionlib::SimpleClientGoalState&  state, const move_base_msgs::MoveBaseResultConstPtr&  result, const std::string otherArg,  geometry_msgs::PoseStamped& goal )
 {
 	if ( otherArg == "GET")		//go to get goods done
 	{
 		if (state.toString() == "SUCCEEDED")
 		{
 			ROS_INFO("I succeed getting it ");
 			sc.say("I succeed  reaching the goal");
 			ros::Duration(5.0).sleep();
 			ROS_INFO("I will back");
 			sc.say("I will back");

 			/////TODO:   
 			move_base_msgs::MoveBaseGoal goal1;
 			goal1.target_pose.header.stamp = ros::Time::now();
 			goal1.target_pose.header.frame_id = "map";
            try{
            goal = goods_mapto_pose.at("terminal");
 			goal1.target_pose = goal;
 			}
 			catch(const std::out_of_range& err ){
 				//ROS_ERROR( err.what() );
                 std::cerr << err.what() << std::endl;
 			}
 			
 			move_base_client->sendGoal(goal1, std::bind(&Shopping::goal_done_cb,this,std::placeholders::_1,std::placeholders::_2, "BACK", goal));
 		}
         else           //failed  =>   clear costmap  , reset the goal
         {
            ROS_INFO("I fail on the way, I will clear costmaps and reset the goal..");
            sc.say("I failed,clear costmaps");
            std_srvs::Empty srv;
            if(clear_costmaps_client.call(srv) == true){
                ROS_INFO("clear costmaps success");
            }
            else{
                ROS_WARN("clear_costmaps failed");
            }
            move_base_msgs::MoveBaseGoal goal1;
            goal1.target_pose = goal;
            goal1.target_pose.header.stamp = ros::Time::now();
            move_base_client->sendGoal(goal1,std::bind(&Shopping::goal_done_cb,this,std::placeholders::_1,std::placeholders::_2, "GET", goal));

         }
 	}	
 	else if (otherArg == "BACK")	//back done
 	{
 		if(state.toString() == "SUCCEEDED")
 		{
 			ROS_INFO("I succeed backing");
 			sc.say("I back.");
 		}
        else{
            ROS_INFO("failed back");
            sc.say("I failed,clear costmaps");
            std_srvs::Empty srv;
            if(clear_costmaps_client.call(srv) == true){
                ROS_INFO("clear costmaps success");
            }
            else{
                ROS_WARN("clear costmaps failed");
            }
            move_base_msgs::MoveBaseGoal goal1;
            goal1.target_pose = goal;
            goal1.target_pose.header.stamp = ros::Time::now();
            move_base_client->sendGoal(goal1,std::bind(&Shopping::goal_done_cb,this,std::placeholders::_1,std::placeholders::_2, "BACK", goal));
        }
 	}

 }

void Shopping::send_nav_goal(const geometry_msgs::PoseStamped goalPose)
{
	move_base_msgs::MoveBaseGoal goal ;
	goal.target_pose = goalPose;
	goal.target_pose.header.stamp = ros::Time::now();
	move_base_client->sendGoal(goal, std::bind(&Shopping::goal_done_cb,this,std::placeholders::_1,std::placeholders::_2, "GET",goalPose));
	move_base_client->waitForResult();
}


int main(int argc,char **argv)
{
    ros::init(argc,argv,"shopping_node");
    ros::NodeHandle nh;
    Shopping shopping(nh);
    ros::spin();
    return 0;
}
