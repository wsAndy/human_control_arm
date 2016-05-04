
/*
*   You can use this to get an narrow of 2DOF arm move. Just for fun. use you arm to control the machine arm.
*   This is a demo of 2DOF's arm moving follow a human's arm(right or left)
*   If you want to use left(right) arm, just change the joint.position[here]
*   if you use left arm, the shoulder is 'C',
*                        the arm's joint is 'B',
*                        the hand is 'A'
*   Please according to xtion's axes.
*   
*   Human's skeleton is got by the package which was wrote by Alessio with the help of `openni2_tracker`.
*   you can find it in `https://github.com/Chaos84/skeleton_tracker`
*
*/

/*
*   						Rule
*   Put your right hand in your right shoulder means to STOP
*   
*   Put your right hand horizonally means to start. And the machine arm will follow your left arm's action.
*/

/*
*   Auther: Sheng Wang
*   Mail: shengwangandy@foxmail.com
*   Date: 2016`1`22
*/

#include "arm_control.h"

HumanControlArm::HumanControlArm()
{
  ros::NodeHandle n;
  ros::NodeHandle nh_;

  count = 0;                     
  threshold_start = 0;

  init_joint_data();

  init_data(A0,B0,C0);           
  init_data(A1,B1,C1);

  theta_below_init = 0;          // 
  theta_abrove_init = 0;         //

  nh_.param("cycle",cycle,10);   //
  nh_.param("INIT",INIT,10);     //
  nh_.param("N",N,5);            //

  nh_.param("joint_name",joint_name,std::string("states") );
  nh_.param("camera_topic",camera_topic,std::string("peopleJoints"));

  joint_save.push_back("arm1_bearing_joint");
  joint_save.push_back("arm2_arm1_joint");
  joint_save.push_back("arm3_arm2_joint");

 // get from the arm model. you need to added `<remap from="joint_state" to="states"/>` in <node joint_state_publisher > which in a launch file belongs to the model package created by SolidWorks.
  joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states",1); 

//  arm_pub = nh_.advertise<explorer_msgs::explorer_arm>("explorer_arm",1);

  joint_sub_robot = n.subscribe(joint_name,10, &HumanControlArm::jointGet, this);

  joint_sub_camera = n.subscribe( camera_topic,10,&HumanControlArm::getFromCamera,this);

}

HumanControlArm::~HumanControlArm()
{
}

void HumanControlArm::init_joint_data()
{
    for(int i = 0; i < joint_save.size(); i++)
    {
        joint_data[joint_save[i]] = 0;
    }

}


void HumanControlArm::getFromCamera(const skeleton_tracker::user_Joints::ConstPtr& human_joint)
{


  if(human_joint->name.size() == 0)
  {
	ROS_INFO_ONCE("Have not find a human");
	return;
  }

// stop in the process
  if( int(abs(human_joint->x[3] - human_joint->x[4])) < 40 
      && int(abs(human_joint->z[3] - human_joint->z[4])) < 60 
      && int(abs(human_joint->y[3] - human_joint->y[4])) < 200 
   )
  {
	threshold_start = 0;
	threshold_rec = 0;
	ROS_INFO_ONCE("STOP...");
	init_data(A0,B0,C0);
	init_data(A1,B1,C1);
	theta_below_init = 0;
	theta_abrove_init = 0;
	init_joint_data();
	return ;
  }


if(  int(abs(human_joint->y[3] - human_joint->y[4])) < 30 
     &&  int(abs(human_joint->x[3] - human_joint->x[4])) > 400 
     &&  threshold_start == 0 )
{
	 threshold_start = 1;
	 ROS_INFO_ONCE("Start...");
} 

if(threshold_start == 1 && threshold_rec < INIT)
{
    threshold_rec ++;

	A0.x += human_joint->x[1];
	A0.y += human_joint->y[1];

	B0.x += human_joint->x[0];
	B0.y += human_joint->y[0];

	C0.x += human_joint->x[2];
	C0.y += human_joint->y[2];

}

if(threshold_rec == INIT)
{
//WARN: you can't change the order
	A0.x = (A0.x-B0.x)/INIT;
	A0.y = (A0.y-B0.y)/INIT;
	
	B0.x = (B0.x-C0.x)/INIT;
	B0.y = (B0.y-C0.y)/INIT;

	theta_below_init = atan2( B0.x , B0.y);
	theta_abrove_init = atan2( A0.y , -A0.x);

	threshold_rec ++;

	ROS_INFO_ONCE("Angle init is ok, working ");
}


if(threshold_rec - INIT ==1)
{
    count ++;
    if(count == N)
	{
    // start to calculate
	A1.x = (A1.x-B1.x)/N;
	A1.y = (A1.y-B1.y)/N;
	
	B1.x = (B1.x-C1.x)/N;
	B1.y = (B1.y-C1.y)/N;

	theta_below_ = atan2( B1.x , B1.y);
	theta_abrove_ = atan2( A1.y , -A1.x);

	det_theta_below = theta_below_ - theta_below_init;     // radian
    det_theta_abrove = theta_abrove_ - theta_abrove_init;   // radian

    joint_data["arm2_arm1_joint"] = det_theta_below * 1.0 ;
    joint_data["arm3_arm2_joint"] = det_theta_abrove*(-1.0) ; // the small arm has a larger angle if joint_data["arm3_arm2_joint"] be smaller

/*
*    here you can publish the angle to the dynamical machine.
*    E.G.   arm_pub.publish(***);
*
*/

	if(joint_data["arm2_arm1_joint"] - 2.6 > 1e-6){joint_data["arm2_arm1_joint"] = 2.6;}
	else if(joint_data["arm2_arm1_joint"] < 0){joint_data["arm2_arm1_joint"] = 0;}

	if(joint_data["arm3_arm2_joint"] - 0.3 > 1e-6){joint_data["arm3_arm2_joint"] = 0.3;}
	else if(joint_data["arm3_arm2_joint"] + 2.6 < 1e-6){joint_data["arm3_arm2_joint"]= -2.5;}

	init_data(A1,B1,C1);
	count =0;

    }
	else{
	// loop to reduce error
	A1.x += human_joint->x[1];
	A1.y += human_joint->y[1];

	B1.x += human_joint->x[0];
	B1.y += human_joint->y[0];

	C1.x += human_joint->x[2];
	C1.y += human_joint->y[2];

	}

}// if(threshold_rec == INIT +1)
}


// for Rviz to show you motion with the arm
void HumanControlArm::jointGet(const sensor_msgs::JointState::ConstPtr& robot_joint)
{

    sensor_msgs::JointState joint;
    joint.header.stamp = ros::Time::now();
    joint.name.resize(joint_save.size());
    joint.position.resize(joint_save.size());

    for(int i =0 ;i< joint_save.size() ;i++)
    {
        joint.name[i] = joint_save[i];
        joint.position[i] = joint_data[joint_save[i]] ;
    }

    joint_pub_.publish(joint);
}


void HumanControlArm::init_data(geometry_msgs::Point &A,geometry_msgs::Point &B,geometry_msgs::Point &C)
{
    A.x = 0;
	A.y = 0;
    B.x = 0;
	B.y = 0;
    C.x = 0;
	C.y = 0;
}


int main(int argc , char ** argv )
{
  ros::init(argc , argv , "human_control_arm");
  HumanControlArm start;

  ros::spin();

  return 0;
}
