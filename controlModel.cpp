#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <math.h>

double posx=0.0;
double posy=0.0;
double thetaO = 0.0;
double pi = 3.14159;

void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg){
	//ROS_INFO("passou");
	
	//ROS_INFO("Seq: [%d]", msg->header.seq);
	//ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
	//ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
//msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
//	ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);

	posx = msg->pose.pose.position.x;
	posy = msg->pose.pose.position.y;	
	thetaO = tf::getYaw(msg->pose.pose.orientation);
//	thetaO = thetaO*180/pi;
}


int main(int argc, char **argv){
	
	ros::init(argc, argv, "subP3AT");
	ros::NodeHandle n;
		
	//função para ouvir os dados da odometria chamando a função Callback acima
	ros::Subscriber sub = n.subscribe("/RosAria/pose",1000, chatterCallback);
	
	geometry_msgs::Twist vel_msg;
	ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel",100);
	ROS_INFO("passouasdeee");

	double Ex=0.0,Ey=0.0; //Erro x e y
	double Xp=-1.0,Yp=1.0; //Posicao desejada de x e y em metros
	double xM=0.0,yM=0.0; //orientacao da velocidade linear e rotacao
	double d=0.25,k1=0.0,k2=0.0;
	double u=0.0,w=0.0;
	
	

	while(ros::ok()){ //loop ate pressionar ctrl+C
			//ROS_INFO("xpositionDentroDoWhil: [%f]",posx);	
			//ROS_INFO("passouasdeeeWhile");
		Ex = Xp - posx;
		Ey = Yp - posy;

		ROS_INFO("Erro de posicao: [%f, %f]",Ex,Ey);	
		xM = (cos(thetaO)*Ex + sin(thetaO)*Ey);
		yM = (-sin(thetaO)*Ex + cos(thetaO)*Ey);			

		ROS_INFO("xM, yM, teta: [%f, %f, %f]",xM,yM, thetaO);	

		xM = xM - d;
		
		k1 = 0.5 /(1 + fabs(xM));
		k2 = (70.0)*(pi/180)/(1 + fabs(yM));

		u = k1*xM;
		w = k2*yM;

		ROS_INFO("u, w: [%f, %f]",u,w);	

		//velocidades lineares
		vel_msg.linear.x = u; //seta a velocidade liner no eixo x para a frente do robo
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;
		
		//velocidades angulares
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = w; //seta o valor de rotação do robo p3at

		//if(chatterCallback->posx>0.7){break;}
		velocity_publisher.publish(vel_msg);
				
		ros::spinOnce();
	}
	//fica fazendo loop ate apertar ctrl+c
	
	ros::spin();
	ROS_INFO("passou2323");
	return 0;
}


