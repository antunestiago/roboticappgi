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
double gama1 = 0.4;
double k = 1;

void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg){
	posx = msg->pose.pose.position.x;
	posy = msg->pose.pose.position.y;	
	thetaO = tf::getYaw(msg->pose.pose.orientation);
}


int main(int argc, char **argv){
	
	ros::init(argc, argv, "subP3AT");
	ros::NodeHandle n;
		
	//função para ouvir os dados da odometria chamando a função Callback acima
	ros::Subscriber sub = n.subscribe("/RosAria/pose",1000, chatterCallback);
	
	geometry_msgs::Twist vel_msg;
	ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel",100);

	double Ex=0.0,Ey=0.0; //Erro x e y
	double Xp=-1.0,Yp=1.0; //Posicao desejada de x e y em metros
	double xM=0.0,yM=0.0; //orientacao da velocidade linear e rotacao
	double d=0.25,k1=0.0,k2=0.0;
	double u=0.0,w=0.0;
	double theta_erro = 0;
	

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

		//u = k1*xM;
		//w = k2*yM;
		u = -gama2(gama1)*funcB1()*coordCircular(xM,yM);
        theta_erro = thetaO - thetaD(xM, yM);

		w = (-funcB2()*u) - (k*theta_erro);

		ROS_INFO("u, w: [%f, %f]",u,w);	

		//velocidades lineares
        //seta a velocidade liner no eixo x para a frente do robo
		vel_msg.linear.x = u; 
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
	return 0;
}

double thetaD(double x, double y){
    return 2*arctan(y/x)*(pi/180);
}

double funcSinC(double theta){
    sinc = 1;

    if (abs(theta) > 0.001)
        sin(theta)/theta;

    return sinc;
}

double funcSign(double x, double y){
    if((x==0 && y<0) || x>0)
        return 1;
    // else if((x=0 && y>0) || x<0)
    return -1;
}

double gama2(double gama){
    return gama/(1 + abs(a))
}

double coordCircular(double x, double y){
    double a;

    a = sign(x,y)*sqrt(x^2 + y^2)/(funcSinC(thetaD(x,y)/2));

    return a;
}

double funcB1(){
    
}

double funcB2(){
    
}


