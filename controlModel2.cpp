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

double normRad(double angle){
    while(angle > pi)
        return angle -= 2*pi;
    while(angle < -pi)
        return angle += 2*pi;
    return angle;
}

double thetaD(double x, double y){
    if(x==0.0)
        x = 0.1;
    return (2*atan(y/x));
}

double funcSinC(double theta){
    double sinc = 1;

    if (abs(theta) > 0.001)
        sin(theta)/theta;

    return sinc;
}

double funcSign(double x, double y){
    if((x==0 && y<0) || x>0)
        return 1;
    else if((x=0 && y>0) || x<0)
        return -1;
}

double gama2(double gama, double a){
    return gama/(1 + abs(a));
}

double coordCircular(double x, double y){
    return funcSign(x,y)*sqrt(x*x + y*y)/(funcSinC(thetaD(x,y)/2));
}

double funcB1(double x, double y, double theta){
    return cos(theta)*(((x*thetaD(x,y))/y)-1) + sin(theta)*((thetaD(x,y)/2)*(1-((x*x)/(y*y))) + x/y); 
}

double funcB2(double x, double y, double theta){
    return (cos(theta)*(2*(y/(x*x))*(1/(1+((y*y)/(x*x)))))) - (sin(theta)*2)/(x*(1+((y*y)/(x*x))));
}


int main(int argc, char **argv){
	
	ros::init(argc, argv, "subP3AT");
	ros::NodeHandle n;
		
	//função para ouvir os dados da odometria chamando a função Callback acima
	ros::Subscriber sub = n.subscribe("/RosAria/pose",1000, chatterCallback);
	
	geometry_msgs::Twist vel_msg;
	ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel",100);

	double Ex=0.0,Ey=0.0; //Erro x e y
	double Xp=1,Yp=2.0, thetaP = 0; //Posicao desejada de x e y em metros
	double xM=0.0,yM=0.0, thetaM = 0.0; //orientacao da velocidade linear e rotacao
	double u=0.0,w=0.0, alpha = 0.0;
	double a=0;

	while(ros::ok()){ //loop ate pressionar ctrl+C
			//ROS_INFO("xpositionDentroDoWhil: [%f]",posx);	
			//ROS_INFO("passouasdeeeWhile");
		Ex = Xp - posx;
		Ey = Yp - posy;

		thetaM = normRad(thetaO - thetaP);
		xM = (cos(thetaP)*Ex + sin(thetaP)*Ey);
		yM = (-sin(thetaP)*Ex + cos(thetaP)*Ey);			

		ROS_INFO("xM, yM, theta, theta_er, theta_d: [%f, %f, %f, %f]",xM,yM, thetaO, thetaM);	
		//ROS_INFO("Erro de posicao: [%f, %f]",Ex,Ey);
		a = coordCircular(xM,yM);	
		alpha = normRad(thetaM-thetaD(xM,yM));
		u = -gama2(gama1,a)*funcB1(xM,yM,thetaM)*a; //perguntar se o paramatro é theta erro ou theta atual
		w = (-funcB2(xM,yM,thetaM)*u) - (k*alpha);

		//ROS_INFO("u, w: [%f, %f]",u,w);	

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
