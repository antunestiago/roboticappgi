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
double gama1 = 1;
double k = 1;

// posicao de parada (ou final ou desejada, como queira chamar)
double Xp=0.5,Yp=-2, thetaP = -pi/2;

void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg){
	posx = msg->pose.pose.position.x;
	posy = msg->pose.pose.position.y;	
	thetaO = tf::getYaw(msg->pose.pose.orientation);
}

double normRad(double angle){
    while(angle > pi)
        return angle -= 2.0*pi;
    while(angle < -pi)
        return angle += 2.0*pi;
    return angle;
}

double thetaD(double x, double y){
    if(x==0.0)
        x = 0.1;
    return (2.0*atan(y/x));
}

double funcSinC(double theta){
    double sinc = 1.0;

    if (abs(theta) > 0.001)
        sin(theta)/theta;

    return sinc;
}

double funcSign(double x, double y){
    if((x==0.0 && y<0.0) || x>0.0)
        return 1.0;
    //else if((x==0.0 && y>=0.0) || x<0.0)
    return -1.0;
}

double gama2(double gama, double a){
    return gama/(1.0 + abs(a));
}

double coordCircular(double x, double y){
    return funcSign(x,y)*sqrt(x*x + y*y)/(funcSinC(thetaD(x,y)/2.0));
}

double funcB1(double x, double y, double theta, double alpha){
    double mybeta = y/x;
    double theta_d = thetaD(x,y);

    if (theta_d != 0)
        return cos(theta)*((theta_d/mybeta) - 1.0) + sin(theta)*(theta_d/2.0*(1.0-1.0/pow(mybeta,2.0)) + 1.0/mybeta);
    return cos(alpha);    
}

double funcB2(double x, double y, double theta){
    double mybeta = y/x;
    double theta_d = thetaD(x,y);

    if (theta_d != 0)
        return cos(theta)*(2.0*mybeta/((1.0+pow(mybeta,2.0))*x)) - sin(theta)*(2.0/((1.0+pow(mybeta,2.0))*x));
    return sin(theta_d/2.0 - theta) * (2.0 / funcSinC(theta_d/2.0));
}


int main(int argc, char **argv){
	
	ros::init(argc, argv, "subP3AT");
	ros::NodeHandle n;
		
	//função para ouvir os dados da odometria chamando a função Callback acima
	ros::Subscriber sub = n.subscribe("/RosAria/pose",1000, chatterCallback);
	
	geometry_msgs::Twist vel_msg;
	ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel",100);

	double Ex=0.0,Ey=0.0;
	double xM=0.0,yM=0.0, thetaM = 0.0;
	double u=0.0,w=0.0, alpha = 0.0;
	double a=0.0, fb1, fb2;

	while(ros::ok()){ //loop ate pressionar ctrl+C
			//ROS_INFO("xpositionDentroDoWhil: [%f]",posx);	
			//ROS_INFO("passouasdeeeWhile");
		Ex = posx - Xp;
		Ey = posy - Yp;
		thetaM = thetaO - thetaP;

		xM = (cos(thetaP)*Ex + sin(thetaP)*Ey);
		yM = (-sin(thetaP)*Ex + cos(thetaP)*Ey);			

		a = coordCircular(xM,yM);	
		alpha = normRad(thetaM-thetaD(xM,yM));

        fb1 = funcB1(xM,yM,thetaM, alpha);
		u = -gama2(gama1,a)*fb1*a;

        fb2 = funcB2(xM,yM,thetaM);
		w = (-fb2*u) - (k*alpha);

        /*
		ROS_INFO("xM, yM, thetaM: [%f, %f, %f]",xM,yM, thetaM);	
		ROS_INFO("thetaO, thetaP, alpha, : [%f, %f, %f]",thetaO, thetaP, alpha);	
		ROS_INFO("fb1, fb2: [%f, %f]",fb1, fb2);	
		ROS_INFO("a, alpha, u, w: [%f, %f,%f, %f]",a, alpha,u,w);	
        */

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
