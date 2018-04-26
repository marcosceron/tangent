#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <cfloat>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <tf/transform_datatypes.h>

//Variaveis Globais Para Leitura de Dados
nav_msgs::Odometry current_pose;
sensor_msgs::LaserScan current_laser;

//Funcao Callback do Laser
void lasercallback(const sensor_msgs::LaserScan::ConstPtr& laser)
{
	current_laser = *laser;

	return;
}

//Funcao Callback da Pose
void posecallback(const nav_msgs::Odometry::ConstPtr& pose)
{
        current_pose = *pose;

	return;
}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "Path");  // Inicializacao do Nodo

    ros::NodeHandle n; 		     

    // Configuracao do topico a ser publicado
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10); 

    // Configuracao dos topicos a serem lidos
    ros::Subscriber sub = n.subscribe("/base_scan", 10, lasercallback);	
    ros::Subscriber sub1 = n.subscribe("/base_pose_ground_truth", 10, posecallback);
   
    // Define a frequencia do no
    ros::Rate loop_rate(2);

    // Declaracoes
    geometry_msgs::Twist speed_create;  // Comando de Velocidade
    double v1=2.0, v2=2.0;              // Velocidades
    double goalx,goaly;                 // Alvos

    //Faz Leitura dos Parametros para o GOAL
    if(argc == 1){
	
        printf("Definido Padrao Aleatorio\n");
        srand(time(NULL));
        goalx=double(rand()%200)/10.0;
        srand(time(NULL));
        goaly=double(rand()%150)/10.0;

    }else{
        if(argc == 3){
            goalx=atof(argv[1]);
            goaly=atof(argv[2]);
            printf("Definido GOAL %lf %lf\n",goalx,goaly);
        }else{
            printf("Error in Goal\n");
            return -1;
        }
    }

    double orientation;
    //Loop Principal
    while(ros::ok()) 
    {
	
	// Diferença X e Y do objetivo para o robô
	double difX = goalx - current_pose.pose.pose.position.x;
	double difY = goaly - current_pose.pose.pose.position.y;
	// Ângulo de orientação do objetivo	
	double goalOrientation = atan2(difY, difX);	
	
	
	/**	
	 Orientação - tipo Quarternion
	 yaw = ângulo de orientação do robô
	**/
	orientation = tf::getYaw(current_pose.pose.pose.orientation);	
	// Ãngulo de radiano para grau	
	orientation = orientation*180/M_PI;	
	goalOrientation = goalOrientation*180/M_PI;	
	
	//ROS_INFO("Goal %lg", goalOrientation);
	//ROS_INFO("Robo %lg", orientation);

	double diferencaAngulos = goalOrientation - orientation;
	
	//ROS_INFO("Dif %lg", diferencaAngulos);

	if (diferencaAngulos > -10 && diferencaAngulos < 10){		
		v2=0;
		
		// pegar a medição 0, central e a última
		
		for (double i=0; i<current_laser.ranges.size(); i+=current_laser.ranges.size()/2) {
			double range = current_laser.ranges[i];
			double xObstaculo = range * cos(current_laser.angle_min + current_laser.angle_increment * i);
			double yObstaculo = range * sin(current_laser.angle_min + current_laser.angle_increment * i);

			// Diferenças coordenadas robô - obstáculo		
			double difObstaculoX = current_pose.pose.pose.position.x - xObstaculo;
			double difObstaculoY = current_pose.pose.pose.position.y - yObstaculo;
		
			printf ("Distancia x: %lg, y: %lg\n", difObstaculoX, difObstaculoY);
				
		}
		
		
		
	}
	
	
	double xRobo = current_pose.pose.pose.position.x;
	double yRobo = current_pose.pose.pose.position.y;

	/**
	printf("Ãngulo máximo: %lf \n", current_laser.angle_max);
	printf("Ãngulo mínimo: %lf \n", current_laser.angle_min);
	printf("Ãngulo incremento: %lf \n", current_laser.angle_increment);
	**/

	//printf("Range mínimo: %lf", current_laser.range_max);

	//printf("Posição atual: %lf, %lf", current_pose.pose.pose.position.x, current_pose.pose.pose.position.y);
	/**	
	if (xRobo>=13.0) {
		printf("Entrei aqui \n");
		v1=0;
		v2=2;
		//printf("Orientação x %lf \n", current_pose.pose.pose.orientation.x);
		//printf("Orientação y %lf \n", current_pose.pose.pose.orientation.y);
		//printf("Orientação z %lf \n", current_pose.pose.pose.orientation.z);
		//printf("Orientação w %lf \n", current_pose.pose.pose.orientation.w);

		
	}
	**/



        // Envia Sinal de Velocidade
        speed_create.linear.x=v1;
        speed_create.angular.z=v2;
        vel_pub.publish(speed_create);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

