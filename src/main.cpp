/*
*
*   LookAt control.- 
*       This program get the value from the detection module, 
*       and send it to arduino to move the servos
*       Authors: Sandra & David
*/

// Librerias para incluir
#include <iostream>
#include <vector>

// Includes to PID
#include <ctime>

// ROS includes
#include "ros/ros.h"

#include "geometry_msgs/Point.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

// Valores para ajustar los valores de los grados
const float DEGREE_ADJUST_X = 0.14f;
const float DEGREE_ADJUST_Y = 0.1875f;

// Valores para definir el centro de la imagen
const int WIDTH = 640;
const int HEIGHT = 480;

// Prototipo de las funciones
void pointCallback(const geometry_msgs::Point::ConstPtr& point);

// Flag variable to control a correct data
bool dataReceived = false;

// Se inicializan las variables para detectar la posición el objeto
int x = 0; 
int y = 0;

// Variable de control de la detección de cara
bool detected = false;

// Definición de los margenes máximos y mínimos del Tilt
const int MAX_TILT = 135;
const int MIN_TILT = 45;

// Bloque principal
int main(int argc, char **argv)
{
    // Variables locales
    int count = 5;

    // Ros Init
    ros::init(argc, argv, "LookAt_Control");
    ros::NodeHandle n;

    // Publisher to the Arduino board
    ros::Publisher Servo_pub = n.advertise<std_msgs::Int32MultiArray>("PAT", 1000);
    ros::Rate loop_rate(10);

    // Subscriber to the detectface
    ros::Subscriber sub = n.subscribe("pointpub", 1000, pointCallback);

    // Variables para el centro del sistema de control
    // Se inicializa con los valores medios de la pantalla
    float xControl = WIDTH / 2.0f;
    float yControl = HEIGHT / 2.0f;

    // Variables de control que serán las mismas para x e y.
    float kProp = 0.2f; // Ganancia proporcional
    float kInt = 0.01f; // Ganancia integral
    float kDeriv = 0.01f; // Ganancia derivativa

    // variables para los errores para X
    float errorAct_X = 0.0f;
    float errorAnt_X = 0.0f;
    float errorAcum_X = 0.0f; // Variable de error acumulado para cuando ha detectado
    float errorAcum_no_X = 0.0f; // Variable de error acumulado para cuando no ha detectado

    // variables para los errores para Y
    float errorAct_Y = 0.0f;
    float errorAnt_Y = 0.0f;
    float errorAcum_Y = 0.0f;    // Variable de error acumulado para cuando ha detectado
    float errorAcum_no_Y = 0.0f; // Variable de error acumulado para cuando no ha detectado

    // Salida del controlador en X
    float outputController_X = 0.0f;

    // Salida del controlador en Y
    float outputController_Y = 0.0f;

    // Variable para guardar el periodo, que variará dependiendo de lo que tarde en ejecutar el código
    float samplePeriod = 0.0f;

    // Variables para el cálculo del tiempo
    clock_t reloj;

    // Variable de control de los grados de giro o yaw
    int degrees_X = 0; // se inicializa como 0

    // Variable de control de los grados de inclinación o pitch
    int degrees_Y = 0; // se inicializa en el 0

    // Inicializo los valores de pitch y yaw
    int pitch = 120;
    int yaw = 120;


    while(1)
    {
        // Se captura el tiempo de inicio del bucle
        reloj = clock();

        // Se calcula el tiempo que ha pasado entre el principio del bucle y se imprime
        // por pantalla los frames por segundo
        reloj = clock() - reloj;
        //std::cout << "\rEl tiempo que ha pasado es " << (static_cast<float>(reloj) / CLOCKS_PER_SEC) << " se ha detectado objeto = " << detected << std::flush;

        // Se actualiza el valor del periodo de muestreo
        samplePeriod = static_cast<float>(reloj) / CLOCKS_PER_SEC;

        if(dataReceived)
        {
            dataReceived = false;   
            // Sistema de control 
            if (detected)
            {
                // Si hay un objeto en pantalla, entonces se mueve el puntero hacia el
                
                // Se ejecuta el control para X

                // Se ajusta el error actual
                errorAct_X = x - xControl;

                // Se actualiza el error integral
                errorAcum_X += errorAct_X;

                // Se calcula la salida del controlador
                // u = Kp * error + Kint * Ts * errorAcumulado + Kder * (e - e(t-1))/Tsample
                outputController_X = kProp * errorAct_X + kInt * samplePeriod * errorAcum_X + (kDeriv * (errorAct_X - errorAnt_X)) / samplePeriod;

                // Se actualiza el valor de la posición 
                // Xsalida = Xactual + corrección en x
                xControl += outputController_X;

                // Se guarda el valor de error para el control derivativo
                errorAnt_X = errorAct_X;

                // Se ejecuta el control para Y

                // Se ajusta el error actual
                errorAct_Y = y - yControl;

                // Se actualiza el error integral
                errorAcum_Y += errorAct_Y;

                // Se calcula la salida del controlador
                // u = Kp * error + Kint * Ts * errorAcumulado + Kder * (e - e(t-1))/Tsample
                outputController_Y = kProp * errorAct_Y + kInt * samplePeriod * errorAcum_Y + (kDeriv * (errorAct_Y - errorAnt_Y)) / samplePeriod;

                // Se actualiza el valor de la posición 
                // Ysalida = Yactual + corrección en y
                yControl += outputController_Y;


                // Se guarda el valor de error para el control derivativo
                errorAnt_Y = errorAct_Y;
            }
            else
            {
                // TODO: Decidir que se hará en reposo.
                // Si no hay un objeto en pantalla, entonces se vuelve al centro.
                
                // Se ejecuta el control para X

                // Se ajusta el error actual
                errorAct_X = (WIDTH / 2.0f) - xControl;

                // Se actualiza el error integral
                errorAcum_X += errorAct_X;

                // Se calcula la salida del controlador
                // u = Kp * error + Kint * Ts * errorAcumulado + Kder * (e - e(t-1))/Tsample
                outputController_X = kProp * errorAct_X + kInt * samplePeriod * errorAcum_X + (kDeriv * (errorAct_X - errorAnt_X)) / samplePeriod;

                // Se actualiza el valor de la posición 
                // Xsalida = Xactual + corrección en x
                xControl += outputController_X;

                // Se guarda el valor de error para el control derivativo
                errorAnt_X = errorAct_X;

                // Se ejecuta el control para Y

                // Se ajusta el error actual
                errorAct_Y = (HEIGHT / 2.0f) - yControl;

                // Se actualiza el error integral
                errorAcum_Y += errorAct_Y;

                // Se calcula la salida del controlador
                // u = Kp * error + Kint * Ts * errorAcumulado + Kder * (e - e(t-1))/Tsample
                outputController_Y = kProp * errorAct_Y + kInt * samplePeriod * errorAcum_Y + (kDeriv * (errorAct_Y - errorAnt_Y)) / samplePeriod;

                // Se actualiza el valor de la posición 
                // Ysalida = Yactual + corrección en y
                yControl += outputController_Y;

                // Se guarda el valor de error para el control derivativo
                errorAnt_Y = errorAct_Y;
            }


            // Se ajusta el error en grados, tanto en X como en Y
            // Esta parte del código será la que mueva el motor

            // Se adapta la salida en X a grados, y se redondea a int
            degrees_X = 90 - static_cast<int>((xControl - 320) * DEGREE_ADJUST_X);

            // Se actualiza el valor para que no pase de 0 o 180
            if(degrees_X > 180)
            {
                // Se fuerza el valor máximo
                degrees_X = 180;
            }

            if(degrees_X < 0)
            {
                // Se fuerza el valor mínimo
                degrees_X = 0;
            }
           
            // Se imprime en la consola, en vez de en pantalla
            //std::cout << "El angulo a corregir en X: " << degrees_X << std::endl;

            // Se adapta la salida en Y a grados, y se redondea a int
            degrees_Y = 90 - static_cast<int>((yControl - 240) * DEGREE_ADJUST_Y);

            // Se actualiza el valor para que no pase de los valores definidos
            // en los margenes
            if(degrees_Y > MAX_TILT)
            {
                // Se fuerza el valor máximo
                degrees_Y = MAX_TILT;
            }

            if(degrees_Y < MIN_TILT)
            {
                // Se fuerza el valor mínimo
                degrees_Y = MIN_TILT;
            }


            // Se envía el valor calculado al Arduino para que actualize la posición de inclinación
            

            // Define the array msg variable
            std_msgs::Int32MultiArray array;

            // Clear the data
            array.data.clear();

            array.data.push_back(degrees_X);
            array.data.push_back(degrees_Y);

            Servo_pub.publish(array);

            // Lo printo en la consola, en vez de en pantalla
            //std::cout << "El angulo a corregir en Y: " << degrees_Y << std::endl;
        }

        // Ros
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Se sale del programa
	return 0;
}

// ROS subscriber function
void pointCallback(const geometry_msgs::Point::ConstPtr& point)
{
    geometry_msgs::Point new_point = *point;
    x = static_cast<int>(new_point.x);
    y = static_cast<int>(new_point.y);
    if(x >= 1000 && y >= 1000)
    {
        // There is no face detected
        detected = false;
    }
    else
    {
        detected = true;
    }

    // Prueba de envio de comandos
    std::cout << "El valor de x es : " << x << " y de y: " << y << std::endl;
    // Habilitate the control
    dataReceived = true;   
}