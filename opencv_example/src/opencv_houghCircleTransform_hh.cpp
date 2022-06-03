/* EXAMPLE 5 USING OPENCV
/*
/*   Base: Hough Circle Transform
/*   from = https://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/hough_circle/hough_circle.html#hough-circle
/*
/*   Base to draw into OpenCV:
/*   from = https://docs.opencv.org/2.4/modules/core/doc/drawing_functions.html
/*
/*   Example in ROS:
/*   from: https://github.com/epsilonorion/ros_tutorials/blob/master/opencv_tut/src/findCircle.cpp
/*
/*   it was modificated by Hernán Hernández to be use like own template
*/

// Includes
#include <ros/ros.h>
#include <stdio.h> // needed to use the class: "vector"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/*#include <iostream>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>     //make sure to include the relevant headerfiles
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <cvaux.h>
#include <math.h>
#include <cxcore.h>
#include <highgui.h>*/

// Defines - General
#define NODE_NAME "opencv_houghCircleTransform_hh"
#define OPENCV_WINDOW1 "Original Image"
#define OPENCV_WINDOW2 "Image Filtered"

// Define - Temas
#define TOPIC1_SUB__IMAGE_INPUT "/usb_cam/image_raw"			 // Imagen obtenida de la cámara (sin procesar).
#define TOPIC1_PUB__IMAGE_OUTPUT "/image_converter/output_video" // Imagen pública a ROS (procesada).

//***Clase: Image Conver (OpenCV)***
class ImageConverter
{
private:
	// NodeHandle ROS
	ros::NodeHandle nh_;

	// Imagen usada
	image_transport::ImageTransport it_;				 // Objeto it_ de la clase de transporte de imágenes (utilizado para el procesamiento de imágenes digitales)
	image_transport::Subscriber topic1_sub__image_input; // Imagen obtenida de la cámara (sin procesar). Formato ROS (Tema)
	image_transport::Publisher topic1_pub__image_output; // Imagen pública a ROS (procesada). Formato ROS (Tema)

public:
	/* Metodo Constructor.
	   TODO */
	ImageConverter() : it_(nh_)
	{
		// Declaración de temas
		topic1_sub__image_input = it_.subscribe(TOPIC1_SUB__IMAGE_INPUT, 1, &ImageConverter::imageCb, this);
		topic1_pub__image_output = it_.advertise(TOPIC1_PUB__IMAGE_OUTPUT, 1);

		// Crear la GUI de Windows (donde imprimir las imágenes)
		cv::namedWindow(OPENCV_WINDOW1);
		cv::namedWindow(OPENCV_WINDOW2);
	}

	/* Método Destructor */
	~ImageConverter()
	{
		// cerrar la interfaz gráfica de usuario de Windows
		cv::destroyWindow(OPENCV_WINDOW1);
		cv::destroyWindow(OPENCV_WINDOW2);
	}

	/* asociado a "TOPIC1_SUB__IMAGE_INPUT" que obtiene la imagen obtenida de la cámara (sin procesar) */
	void imageCb(const sensor_msgs::ImageConstPtr &msg) // msg es la imagen obtenida de la cámara (sin procesar)
	{
		// Convertir imagen ROS (Tema) a imagen OpenCV (Ptr)
		cv_bridge::CvImagePtr cv_OriginalImage_ptr;
		try
		{
			cv_OriginalImage_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception &e)
		{
			// Imprimir un error si se detecta
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		/****************************/
		/* procesando imagen digital */
		/****************************/

		// Convertir datos a la clase cv::Mat
		cv::Mat cvMat_Image = cv_OriginalImage_ptr->image;

		// Transforma la imagen original a formato gris.
		cv::Mat cvMat_GrayImage;
		cv::cvtColor(cvMat_Image, cvMat_GrayImage, CV_BGR2GRAY);

		// Reducir el ruido aplicado un filtro gaussiano, así evitamos la detección de círculos falsos
		cv::Mat cvMat_GrayImage_filtered;
		cv::GaussianBlur(cvMat_GrayImage, cvMat_GrayImage_filtered, cv::Size(9, 9), 2, 2);

		// Aplicar la transformada de Hough para encontrar los círculos
		std::vector<cv::Vec3f> circles;
		cv::HoughCircles(cvMat_GrayImage_filtered, circles, CV_HOUGH_GRADIENT, 2, 20, 100, 155, 0, 0);
		/* Ejemplo: HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 200, 100, 0, 0 );
		Parámetros:
		src_gray: Imagen de entrada (escala de grises)
			círculos: un vector que almacena conjuntos de 3 valores: x_{c}, y_{c}, r para cada círculo detectado.
		CV_HOUGH_GRADIENT: Definir el método de detección. Actualmente este es el único disponible en OpenCV
		dp = 1: La razón inversa de la resolución TODO
		min_dist = src_gray.rows/8: Distancia mínima entre centros detectados POR HACER
		param_1 = Umbral superior para el detector de borde Canny interno TODO
		param_2 = Umbral para la detección del centro. QUE HACER
			min_radius = 0: Radio mínimo a detectar. Si no lo sabe, ponga cero por defecto.
			max_radius = 0: Radio máximo a detectar. Si no lo sabe, ponga cero por defecto */

		// Círculos detectados
		for (size_t i = 0; i < circles.size(); i++) // size_t es una variable que puede almacenar el tamaño máximo de un objeto teóricamente posible, en este caso la longitud de "i" (tamaño desconocido)
		{
			// Dibujar círculos sobre la imagen original
			cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			int radius = cvRound(circles[i][2]);
			circle(cvMat_Image, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);	 // centro del circulo (con radio=3, Color verde)
			circle(cvMat_Image, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0); // contorno del circulo (Con radio real, Color rojo)

			/* circulo(Mat& img, Punto centro, int radio, const Scalar& color, int thick=1, int lineType=8, int shift=0)
			Parámetros:
			img – Imagen donde se dibuja el círculo.
			center – Centro del círculo.
				radio - Radio del círculo.
				color: color del círculo.
				grosor: grosor del contorno del círculo. Si tiene un grosor negativo significa que se va a dibujar un círculo relleno.
				lineType: tipo del límite del círculo. Consulte la descripción de la línea(). Predeterminado 8
				shift – Número de bits fraccionarios en las coordenadas del centro y en el valor del radio. predeterminado 0*/

			// Terminal de impresión
			ROS_INFO("Circle detected #%d / %d: ", int(i) + 1, (int)circles.size());
			ROS_INFO("    x=%d, y=%d, r=%d: ", cvRound(circles[i][0]), cvRound(circles[i][1]), cvRound(circles[i][2]));
		}

		/*********************************/
		/* FIN: procesamiento de imagen digital */
		/*********************************/

		// Actualizar GUI Window1 - Imagen original
		cv::imshow(OPENCV_WINDOW1, cvMat_Image);
		cv::waitKey(3);

		// Actualizar GUI Window2 - Filtro aplicado
		cv::imshow(OPENCV_WINDOW2, cvMat_GrayImage_filtered);
		cv::waitKey(3);
	}
};

//***Función Principal***
int main(int argc, char **argv)
{
	// Inicializamos ROS
	ros::init(argc, argv, NODE_NAME);

	// Objeto de inicialización de la clase ImageConverter, definida anteriormente
	ImageConverter ic;

	// Mientras sea cierto. Obtener datos del tema de suscripción
	ros::spin();
	return 0;
}
