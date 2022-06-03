/* EXAMPLE 2 USING OPENCV
/*
/*   Base: OpenCV tutorial: "Load, Modify, and Save an Image"
/*   from = https://docs.opencv.org/2.4/doc/tutorials/introduction/load_save_image/load_save_image.html#load-save-image
/*
/*   Aditional: Presentation "ROS_Lecture10.pptx"
/*   File: Annexed
/*
/*   it was modificated by Hernán Hernández to be use like own template
*/

// Includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Defines - General
#define NODE_NAME "opencv_grayImage_hh"
#define OPENCV_WINDOW1 "Original Image"
#define OPENCV_WINDOW2 "Gray Image"

// Defines - Topics
#define TOPIC1_SUB__IMAGE_INPUT "/usb_cam/image_raw"			 // Imagen obtenida de la cámara (sin procesar).
#define TOPIC1_PUB__IMAGE_OUTPUT "/image_converter/output_video" // Imagen pública a ROS (procesada).

// Clase: Image Conver (OpenCV)
class ImageConverter
{
private:
	// NodeHandle ROS
	ros::NodeHandle nh_;

	// Imagen utilizada
	image_transport::ImageTransport it_;				 // Objeto it_ de la clase de transporte de imágenes (utilizado para el procesamiento de imágenes digitales)
	image_transport::Subscriber topic1_sub__image_input; // Imagen obtenida de la cámara (sin procesar). Formato ROS (Tema)
	image_transport::Publisher topic1_pub__image_output; // Imagen pública a ROS (procesada). Formato ROS (Tema)

public:
	/* Constructor Method.
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
		cv::Mat cvMat_OriginalImage_ptr = cv_OriginalImage_ptr->image;

		// Transforma la imagen original a formato gris.
		cv::Mat cvMat_GrayImage_ptr;
		cv::cvtColor(cvMat_OriginalImage_ptr, cvMat_GrayImage_ptr, CV_BGR2GRAY);

		/*********************************/
		/* FIN: procesamiento de imagen digital */
		/*********************************/

		// Actualizar GUI Window1 - Imagen original
		cv::imshow(OPENCV_WINDOW1, cv_OriginalImage_ptr->image);
		cv::waitKey(3);

		// Actualizar GUI Window2 - Imagen gris
		cv::imshow(OPENCV_WINDOW2, cvMat_GrayImage_ptr);
		cv::waitKey(3);

		// Convertir imagen OpenCV (Mat) a imagen OpenCV (Bridge) a imagen ROS (Tema)
		cv_bridge::CvImage cv_GrayImage;							 // Es necesario usar la Clase CvImage no CvImagePtr
		cv_GrayImage.header = cv_OriginalImage_ptr->header;			 // Misma marca de tiempo y marco tf que la imagen original. El seq es asignado por ROS automáticamente
		cv_GrayImage.encoding = sensor_msgs::image_encodings::MONO8; // MONO8 Y MONO16 son iguales al formato GREY
		cv_GrayImage.image = cvMat_GrayImage_ptr;					 // Datos
																	 // Emitir flujo de video modificado
		topic1_pub__image_output.publish(cv_GrayImage.toImageMsg());
	}
};

// Función main
int main(int argc, char **argv)
{
	// Inicializa ROS
	ros::init(argc, argv, NODE_NAME);

	// Objeto de inicialización de la clase ImageConverter, definida anteriormente
	ImageConverter ic;

	// Mientras sea cierto. Obtener datos del tema de suscripción
	ros::spin();
	return 0;
}
