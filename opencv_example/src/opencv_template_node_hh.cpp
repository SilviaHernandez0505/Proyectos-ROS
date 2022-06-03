/* Template to use Open Cv inside ROS.
/*
/*   Base: ROS tutorial
/*   from = http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages 
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
#define    NODE_NAME       	"opencv_template_node_hh"
#define    OPENCV_WINDOW       "Image window"

// Defines - Topics 
#define    TOPIC1_SUB__IMAGE_INPUT      "/usb_cam/image_raw" 		// Image get from camera (raw). 
#define    TOPIC1_PUB__IMAGE_OUTPUT     "/image_converter/output_video" // Image public to ROS (processed).

// CLASE: Image Conver (OpenCV)
class ImageConverter
{
    private: 
    	// NodeHandle ROS
    	ros::NodeHandle nh_;

    	// Imagen usada
    	image_transport::ImageTransport it_; // Objeto it_ de la clase de transporte de imágenes (utilizado para el procesamiento de imágenes digitales)
    	image_transport::Subscriber topic1_sub__image_input; // Imagen obtenida de la cámara (sin procesar). Formato ROS (Tema)
    	image_transport::Publisher topic1_pub__image_output;// Imagen pública a ROS (procesada). Formato ROS (Tema)

    public:

	/* Metodo constructor. 
	   TODO */
  	ImageConverter() : it_(nh_)
  	{
    	    // Declaración de temas
       	    topic1_sub__image_input = it_.subscribe(TOPIC1_SUB__IMAGE_INPUT, 1, &ImageConverter::imageCb, this); 
   	    topic1_pub__image_output = it_.advertise(TOPIC1_PUB__IMAGE_OUTPUT, 1);

	    // Crear la ventana GUI (donde imprimir la imagen)
    	    cv::namedWindow(OPENCV_WINDOW);
  	}

	/* Método Destructor */
  	~ImageConverter()
  	{
	   // cerrar la ventana GUI
    	    cv::destroyWindow(OPENCV_WINDOW);
  	}
/* asociado a "TOPIC1_SUB__IMAGE_INPUT" que obtiene la imagen obtenida de la cámara (sin procesar) */
	void imageCb(const sensor_msgs::ImageConstPtr& msg) // msg es la imagen obtenida de la cámara (sin procesar)
  	{
	   // Convertir imagen ROS (Tema) a imagen OpenCV (Ptr)
    	    cv_bridge::CvImagePtr cv_OriginalImage_ptr; // copia la imagen del tema de ROS a CvImage de OpenCV
    	    try
    	    {
      		cv_OriginalImage_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); // constantes posibles: "mono8", "bgr8", "bgra8", "rgb8", "rgba8", "mono16"... Opción cuando se usa el tipo de datos "cv_bridge::CvImagePtr"
    	    }
	    catch (cv_bridge::Exception& e)
    	    {
	// Imprimir un error si se detecta
      		ROS_ERROR("cv_bridge exception: %s", e.what());
      		return;
    	    }

		/****************************/
		/* procesando imagen digital */
		/****************************/
			
		// TODO TODO TODO

		/***********************************/
		/* FIN: procesamiento de imagen digital */
		/***********************************/

     // Dibujar un círculo de ejemplo en la transmisión de video
    	    if (cv_OriginalImage_ptr->image.rows > 60 && cv_OriginalImage_ptr->image.cols > 60)
      	    cv::circle(cv_OriginalImage_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    	    // Actualizar ventana GUI
   	    cv::imshow(OPENCV_WINDOW, cv_OriginalImage_ptr->image);
    	    cv::waitKey(3);

		// Convertir imagen OpenCV (Ptr) a imagen ROS (Tema)
		// Emitir flujo de video modificado

    	    topic1_pub__image_output.publish(cv_OriginalImage_ptr->toImageMsg());
  	}
};

//---Función MAIN---
int main(int argc, char** argv)
{
    // Inicializar ROS
    ros::init(argc, argv, NODE_NAME);
  
// Objeto de inicialización de la clase ImageConverter, definida anteriormente    
	ImageConverter ic;

// Mientras sea cierto. Obtener datos del tema de suscripción    
	ros::spin();
    return 0;
}
