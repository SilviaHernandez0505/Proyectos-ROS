/* EXAMPLE 4 USING OPENCV
/*
/*   Base: Smoothing Images
/*   from = https://docs.opencv.org/2.4/doc/tutorials/imgproc/gausian_median_blur_bilateral_filter/gausian_median_blur_bilateral_filter.html#smoothing
/*
/*   Video explain the Gaussian Blur Filter 
/*   From = https://www.youtube.com/watch?v=7LW_75E3A1Q
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
#define    NODE_NAME       	"opencv_smoothingImages_hh"
#define    OPENCV_WINDOW1       "Original Image"
#define    OPENCV_WINDOW2       "New Image (Filter Applied)"

// Defines - Topics 
#define    TOPIC1_SUB__IMAGE_INPUT      "/usb_cam/image_raw" 		// Image get from camera (raw). 
#define    TOPIC1_PUB__IMAGE_OUTPUT     "/image_converter/output_video" // Image public to ROS (processed).

// Define - Filter Selected
#define    HOMOGENEOUS_BLUR	0
#define    GAUSSIAN_BLUR	1
#define    MEDIAN_BLUR		2
#define    BILATERAL_BLUR	3
int Filter_Selected = 0; // From 0 to 3, to select the filter used

//***FUNCIÓN ESTÁTICA: Trackbar para definir el kernel_length aplicado en los filtros
int Kernel_length_slider = 3; // Slider value
static void trackbar1_func(int, void*)
{
//Kernel_length_slider de 0 a 30 (Define cuándo se crea la barra de seguimiento
}

//***FUNCIÓN ESTÁTICA: Trackbar para definir el filtro utilizado
static void trackbar2_func(int, void*)
{
// Valor mínimo 1. Valor máximo 3 (Definido en)
     // Filtro_Seleccionado
}

//***CLASE: Image Conver (OpenCV)***
class ImageConverter
{
    private: 
    	// NodeHandle ROS
    	ros::NodeHandle nh_;

    	// Image used 
    	image_transport::ImageTransport it_; // Objeto it_ de la clase de transporte de imágenes (utilizado para el procesamiento de imágenes digitales)
    	image_transport::Subscriber topic1_sub__image_input; // Imagen obtenida de la cámara (sin procesar). Formato ROS (Tema)
    	image_transport::Publisher topic1_pub__image_output; // Imagen pública a ROS (procesada). Formato ROS (Tema)

    public:

	/* Método constructor.
	 */
  	ImageConverter() : it_(nh_)
  	{
    	  // Declaración de temas
       	    topic1_sub__image_input = it_.subscribe(TOPIC1_SUB__IMAGE_INPUT, 1, &ImageConverter::imageCb, this); 
   	    topic1_pub__image_output = it_.advertise(TOPIC1_PUB__IMAGE_OUTPUT, 1);

	   // Crear la GUI de Windows (donde imprimir las imágenes)
    	    cv::namedWindow(OPENCV_WINDOW1);
	    cv::namedWindow(OPENCV_WINDOW2);

	   // Crear una nueva barra de desplazamiento/barra de seguimiento
	    int trackbar1_maxValue = 50; // In units.
	    cv::createTrackbar("Kernel length [1-50]", OPENCV_WINDOW2, &Kernel_length_slider, trackbar1_maxValue, trackbar1_func);
// Comentarios: Ver el código "opencv_change_contrast_hh.cpp"
	    int trackbar2_maxValue = 3; // In units.
	    cv::createTrackbar("Filter [1-3]", OPENCV_WINDOW2, &Filter_Selected, trackbar2_maxValue, trackbar2_func);
  	}

/* Método Destructor */
  	~ImageConverter()
  	{
	   // cerrar la interfaz gráfica de usuario de Windows
    	    cv::destroyWindow(OPENCV_WINDOW1);
	    cv::destroyWindow(OPENCV_WINDOW2);
  	}
/* asociado a "TOPIC1_SUB__IMAGE_INPUT" que obtiene la imagen obtenida de la cámara (sin procesar) */
	void imageCb(const sensor_msgs::ImageConstPtr& msg) // msg es la imagen obtenida de la cámara (sin procesar)
  	{
	   // Convertir imagen ROS (Tema) a imagen OpenCV (Ptr)    
    	   
		    cv_bridge::CvImagePtr cv_OriginalImage_ptr;
    	    try
    	    {
      		cv_OriginalImage_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
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
   // Convertir ImagenOriginal a clase cv::Mat

	    cv::Mat cv_OriginalImage_Mat = cv_OriginalImage_ptr->image;
	    
// Nueva imagen a procesar de la clase cv::Mat
	    cv::Mat cv_ImageProcessed_Mat = cv_OriginalImage_Mat.clone();

	   //** Filtros aplicados **/
	    int l = kernel_format(Kernel_length_slider);
	   
	    switch(Filter_Selected)
	    {
		// Desenfoque homogéneo
		case 0:
		    cv::blur(cv_OriginalImage_Mat, cv_ImageProcessed_Mat, cv::Size(l, l), cv::Point(-1,-1));
	    	    ROS_INFO("FILTER APPLIED: ** 0. Homogeneous blur **");
		break;
	// Aplicando desenfoque gaussiano
		case GAUSSIAN_BLUR:
		    cv::GaussianBlur(cv_OriginalImage_Mat, cv_ImageProcessed_Mat, cv::Size(l, l), 0, 0);
		    ROS_INFO("FILTER APPLIED: ** 1. Gaussian blur **");
		break;
		// Aplicando desenfoque medio
		case MEDIAN_BLUR:
		    cv::medianBlur(cv_OriginalImage_Mat, cv_ImageProcessed_Mat, l);
		    ROS_INFO("FILTER APPLIED: ** 2. Median blur **");
		break;
		// Aplicando Filtro Bilateral
		case BILATERAL_BLUR:
		    cv::bilateralFilter(cv_OriginalImage_Mat, cv_ImageProcessed_Mat, l, l*2, l/2 );
		    ROS_INFO("FILTER APPLIED: ** 3. Bilateral blur **");
		break;	
	    }	
	    ROS_INFO("    KERNEL_LENGTH: %d x %d", l, l);
	    ROS_INFO(" ");
//** FIN: Filtros aplicados **/
	
	    /*********************************/ 
/* FIN: procesamiento de imagen digital */
	    /*********************************/

 // Actualizar GUI Window1 - Imagen original

   	    cv::imshow(OPENCV_WINDOW1, cv_OriginalImage_ptr->image);
    	    cv::waitKey(3);

// Actualizar ventana GUI 2 - Nueva imagen (contraste y brillo)   	   
		   cv::imshow(OPENCV_WINDOW2, cv_ImageProcessed_Mat);
	    cv::waitKey(3);

		// Convertir imagen OpenCV (Mat) a imagen OpenCV (Bridge) a imagen ROS (Tema)
		//puente_cv::ImagenCv cv_NuevaImagen; // es necesario usar la Clase CvImage no CvImagePtr
		//cv_ImagenNueva.header = cv_ImagenOriginal_ptr->header; // Misma marca de tiempo y marco tf que la imagen original. El seq es asignado por ROS automáticamente
		//cv_NewImage.encoding = cv_OriginalImage_ptr->encoding; // Mismo formato que la imagen original
		//cv_NuevaImagen.imagen = cvMat_NuevaImagen_ptr; // datos
			// Emitir flujo de video modificado
		//topic1_pub__image_output.publish(cv_NewImage.toImageMsg());
  	}
	
	/* Toma solo los números impares */
	int kernel_format(int value)
	{ 
	    if(value%2 == 0)
		return value + 1;
	    else 
		return value;
	}
};

//***Main***
int main(int argc, char** argv)
{
    // Init ROS 
    ros::init(argc, argv, NODE_NAME);
  
// Objeto de inicialización de la clase ImageConverter, definida anteriormente    ImageConverter ic;

// Mientras sea cierto. Obtener datos del tema de suscripción    ros::spin();
    return 0;
}
