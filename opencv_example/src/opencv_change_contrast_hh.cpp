/* EXAMPLE 3 USING OPENCV
/*
/*   Base: OpenCV tutorial: "Changing the contrast and brightness of an image!"
/*   from = https://docs.opencv.org/2.4/doc/tutorials/core/basic_linear_transform/basic_linear_transform.html#basic-linear-transform
/*
/*   For implement the scrollbar/trackbar was used the tutorial: "Adding a Trackbar to our applications!"
/*   From = https://docs.opencv.org/2.4/doc/tutorials/highgui/trackbar/trackbar.html
/*
/*   it was modificated by Hernán Hernández to be use like own template
*/

// Includes - Incluir librerías
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Defines - General
#define NODE_NAME "opencv_change_contrast_hh" // Nombre del nodo
#define OPENCV_WINDOW1 "Original Image"
#define OPENCV_WINDOW2 "New Image (Contrast & brightness)"

// Define Topicos para obtener la imagen de la cámara
#define TOPIC1_SUB__IMAGE_INPUT "/usb_cam/image_raw"			 // Imagen obtenida de la cámara (sin procesar).
#define TOPIC1_PUB__IMAGE_OUTPUT "/image_converter/output_video" // Imagen pública a ROS (procesada).

//*** FUNCIÓN ESTÁTICA: Método de barra de seguimiento para valor alfa. Interfaz gráfica de usuario de OpenCV***
double Alpha = 1.5;	  // Variable de control del contraste
int trackbar1_slider; // Variable para almacenar valor real de la barra

// Función estática para la barra
static void trackbar1_func(int, void *)
{
	// Escalar el valor de la barra de seguimiento [0 - 100%] al valor alfa [0.0 - 3.0]
	Alpha = trackbar1_slider * 3.0 / 100.0;
}

int Beta = 30;		  // Control de brillo simple. Valor de 0 a 100
int trackbar2_slider; // Donde se almacena el valor real de la barra de seguimiento

static void trackbar2_func(int, void *)
{
	// Change Beta value
	Beta = trackbar2_slider;
}

//***Clase: Image Conver (OpenCV)***
class ImageConverter
{
private:
	// NodeHandle ROS
	ros::NodeHandle nh_;

	// Imagen usado
	image_transport::ImageTransport it_;				 // Objeto it_ de la clase de transporte de imágenes (utilizado para el procesamiento de imágenes digitales)
	image_transport::Subscriber topic1_sub__image_input; // Imagen obtenida de la cámara (sin procesar). Formato ROS (Tema)
	image_transport::Publisher topic1_pub__image_output; // Imagen pública a ROS (procesada). Formato ROS (Tema)

public:
	/* Metodo constructor de la clase
	   TODO */
	ImageConverter() : it_(nh_)
	{
		// Declaración de topicos
		topic1_sub__image_input = it_.subscribe(TOPIC1_SUB__IMAGE_INPUT, 1, &ImageConverter::imageCb, this);
		topic1_pub__image_output = it_.advertise(TOPIC1_PUB__IMAGE_OUTPUT, 1);

		// Creación de la GUI para presentar la imagen
		cv::namedWindow(OPENCV_WINDOW1);
		cv::namedWindow(OPENCV_WINDOW2);

		// Crear una nueva barra de desplazamiento/barra de seguimiento
		int trackbar_maxValue = 100;																				// Valor en Porcentaje
		cv::createTrackbar("Alpha [0-100%]", OPENCV_WINDOW2, &trackbar1_slider, trackbar_maxValue, trackbar1_func); // // Tenga en cuenta lo siguiente: 1) Nuestro Trackbar tiene una etiqueta "Alfa", 2) El Trackbar se encuentra en la ventana "OPENCV_WINDOW2", 3) Los valores del Trackbar estarán en el rango de 0 a "trackbar_maxValue" (el límite mínimo siempre es cero), 4) El valor numérico de Trackbar se almacena en "trackbar_slider", y 5) Cada vez que el usuario mueve la Trackbar, se llama a la función de devolución de llamada on_trackbar
		cv::createTrackbar("Beta [0-100%]", OPENCV_WINDOW2, &trackbar2_slider, trackbar_maxValue, trackbar2_func);
	}

	/*Metodo destructor*/
	~ImageConverter()
	{
		// Cierra la GUI Windows
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
		cv::Mat cvMat_Image_ptr = cv_OriginalImage_ptr->image;

		// Realiza la operación new_image(i,j) = Alpha*Orginal_image(i,j) + Beta
		cv::Mat cvMat_NewImage_ptr = cv::Mat::zeros(cvMat_Image_ptr.size(), cvMat_Image_ptr.type()); // Matriz de 0 del mismo tamaño
		for (int y = 0; y < cvMat_Image_ptr.rows; y++)
		{
			for (int x = 0; x < cvMat_Image_ptr.cols; x++)
			{
				for (int c = 0; c < 3; c++)
				{
					cvMat_NewImage_ptr.at<cv::Vec3b>(y, x)[c] =
						cv::saturate_cast<uchar>(Alpha * (cvMat_Image_ptr.at<cv::Vec3b>(y, x)[c]) + Beta);
				}
			}
		}
		/*********************************/
		/* FIN: procesamiento de imagen digital */
		/*********************************/

		// Actualizar GUI Window1 - Imagen original
		cv::imshow(OPENCV_WINDOW1, cv_OriginalImage_ptr->image);
		cv::waitKey(3);

		// Actualizar ventana GUI 2 - Nueva imagen (contraste y brillo)
		cv::imshow(OPENCV_WINDOW2, cvMat_NewImage_ptr);
		ROS_INFO("Alpha %f ------ Beta %d", Alpha, Beta);
		cv::waitKey(3);

		// Convertir imagen OpenCV (Mat) a imagen OpenCV (Bridge) a imagen ROS (Tema)
		cv_bridge::CvImage cv_NewImage;						   // es necesario usar la Clase CvImage no CvImagePtr
		cv_NewImage.header = cv_OriginalImage_ptr->header;	   // Misma marca de tiempo y marco tf que la imagen original. El seq es asignado por ROS automáticamente
		cv_NewImage.encoding = cv_OriginalImage_ptr->encoding; // Mismo formato que la imagen original
		cv_NewImage.image = cvMat_NewImage_ptr;				   // datOS
															   // Emitir flujo de video modificado
		topic1_pub__image_output.publish(cv_NewImage.toImageMsg());
	}
};

// Función principal
int main(int argc, char **argv)
{
	// Inicializa ros
	ros::init(argc, argv, NODE_NAME);

	// Objeto de inicio de clase, en la parte superior se define la clase ImageConverter
	ImageConverter ic;

	// Obtener datos del tema de suscripción, si ros está ejecutandose o sea es True
	ros::spin();
	return 0;
}
