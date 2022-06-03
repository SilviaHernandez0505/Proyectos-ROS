/* Template to use Open Cv inside ROS.
/*
/*   ROS tutorial
/*   it was download from = http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

// CLASE IMAGE CONVERTER
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  // METODO CONSTRUCTOR
  ImageConverter()
      : it_(nh_)
  {
    // Suscríbase a la fuente de video de entrada y publique la fuente de video de salida
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
                               &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }
  // METODO DESTRUCTOR
  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr &msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Dibujar un círculo de ejemplo en la transmisión de video
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));

    // Actualizar ventana GUI
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3); // Evento de teclado para cerrar ventana

    // Emitir flujo de video modificado
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

// Función principal
int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_converter"); // Inicializa ros
  ImageConverter ic;                        // Objeto de la clase definidata anteriormente
  // Mantiene el nodo funcionando
  ros::spin();
  return 0;
}
