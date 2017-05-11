/**
 * Written by Joris Guerry (joris.guerry@onera.fr) 2015/09
 * with snippets of code from:
 * http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
 * published under license: CC BY 3.0 http://creativecommons.org/licenses/by/3.0/
 *
 * Modified:
 * 2015/09 timestamps in filenames / Bertrand le Saux (bertrand.le_saux@onera.fr)
 * 2016/09 adding buffering / Joris Guerry (joris.guerry@onera.fr)
 */


#include <boost/thread/thread.hpp>
#include <boost/filesystem.hpp>

// pour ROS et OpenCV
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>




// pour le traitement

#include <stdio.h>
#include <dirent.h>
#include <ios>
#include <stdexcept>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>


// Les classiques
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdlib.h>




using namespace std;
using std::string;
using namespace boost::filesystem;

class ExportImage
{
		ros::NodeHandle nh;
		image_transport::ImageTransport it;
		image_transport::Subscriber sub_video_png_input;
		int counter;
		int tampon;
    double timebase;
		string flux;
		string path;
		string format;




	public:
  ExportImage( string _flux, string _path, string _fmt, int _tampon ): it( nh ) //constructeur qui initialise un objet ImageTransport (it) avec la handle du noeud ROS (nh)
		{

			flux = _flux;
			path = _path;
      format = _fmt;
			tampon = _tampon;
			boost::filesystem::path dir( _path.c_str() );
			if( boost::filesystem::create_directory( dir ) )
			{
				std::cerr << "Directory Created: " << _path << std::endl;
			}



			sub_video_png_input = it.subscribe( flux, tampon, &ExportImage::video_png_input_Cb, this );

			counter = 0;
			timebase = 0;




		}

		~ExportImage()
		{
			cout << "Compteur : " << counter << " images sauvegardées." << endl;
		}

		void video_png_input_Cb( const sensor_msgs::ImageConstPtr& msg )
		{
      //
			if (counter==0) {
        cout << "msg->encoding :" << msg->encoding << endl;
      }
	    // cv_bridge::CvImageConstPtr cv_ptr;
	    cv_bridge::CvImagePtr cv_ptr;
			try
			{
          sensor_msgs::Image img;
          img.header = msg->header;
          img.height = msg->height;
          img.width = msg->width;
          img.is_bigendian = msg->is_bigendian;
          img.step = msg->step;
          img.data = msg->data;
          // img.encoding = "mono16";
          // cv_ptr = cv_bridge::toCvCopy( img , sensor_msgs::image_encodings::MONO16 );
          img.encoding = "32FC1";
          cv_ptr = cv_bridge::toCvCopy( img , sensor_msgs::image_encodings::TYPE_32FC1 );

					// scale from 0-8000mm to 0 255 uint.
					// cv_ptr->image
					// THRESH_TOZERO_INV


					float min_disparity = 0;
					float max_disparity = 10;// max value in meter before tresholding to zero
					float multiplier = 255.0f / (max_disparity - min_disparity);
					cv::threshold(cv_ptr->image, cv_ptr->image, max_disparity, 0, cv::THRESH_TOZERO_INV);
					cv_ptr->image=multiplier*cv_ptr->image;

          // cv_ptr = cv_bridge::toCvCopy( img , sensor_msgs::image_encodings::CV_8UC1 );

			}
			catch( cv_bridge::Exception& e )
			{
				ROS_ERROR( "cv_bridge exception: %s", e.what() );
				return;
			}
			counter++;




      /* with timebase, understandable timestamp */
      if (counter==1) {
        timebase=0 ;//msg->header.stamp.toSec();
      }
      double timestampsec = msg->header.stamp.toSec() - timebase;
      unsigned int timestamp = static_cast<unsigned int> ( timestampsec * 100000);

                        /* timestamp based on micro-seconds timestamp since beginning */
			std::stringstream saveLocationFlux;
			cout << path << "/" << std::setw( 7 ) << std::setfill( '0' ) << counter << "-" << std::setw( 12 ) << std::setfill( '0' ) << timestamp << "." << format << "\xd";
			saveLocationFlux << path << "/" << std::setw( 7 ) << std::setfill( '0' ) << counter << "-" << std::setw( 12 ) << std::setfill( '0' ) << timestamp << "." << format;

			std::string saveLocation = saveLocationFlux.str();

      if ( (!format.compare(string("png"))) || (!format.compare(string("jpg"))) || (!format.compare(string("jpeg"))) ) {
        cv::imwrite( saveLocation, cv_ptr->image );
      } else {
        ROS_ERROR( "cv::imwrite error: unknown image format" );
        return;
      }

		}
};

int main( int argc, char** argv )
{
	std::cout << "###########################################################################" << std::endl << std::endl ;
	std::cout << "Export image" <<  std::endl << std::endl;
	std::cout << "###########################################################################" << std::endl << std::endl ;


	std::string flux;
	std::string path;
	std::string format;
	int tampon;

	ros::init( argc, argv, "ExportImageDepth" );
	if( argc == 4 )
	{
		format = "png";
		flux = argv[1];
		path = argv[2];
		tampon = std::stoi(argv[3]);
	}
	else
	{
		ROS_ERROR( "Parameters invalid" );
		return 1;
	}



	std::cout <<"Save " << flux << " to " << path << " ["<< format <<"] with a buffer size of " << tampon <<"."<< std::endl;


	ExportImage toImage( flux, path, format, tampon );
	ros::spin();

	return 0;




}
