// ROS node that decompresses incoming compressed image messages

// The general outline is as follows :
// 1. Turns Compressed Image into AVPacket
// 2. Decodes AVPacket
// 3. Inflates AVFrame with AVPacket info
// 4. Publishes AVFrame as Image message


#include <ros/ros.h>

// ffmpeg libs to decompress images
extern "C"
{
	#include <libavcodec/avcodec.h>
	#include <libswscale/swscale.h>
}

// standard for image transport in ROS
// allows a node to be both subscriber/publisher
#include <image_transport/image_transport.h>


#include <sensor_msgs/CompressedImage.h>

// used by image transport
ros::Publisher pub;

// the h264 codec used to compress/decompress
AVCodecContext *codec = 0;
SwsContext *sws = 0;

void callback(const sensor_msgs::CompressedImageConstPtr& img){


	// init the AVPacket with compressed image info
	AVPacket avpkt;
	av_init_packet(&avpkt);
	avpkt.data = const_cast<uint8_t*>(img->data.data());
	avpkt.size = img->data.size();
	avpkt.pts = AV_NOPTS_VALUE;
	avpkt.dts = AV_NOPTS_VALUE;


	// AVFrame to contain our decompressed image
	AVFrame picture;
	memset(&frame, 0, sizeof(frame));

	int got_picture;

	// decode the frame
	if(avcodec_decode_video2(codec, &picture, &got_picture, &avpkt) < 0){
		av_free_packet(&avpkt);
		return;

	}


	// if ok, make new Image message and publish
	if(got_picture){

		sensor_msgs::ImagePtr img(new sensor_msgs::Image);

		img->encoding = "rgb8";
		img->data.resize(picture.width * picture.height * 3);
		img->step = picture.width * 3;
		img->width = picture.width;
		img->height = picture.height;
		img->header.frame_id = "stream";
		img->header.stamp = ros::Time::now();

		pub.publish(img);

	}


}

int main(int argc, char** argv){

	ros::init(argc, argv, "receive_stream");

	ros::NodeHandle nh("~");

	avcodec_register_all();
	av_log_set_level(AV_LOG_QUIET);

	AVCodec* decoder = avcodec_find_decoder(AV_CODEC_ID_H264);

	codec = avcodec_alloc_context3(decoder);
	codec->flags |= CODEC_FLAG_LOW_DELAY;
	codec->flags2 |= |= CODEC_FLAG2_SHOW_ALL;

	codec->thread_type = 0;

	if(avcodec_open2(codec, decoder, 0) != 0)
		return -1;

	pub = nh.advertise<sensor_msgs::Image>("image", 1);
	ros::Subscriber sub = nh.subscribe("compressed", 5, &callback);

	ros::spin();

	return 0;
}