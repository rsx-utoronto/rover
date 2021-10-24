#ifndef PUBLISHER_SUBSCRIBER_H
#define PUBLISHER_SUBSCRIBER_H

#include <ros/ros.h>
#include <string>

// Basic generic template to allow a node to both publish and subscribe
// Can be re-used with different message types :)

template<typename PublishT, typename SubscribeT>
class PublisherSubscriber{
	public:
		PublisherSubscriber(){}
		PublisherSubscriber(std::string publisher_topic, std::string subscriber_topic, int queue_size){
			publisherObject = nh.advertise<PublishT>(publisher_topic, queue_size);
			subscriberObject = nh.subscribe<SubscribeT>(subscriber_topic, queue_size, &PublisherSubscriber::subscriberCallback, this);
		}
		void subscriberCallback(const typename SubscribeT::ConstPtr& recievedMsg);


	// subscriber and publisher objects
	protected:
		ros::Subscriber subscriberObject;
		ros::Publisher publisherObject;
		ros::NodeHandle nh;
};

#endif