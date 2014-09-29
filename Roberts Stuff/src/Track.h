#ifndef __TRACK_H__
#define __TRACK_H__

#include <iostream>
#include "HandTrackingClient.h"
#include "HandTrackingListener.h"

//Get functions and use instructions are near the bottom of this file.
//
//Hand tracking code use example, this code will continuously print x coordinate of last left hand point:
//
//
//#include "Track.h"
//#include "HandTrackingClient.h"
//#include <cstdio>
//int main()
//{
//  HandTrackingClient::Client client;
//	Track *track = new Track();
//  client.addHandTrackingListener(track);
//	client.connect();
//	while(1)
//	{
//		std::cout << track->getIndexPoint(0).x << std::endl;
//	}
//    if(getchar())
//        client.stop();
//    return 0;
//}


class Track : public HandTrackingClient::HandTrackingListener
{
private:
	std::array<HandTrackingClient::Vector3f, 3> pinches;
	std::array<HandTrackingClient::Vector3f, 3> releases;
	std::array<HandTrackingClient::Vector3f, 3> indexPoints;
	std::array<std::array<HandTrackingClient::Transformf, 17>, 3> joints;

public:
    Track(){}

    virtual void handleEvent(const HandTrackingClient::HandTrackingMessage& baseMessage)
    {
        using HandTrackingClient::HandTrackingMessage;
		using HandTrackingClient::PinchMessage;
		using HandTrackingClient::PoseMessage;

		const int pointing = HandTrackingClient::PoseMessage::HandPose::HAND_POINTING;
		const float poseThreshold = 0.2;

		if (baseMessage.getType() == HandTrackingMessage::PRESSED)
        {
            const PinchMessage& message = dynamic_cast<const PinchMessage&>(baseMessage);

			int hand = message.getHand();

            HandTrackingClient::HandState state = message.getHandState(hand);
            HandTrackingClient::Vector3f position = state.getPosition();
			pinches[hand] = position;
        }
		else if (baseMessage.getType() == HandTrackingMessage::RELEASED)
        {
            const PinchMessage& message = dynamic_cast<const PinchMessage&>(baseMessage);

			const int hand = message.getHand();

            HandTrackingClient::HandState state = message.getHandState(hand);
            HandTrackingClient::Vector3f position = state.getPosition();
			releases[hand] = position;
        }
		else if (baseMessage.getType() == HandTrackingMessage::POSE)
        {
            const PoseMessage& message = dynamic_cast<const PoseMessage&>(baseMessage);

			const std::array<float, 7> leftPoses = message.getHandPoseConfidences(0);
			const std::array<float, 7> rightPoses = message.getHandPoseConfidences(1);

			joints[0] = message.getJointTransforms(0);
			joints[1] = message.getJointTransforms(1);

			if (leftPoses[pointing] > poseThreshold) 
			{
				indexPoints[0] = message.getFingerTips(0)[1];
			}
			if (rightPoses[pointing] > poseThreshold)
			{
				indexPoints[1] = message.getFingerTips(1)[1];
			}
        }
    }

    virtual void handleConnectionClosed() { }

	
	//Returns most recent pinch location as an <x, y, z> array for given hand (0 = left, 1 = right).
	virtual HandTrackingClient::Vector3f getPinchPoint(int hand)
	{
		return pinches[hand];
	}

	//Returns most recent pinch release (unpinch) location as an <x, y, z> array for given hand (0 = left, 1 = right).
	virtual HandTrackingClient::Vector3f getReleasePoint(int hand)
	{
		return releases[hand];
	}

	//Returns most recent point location as an <x, y, z> array for given hand (0 = left, 1 = right).
	virtual HandTrackingClient::Vector3f getIndexPoint(int hand)
	{
		return indexPoints[hand];
	}

	//Returns most recent location + rotation of hand joints as a transform for given hand (0 = left, 1 = right) 
	//and joint, joint = HandTrackingClient::JointFrameIndex::JOINT_NAME (e.g. WRIST_JOINT)
	//	see http://www.threegear.com/latest/doc/handModel.html (Figure 3)
	//
	//transform[joint].rotation returns joint rotations as a quaternion.
	//transform[joint].translation returns joint locations as <x, y, z> array
	//
	//e.g. getJointPoint(0)[HandTrackingClient::JointFrameIndex::WRIST_JOINT].translation.x
	//returns x coordinate of the wrist joints last known location.
	virtual std::array<HandTrackingClient::Transformf, 17> getJointPoint(int hand)
	{
		return joints[hand];
	}

};


#endif // __TRACK_H__
