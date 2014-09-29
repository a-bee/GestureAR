#include "Track.h"
#include "HandTrackingClient.h"

#include <cstdio>
int main()
{
    HandTrackingClient::Client client;
	Track *track = new Track();
    client.addHandTrackingListener(track);
	client.connect();

	while(1)
	{
		std::cout << track->getIndexPoint(0).x << std::endl;
	}

    if(getchar())
        client.stop();
    return 0;

}
