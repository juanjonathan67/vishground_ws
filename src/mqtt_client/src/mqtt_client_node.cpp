#include <iostream>
#include <cstdlib>
#include <string>
#include <thread>	// For sleep
#include <atomic>
#include <chrono>
#include <cstring>
#include "mqtt/async_client.h"

using namespace std;

const string DFLT_SERVER_ADDRESS { "tcp://hairdresser.cloudmqtt.com:17900" };

const string TOPIC { "test" };
const int QOS = 1;

const char* PAYLOADS[] = {
	"Hello World!",
	"Hi there!",
	"Is anyone listening?",
	"Someone is always listening.",
	nullptr
};

const auto TIMEOUT = std::chrono::seconds(10);

/////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[])
{
	string address = (argc > 1) ? string(argv[1]) : DFLT_SERVER_ADDRESS;

	cout << "Initializing for server '" << address << "'..." << endl;
	mqtt::async_client cli(address, "");

	cout << "  ...OK" << endl;

    auto connOpts = mqtt::connect_options_builder()
        .user_name("aoaljzzp")
        .password("U1rrWLMwGVo6")
        .keep_alive_interval(std::chrono::seconds(20))
        .clean_session()
        .finalize();

	try {
		cout << "\nConnecting..." << endl;
		cli.connect(connOpts)->wait();
		cout << "  ...OK" << endl;

		cout << "\nPublishing messages..." << endl;

		mqtt::topic top(cli, "test", QOS);
		mqtt::token_ptr tok;

		size_t i = 0;
		while (PAYLOADS[i]) {
			tok = top.publish(PAYLOADS[i++]);
		}
		tok->wait();	// Just wait for the last one to complete.
		cout << "OK" << endl;

		// Disconnect
		cout << "\nDisconnecting..." << endl;
		cli.disconnect()->wait();
		cout << "  ...OK" << endl;
	}
	catch (const mqtt::exception& exc) {
		cerr << exc << endl;
		return 1;
	}

 	return 0;
}