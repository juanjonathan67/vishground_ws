#include "mqtt/async_client.h"

#include <iostream>
#include <chrono>
#include <string>
#include <map>

class MQTTClient{
  public: 
    MQTTClient(
      const std::string &address = "tcp://hairdresser.cloudmqtt.com:17900",
      const std::string &username = "aoaljzzp",
      const std::string &password = "U1rrWLMwGVo6",
      const bool &clean_session = true,
      const int &keep_alive_interval = 20,
      const int &timeout = 10,
      const int &max_inflight = 10
    );

    ~MQTTClient();

    mqtt::topic get_publisher(const std::string &topic);

    void create_publisher(const std::string &topic, const int &qos = 1);

    void create_subscriber(const std::string &topic, const int &qos = 0);

    void subscribe();

  private:
    std::map<std::string, mqtt::topic *> publishers;

    mqtt::async_client *cli;
    mqtt::connect_options conn_opts;
    mqtt::token_ptr tkn_ptr;

    

};