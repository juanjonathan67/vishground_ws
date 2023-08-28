#include "mqtt_client/mqtt_client.hpp"

MQTTClient::MQTTClient(
  const std::string &address,
  const std::string &username,
  const std::string &password,
  const bool &clean_session,
  const int &keep_alive_interval,
  const int &timeout,
  const int &max_inflight
){
  cli = new mqtt::async_client(address, "");

  cli->start_consuming();

  conn_opts = mqtt::connect_options_builder()
    .user_name(username)
    .password(password)
    .keep_alive_interval(std::chrono::seconds(keep_alive_interval))
    .clean_session(clean_session)
    .connect_timeout(std::chrono::seconds(timeout))
    .max_inflight(max_inflight)
    .finalize();

  try{
    std::cout << "Connecting to the MQTT server..." << std::endl;

    cli->connect(conn_opts)->wait();
    
    std::cout << "Connected to the MQTT server" << std::endl;

  }catch(const mqtt::exception &exc){
    std::cerr << exc << std::endl;
  }
}

MQTTClient::~MQTTClient(){
  std::cout << "Disconnecting from the MQTT server..." << std::endl;
  cli->disconnect()->wait();
  std::cout << "Disconnected from the MQTT server" << std::endl;
}

mqtt::topic MQTTClient::get_publisher(const std::string &topic){
  std::cout << "Getting publisher " << topic << std::endl;
  std::map<std::string, mqtt::topic *>::iterator it = publishers.find(topic);
  if(it != publishers.end()){
    return *(it->second);
  }else{
    std::cerr << "Publisher not found" << std::endl;
    return *(new mqtt::topic(*cli, "/error", 0, true));
  }
}

void MQTTClient::create_publisher(const std::string &topic, const int &qos){
  publishers[topic] = new mqtt::topic(*cli, topic, qos, true);
}

void MQTTClient::create_subscriber(const std::string &topic, const int &qos){
  cli->subscribe(topic, qos)->wait();
}

void MQTTClient::subscribe(){
  mqtt::const_message_ptr msg = cli->consume_message();
  std::cout << msg->get_topic() << " : " << msg->to_string() << std::endl;
}