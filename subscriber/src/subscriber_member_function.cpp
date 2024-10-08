#include <functional>
#include <memory>
#include <chrono>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"

#include "cipher_interfaces/srv/cipher_answer.hpp" 
#include "cipher_interfaces/msg/cipher_message.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("subscriber_action_caller")
  {
   	this->declare_parameter("name_of_topic", "topic");
   	this->declare_parameter("name_of_service", "cipher");
   	std::string name_of_topic = this->get_parameter("name_of_topic").as_string(); 
    subscription_ = this->create_subscription<cipher_interfaces::msg::CipherMessage>( 
      name_of_topic, 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
	//this runs when code heard something in the given topic!!
  void topic_callback(const cipher_interfaces::msg::CipherMessage & msg) const 
  {
  	//retrieve message and key from topic
 		std::string encoded_message = msg.message;
  	int key = msg.key;
  	
  	//log to console (not actually console more like ros console but it will end up in the console)
    RCLCPP_INFO(this->get_logger(), "Coded message is: '%s'", encoded_message.c_str());
    RCLCPP_INFO(this->get_logger(), "Key is: '%i'", key);
    
    //decode message
    
    //loop through each char in the encoded_msg string..
    for (unsigned int i = 0; i < encoded_message.length(); i++) {
    
    	//safeguard for upper case
    	bool upper = false;
    	
    	if (encoded_message[i] >= 'A' && encoded_message[i] <= 'Z') {
    		encoded_message[i] = tolower(encoded_message[i]);
    		upper = true;
    	}
    	
    	//do inverse of key
    	encoded_message[i]-=key;
    	
    	//check if went under/over
    	while (encoded_message[i] < 'a') {
    		encoded_message[i]+= 27;
    	}
    	while(encoded_message[i] > 'z') {
    		encoded_message[i]+= 27;
    	}
    	
    	//if was uppercase return to uppercase
    	if (upper) {
    		encoded_message[i] = toupper(encoded_message[i]);
    	}	
    }
    
    RCLCPP_INFO(this->get_logger(), "Decoded message is: '%s'", encoded_message.c_str());
     
    //now encoded_message stores decoded message
    
    
    //now do client call
    //first get name of service to call to from parameter (set in .yaml launch file)
    std::string service_name = this->get_parameter("name_of_service").as_string();
    
    //create the node from which to call (tried to it from same node but couldnt pull it off)
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("cipher_client_node");    
 		rclcpp::Client<cipher_interfaces::srv::CipherAnswer>::SharedPtr client =              
    	node->create_client<cipher_interfaces::srv::CipherAnswer>(service_name); 
  
  //create request body
    auto request = std::make_shared<cipher_interfaces::srv::CipherAnswer::Request>();     
    request->answer = encoded_message;
    // request->header = idk
    
    while (!client->wait_for_service(1s)) {
    	if (!rclcpp::ok()) {
	      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
	      return;
	    }
	    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  	}

		auto result = client->async_send_request(request);
		// Wait for the result.
		if (rclcpp::spin_until_future_complete(node, result) ==
			rclcpp::FutureReturnCode::SUCCESS)
		{
			//if in here, then server returned
			if(result.get()->result) {
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Decripting was correct");
			} else {
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Decripting failed");
			}
		} else { 	
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_three_ints");    
		}

		//shutdown
		rclcpp::shutdown();
	}
  
  	rclcpp::Subscription<cipher_interfaces::msg::CipherMessage>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
  	rclcpp::spin(std::make_shared<MinimalSubscriber>());
  }
  catch (const std::runtime_error& e) {
  	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "About to kill this mf");
  }
  	
  rclcpp::shutdown();
}
