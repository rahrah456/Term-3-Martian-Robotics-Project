#include <MiniMessenger.h>
#include "secrets.h"

/*
Important Tips

messenger.loop(): This MUST be called frequently in your loop() function to handle incoming messages and maintain the connection.
Group ID: Ensure your GROUP_ID is unique to your team by using your assigned team number.
Board IDs: Here, you define your robot's actual fancy name!!! Please do not use spaces, and keep names short.
Testing your Robot Communication

Once you have uploaded your code to the robot, connect your laptop/phone/neuralink device to the same network as the robot (WiFi SSID and password are written on the whiteboard in the lab). 
Once connected, you can access via a web browser your team's dashboard. It should be the following: http://192.168.0.74:8090/team/<your_team_number_here(GROUP)ID>

Here you will see a dashboard similar to what the main command control will be using. Try the enable/disable button to check your code reacts appropriately. 
You can even maybe try sending commands to verify RFID tags 😉 check the Message Dictionary on the repo .

Happy Coding! 🤖

*/

MiniMessenger messenger;
const char* BoardId = "Unknown";  // Remember to choose a name for your robot :)

// Callback function that runs whenever a new message arrives
void onMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
  Serial.print("Message from Board ");
  Serial.print(metadata.fromBoardId);
  Serial.print(": ");
  
  // Print the payload as text
  for (size_t i = 0; i < length; i++) {
    Serial.write(payload[i]);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  
  // Register the callback function
  messenger.onMessage(onMessage);

  // Initialize messenger
  messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, BoardId);

  Serial.println("Messenger Ready!");
}

void loop() {
  // Critical: Keep the messenger connection alive
  messenger.loop();

  static unsigned long lastSend = 0;
  if (messenger.isConnected() && millis() - lastSend > 5000) {
    lastSend = millis();
    
    // Send a message to Board "2"
    messenger.sendToBoard("2", "Hello from Unknown!"); // Send whatever message you want here
    Serial.println("Message sent!");
  }
}