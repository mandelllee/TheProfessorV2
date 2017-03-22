//char ch;

//
//
//String sendATCommand(String command, String expectedResponse, int timeout ) {
//  Serial.println("> " + command );
//  addTextToDisplay( "> " + command + "\n" );
//  int timeoutTime = millis() + timeout;
//  String response = "";
//  int count = 0;
//  int len = expectedResponse.length();
//
//  BT.print( command );
//
//  while (1) {
//
//    if ( millis() >= timeoutTime ) {
//
//      addTextToDisplay( "Timeout.\n" );
//      Serial.println("Command timeout.");
//      break;
//      return "";
//    }
//    if ( BT.available() ) {
//      ch = BT.read();
//      count++;
//      response += ch;
//      Serial.print( "[" + String(ch) + "]" );
//      if ( count == len ) {
//        addTextToDisplay( "" + response );
//        Serial.print( "\n" );
//        break;
//      }
//    }
//    delay(10);
//  }
//
//  Serial.println("response: " + response );
//
//  return response;
//}
//
void setupBluetooth() {
//
//  displayTextOnDisplay("initializing\nBluetooth...\n");
//
//  Serial.println("initializing Bluetooth...");
//  // set the data rate for the SoftwareSerial port
//  BT.begin(9600);
//
//  delay( 1 * 1000);
//  //
//  //  BT.println( "AT" );
//  //  delay(1000);
//  //  BT.println( "AT+NAME" );
//  //  delay(1000);
//  //
//  String ok = sendATCommand( "AT", "OK", 10 * 1000 );
//  if ( ok == "OK") {
//    bluetoothAvailable = true;
//    Serial.println("Found BT module.");
//    Serial.println("Setting name: " + String(_hostname) );
//    addTextToDisplay( " + Found Adapter" );
//    addTextToDisplay( " + Setting name to [" + String(_hostname) + "]" );
//
//    String r = sendATCommand( "AT+NAME" + String(_hostname), "OKsetname", 2000 );
//    //BT.print("AT+NAME" + String(_hostname)  );
//    //BT.print("AT+NAME");
//    //delay(600);
//    Serial.print("BT Name: " + r );
//    addTextToDisplay( "response: " + String( r ) );
//
//    String b = sendATCommand( "AT+BAUD8", "OKsetbaud", 2000 );
//
//    // Send test message to other device
//    //BT.println( getJSONStatus() );
//  } else {
//    Serial.println("BT not available");
//  }
//  delay( 1000 );
}
//
//
//
//
//char a; // stores incoming character from other device
//String bt_command = "";
//String bt_string = "";
//
//bool haveCommand = false;
//bool btWelcomeSent = false;
//void handleBluetooth()
//{
//  if (BT.available()) {
//    a = (BT.read());
//
//    if ( a == '\n' ) {
//      haveCommand = true;
//    } else {
//      bt_command += a;
//      //Serial.println("BT data: [" + String(a) + "]");
//    }
//  }
//
//  if ( haveCommand ) {
//    Serial.println( "BT RECIEVED: " + bt_command );
//    haveCommand = false;
//
//    delay(10);
//    //BT.println( bt_command );
//    bt_command.replace( "\n", "");
//    bt_command.replace( "\r", "");
//
//    if ( bt_command == "?\n" || bt_command == "?" ) {
//      Serial.println("Status COMMAND DETECTED");
//      BT.println( getJSONStatus() );
//
//    } else if ( bt_command == "AT\n" || bt_command == "AT" ) {
//
//      BT.println("AT COMMAND DETECTED");
//      delay(250);
//    } else if ( bt_command == "TOGGLE" || bt_command == "T" ) {
//
//      BT.println("TOGGLE COMMAND DETECTED");
//      toggleAllSwitches();
//      BT.print("New state: " + String(switch_state) );
//
//    } else {
//
//      BT.println( getJSONStatus("unknown command: " + bt_command ) );
//    }
//
//    bt_string = bt_command;
//    a = 0;
//    bt_command = "";
//  }
//
//}
