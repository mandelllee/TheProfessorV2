

String urlRequest( String host, String url, int httpPort ) {

  //String url = "/garden/garden.php?uid=" + _userid + "&action=ph&value=" + String(input) + "&tempc=" + String(temp_c) + "&vcc=" + String(voltValue) + "";
  //char host[] = "gbsx.net";

  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  if (!client.connect(host.c_str(), httpPort)) {
    Serial.println("connection failed");
    return "-1";
  }

  // We now create a URI for the request
  Serial.print("Requesting URL: ");
  Serial.println(host + url);

  // This will send the request to the server
  client.print( String("GET ") + url + " HTTP/1.1\r\n" +
                "Host: " + host + "\r\n" +
                "Connection: close\r\n\r\n");
  delay(10);

  String response = "";
  String headers = "";
  bool pastHeaders = false;
  int rn = 0;
  // Read all the lines of the reply from server and print them to Serial
  while (client.available()) {
    String line = client.readStringUntil('\r');
    //Serial.print(line);

    if ( line.equals("\n") ) {
      pastHeaders = true;
    }
    
    line.trim();

    if (
      line.length() == 0
//      || line.equals("\n")
//      || line.equals("a")
//      || line.equals("a\n")
//      || line.equals("a\r")
//      || line.equals("0")
//      || line.equals("0\n")
//      || line.equals("0\r")
    )
    {
      Serial.println("skipping:" + line );

    } else {
      if ( pastHeaders ) {

        if( rn==1 ){
          response += line;
        }
        Serial.println( "[" + String(rn) + "]" + line );
        
        rn++;
      } else {
        headers += line;
        //response="";
      }
    }
  }// end response processing

  Serial.println( "RESPONSE=[" + response + "]" );
  //  Serial.println("closing connection");
  return response;
}



int _now = 0;
/**
   This will set the value _now using the api

 **/
void getTime() {

  String url = "/now";
  String response = urlRequest( API_HOST, url, API_PORT );
  _now = response.toInt();

  Serial.println( "NOW=[" + String(_now ) + "]");
}
void provisionDevice() {

  String url = "/v1/provision?type=node";

  url += "&nodeid=" + BOARD_ID;
  url += "&platform=arduino";
  url += "&hardware=ESP8266";
  url += "&boardid=" + chip_id;

  url += "&core_version=" + VERSION;
  url +=  + "&boardname=" + _hostname;

  Serial.println( String(API_HOST ) + "" + url );

  urlRequest( API_HOST, url, API_PORT );
}

void recordValue(  String ty, String propertyname, String value, String id_value ) {

  if ( SEND_DATA_TO_API == false ) {
    Serial.println("API disbaled");
    return;
  }
  //char host[] = "10.5.1.25";
  //  char host[] = API_HOST;

  String url = "/v1/record?type=" + String(ty) + "&propertyname=" + String(propertyname)  + "&value=" + String(value) + "&id=" + String(id_value) + "&boardid=" + BOARD_ID
               + "&core_version=" + VERSION;
  url +=  + "&boardname=" + _hostname;

  Serial.println( String(API_HOST ) + "" + url );

  urlRequest( API_HOST, url, API_PORT );

}

