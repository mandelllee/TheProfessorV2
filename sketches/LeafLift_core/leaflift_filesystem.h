#include <ArduinoJson.h>

void writeWiFiConfigToFilesystem() {

  String path = "/wifi.json";
  Serial.println("attempting to open WiFi config file...");
  File f = SPIFFS.open(path, "w");
  f.println( );
  f.close();
}

void writeConfigToFilesystem() {

//  String path = "/config.json";
//  Serial.println("writing config file...");
//  File f = SPIFFS.open(path, "w");
//  f.println( getJSONStatus() );
//  f.close();

}

bool parseConfigData( String configJSON ) {
  Serial.println( "Parsing config:\n" );
  Serial.println( configJSON );
  Serial.println( "- - - - - - - \n\n\n" );
  StaticJsonBuffer<800> jsonBuffer;

  JsonObject& root = jsonBuffer.parseObject(configJSON);

  if (!root.success()) {
    Serial.println("parseObject() failed");
    return false;
  }

  const char* hostn = root["hostname"];
  Serial.println( "HOSTNAMW: [" + String(hostn) + "]");
  return true;
}
void formatFilesystem() {

  Serial.println("Formatting filesystem...");
  if ( SPIFFS.format() ) {
    Serial.println("Formatting successful.");
    displayTextOnDisplay("formatting successful...");
    delay( 5 * 1000 );
    //        f = SPIFFS.open(path, "w");
    //        f.println( getJSONStatus() );
    //        f.close();
  }

}

bool setupFilesystem() {


  bool fs_status = SPIFFS.begin();

  if ( fs_status ) {
    String path = "/config.json";
    Serial.println("attempting to open config file...");
    File f = SPIFFS.open(path, "r");

    //if (!f) {
    if ( true ) {
      Serial.println("Opening file failed.");

//      displayTextOnDisplay("no config exists...");
//      delay( 10 * 1000 );
      return false;

    } else {

      Serial.println("Config file loaded.\n");
      char buf[] = "";
      String configJSON = "";

      while (f.available()) {
        //Lets read line by line from the file
        String line = f.readStringUntil('\n');
        configJSON += line;
        Serial.println(line);
      }
      return parseConfigData( configJSON );

      Serial.println("\n\n\n\n");
    }
  } else {
    Serial.println("Fatal Filesystem failure...");
  }

}
