

void firmwareProgress(unsigned int progress, unsigned int total) {
  Serial.printf("Progress: %u%%\r", (progress / (total / 100)));

  //display.clearDisplay();
  displayTextOnDisplay( "\nUpdating Firmware...\n\n  " +  String( progress / (total / 100) ) + "%" );


};

void setupOTAUpdate() {

  // Port defaults to 8266
  ArduinoOTA.setPort(8266);

  char hn[_hostname.length() + 1];
  memset(hn, 0, _hostname.length() + 1);

  for (int i = 0; i < _hostname.length(); i++)
    hn[i] = _hostname.charAt(i);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname( hn );


  // No authentication by default
  //ArduinoOTA.setPassword( (const char *)"123" );

  ArduinoOTA.onStart([]() {

    recordValue( "firmware", "install-ota", "start", "?");
    _renderDisplayEnabled = false;
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
    recordValue( "firmware", "install-ota", "complete", "?");


    recordValue( "system", "status", "reboot", "?");

    Serial.println("\Rebooting");
    displayTextOnDisplay( "\n\nRebooting..." );

    ESP.restart();
  });
  ArduinoOTA.onProgress(firmwareProgress);
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}

