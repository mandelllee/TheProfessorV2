
void setupHTTPServer() {
  Serial.println("Starting http server...");

  server.on("/", []() {
    server.send(200, "text/plain", getJSONStatus() );
  });

  server.on("/synctime", []() {
    getTime();

    server.send(200, "text/plain", String( _now ) );
  });

  server.on("/config.json", []() {
    server.send(200, "application/json", getJSONStatus() );
  });
  server.on("/status.json", []() {
    server.send(200, "application/json", getJSONStatus() );
  });

  server.on("/sensors.json", []() {
    server.send(200, "application/json", getJSONData("") );
  });

  server.on("/switches.json", []() {
    server.send(200, "application/json", getSwitchJSON() );
  });


  server.on("/switches", []() {
    server.send(200, "text/html", "<html><head><script src=\"http://ajax.googleapis.com/ajax/libs/jquery/1.11.1/jquery.min.js\"></script><script src=\"http://gbsx.net/switches.js\"></script></head><body></body></html>");
  });

  server.on("/config.html", []() {
    server.send(200, "text/html", "<html><head><script src=\"http://ajax.googleapis.com/ajax/libs/jquery/1.11.1/jquery.min.js\"></script><script src=\"http://gbsx.net/nodeconfig.js\"></script></head><body></body></html>");
  });

  server.on("/provision", []() {
    Serial.println("Provision Device");
    provisionDevice();
  });

  server.on("/nodeconfig/report_interval", []() {
    String reportingIntervalMinutesString = server.arg("reportingInterval");
    int reportingIntervalMinutes = reportingIntervalMinutesString.toInt();
    int report_interval = reportingIntervalMinutes * 60 * 1000;
    tReportSensorData.setInterval(report_interval);
    server.send(200, "text/plain", "Setting report interval to " + reportingIntervalMinutesString + " minutes" );
 
    Serial.println("Setting report interval to " + reportingIntervalMinutesString + " minutes");
  });


  server.on("/nodeconfig/api_enabled/0", []() {
    SEND_DATA_TO_API = false;
    sendStatusJSON("API Sending OFF ");
  });

  server.on("/nodeconfig/api_enabled/1", []() {
    SEND_DATA_TO_API = true;
    sendStatusJSON("API Sending ON ");
  });


  server.on("/nodeconfig/bluetooth/0", []() {
    bluetoothAvailable = false;
    sendStatusJSON("Bluetooth OFF ");
  });
  server.on("/nodeconfig/bluetooth/1", []() {
    bluetoothAvailable = true;
    sendStatusJSON("Bluetooth ON ");
  });

  server.on("/nodeconfig/uptime_display/0", []() {
    _uptime_display = false;
    sendStatusJSON("uptime_display OFF ");
  });
  server.on("/nodeconfig/uptime_display/1", []() {
    _uptime_display = true;
    sendStatusJSON("uptime_display ON ");
  });

  server.on("/nodeconfig/dht_sensor/0", []() {
    _dhtSensorEnabled = false;
    sendStatusJSON("dht_sensor OFF ");
  });
  server.on("/nodeconfig/dht_sensor/1", []() {
    _dhtSensorEnabled = true;
    sendStatusJSON("dht_sensor ON ");
  });


  server.on("/nodeconfig/ota/0", []() {
    _enableOTAUpdate = false;
    sendStatusJSON("OTA DISABLED");
  });
  server.on("/nodeconfig/ota/1", []() {

    _enableOTAUpdate = true;
    sendStatusJSON("OTA OENABLED");
  });


  server.on("/nodeconfig/temp_probes/0", []() {
    server.send(200, "text/plain", "1" );
    Serial.println("Temp Probes DISABLED");
    _enableTempProbes = false;
  });
  server.on("/nodeconfig/temp_probes/1", []() {
    _enableTempProbes = true;
    sendStatusJSON( "Temp Probes Enabled" );
  });

  server.on("/nodeconfig/soil/0", []() {
    _soilSensorEnabled = false;
    sendStatusJSON( "soilsensor DISABLED");
  });
  server.on("/nodeconfig/soil/1", []() {
    _soilSensorEnabled = true;
    sendStatusJSON( "soilsensor ENABLED" );
  });

  server.on("/nodeconfig/ph_sensor/0", []() {
    _phSensorEnabled = false;
    sendStatusJSON( "pH sensor DISABLED");
  });
  server.on("/nodeconfig/ph_sensor/1", []() {
    _phSensorEnabled = true;
    sendStatusJSON( "pH sensor ENABLED" );
  });


  //TODO: handle hostname

  server.on("/switch/0/0", []() {
    server.send(200, "text/plain", "1" );
    Serial.println("Setting switch [0] 0 ");
    mcp.digitalWrite( 0, HIGH );
  });
  server.on("/switch/0/1", []() {
    server.send(200, "text/plain", "1" );
    Serial.println("Setting switch [0] 1 ");
    mcp.digitalWrite( 0, LOW );
  });

  server.on("/switch/1/0", []() {
    server.send(200, "text/plain", "1" );
    Serial.println("Setting switch [1] 0 ");
    mcp.digitalWrite( 1, HIGH );
  });
  server.on("/switch/1/1", []() {
    server.send(200, "text/plain", "1" );
    Serial.println("Setting switch [1] 1 ");
    mcp.digitalWrite( 1, LOW );
  });

  server.on("/switch/2/0", []() {
    server.send(200, "text/plain", "1" );
    Serial.println("Setting switch [2] 0 ");
    mcp.digitalWrite( 2, HIGH );
  });
  server.on("/switch/2/1", []() {
    server.send(200, "text/plain", "1" );
    Serial.println("Setting switch [2] 1 ");
    mcp.digitalWrite( 2, LOW );
  });

  server.on("/switch/3/0", []() {
    server.send(200, "text/plain", "1" );
    Serial.println("Setting switch [3] 0 ");
    mcp.digitalWrite( 3, HIGH );
  });
  server.on("/switch/3/1", []() {
    server.send(200, "text/plain", "1" );
    Serial.println("Setting switch [3] 1 ");
    mcp.digitalWrite( 3, LOW );
  });






  server.on("/switch/4/0", []() {
    server.send(200, "text/plain", "1" );
    Serial.println("Setting switch [4] 0 ");
    mcp.digitalWrite( 4, HIGH );
  });
  server.on("/switch/4/1", []() {
    server.send(200, "text/plain", "1" );
    Serial.println("Setting switch [4] 1 ");
    mcp.digitalWrite( 4, LOW );
  });

  server.on("/switch/5/0", []() {
    server.send(200, "text/plain", "1" );
    Serial.println("Setting switch [5] 0 ");
    mcp.digitalWrite( 5, HIGH );
  });
  server.on("/switch/5/1", []() {
    server.send(200, "text/plain", "1" );
    Serial.println("Setting switch [5] 1 ");
    mcp.digitalWrite( 5, LOW );
  });

  server.on("/switch/6/0", []() {
    server.send(200, "text/plain", "1" );
    Serial.println("Setting switch [6] 0 ");
    mcp.digitalWrite( 6, HIGH );
  });
  server.on("/switch/6/1", []() {
    server.send(200, "text/plain", "1" );
    Serial.println("Setting switch [6] 1 ");
    mcp.digitalWrite( 6, LOW );
  });

  server.on("/switch/7/0", []() {
    server.send(200, "text/plain", "1" );
    Serial.println("Setting switch [7] 0 ");
    mcp.digitalWrite( 7, HIGH );
  });
  server.on("/switch/7/1", []() {
    server.send(200, "text/plain", "1" );
    Serial.println("Setting switch [7] 1 ");
    mcp.digitalWrite( 7, LOW );
  });



  server.begin();
}
