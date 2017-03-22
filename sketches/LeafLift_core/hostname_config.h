


void configureHostname() {

  chip_id = String( ESP.getChipId() );
  Serial.println("Chip ID: " + chip_id);
  //char c[] = chip_id.c_str();

  char c[chip_id.length() + 1];
  memset(c, 0, chip_id.length() + 1);

  for (int i = 0; i < chip_id.length(); i++)
    c[i] = chip_id.charAt(i);

  //generate the MD5 hash for our string
  unsigned char* hash = MD5::make_hash( c );

  //generate the digest (hex encoding) of our hash
  char *md5str = MD5::make_digest(hash, 16);
  free(hash);
  //print it on our serial monitor
  //Serial.println(md5str);
  //Give the Memory back to the System if you run the md5 Hash generation in a loop
  BOARD_ID = md5str;

  free(md5str);

  //hostname = ESP.getChipId()
  //char hostname [12+1];
  //_hostname = "ESP_" + chip_id;

  if ( chip_id == "13904180" ) {
    _hostname = "soil";
    _soilSensorEnabled = true;


  } else if ( chip_id == "1265664" ) {

    _hostname = "carrot";    
    useIOForSoilSensor = true;
    
  } else if ( chip_id == "1189564" ) {

    wifi_ssid = "Bork 2.4";
    wifi_psk = "OrlandoNakazawa!";

    _hostname = "pea";
    useIOForSoilSensor = true;
    
    channel_pins[1]= 0; //LED
    //button_pins[1] = 4; //BUTTON
    
    channel_pins[2]= 1; //LED
    //button_pins[2] = 5; //BUTTON
    //button_pins[3] = 6; //BUTTON
    //button_pins[4] = 7; //BUTTON
    channel_pins[3]= 2; //LED
    channel_pins[4]= 3; //LED
    
  } else if ( chip_id == "12628048" ) {
    _hostname = "proto1";

    channel_pins[1] = 0;
    channel_pins[2] = 1;
    channel_pins[3] = 2;
    channel_pins[4] = 3;
    channel_pins[5] = 4;
    channel_pins[6] = 5;
    channel_pins[7] = 6;
    channel_pins[8] = 7;

    _buttonBoardConnected = true;
    dry_calibration[0] = 13101;
    wet_calibration[0] = 22318;

    _soilSensorEnabled = false;
    soilSensorLabel[0] = "sensor1";
    useIOForSoilSensor = true;
    soilSensorPin1 = 7;
    _soilConfigJSON = ",\n      \"config\": { \n";
    _soilConfigJSON += "        \"io\": { \n";
    _soilConfigJSON += "          \"controller\":\"" + String(useIOForSoilSensor == true ? "mcp" : "gpio") + "\",\n";
    _soilConfigJSON += "          \"pins\": { \n";
    _soilConfigJSON += "            \"1\":\"" + String(soilSensorPin1) + "\",\n";
    _soilConfigJSON += "            \"2\":\"" + String(soilSensorPin2) + "\",\n";
    _soilConfigJSON += "            \"3\":\"" + String(soilSensorPin3) + "\",\n";
    _soilConfigJSON += "            \"4\":\"" + String(soilSensorPin4) + "\"\n";
    _soilConfigJSON += "           }\n";
    _soilConfigJSON += "         }\n";
    _soilConfigJSON += "       },\n";
    _soilConfigJSON += "      \"calibration\": { \n";
    _soilConfigJSON += "        \"label\": { \n";
    _soilConfigJSON += "          \"1\":\"" + soilSensorLabel[0] + "\",\n";
    _soilConfigJSON += "          \"2\":\"" + soilSensorLabel[1] + "\",\n";
    _soilConfigJSON += "          \"3\":\"" + soilSensorLabel[2] + "\",\n";
    _soilConfigJSON += "          \"4\":\"" + soilSensorLabel[3] + "\"\n";
    _soilConfigJSON += "         },\n";
    _soilConfigJSON += "        \"dry\": { \n";
    _soilConfigJSON += "          \"1\":\"" + String(dry_calibration[0]) + "\",\n";
    _soilConfigJSON += "          \"2\":\"" + String(dry_calibration[1]) + "\",\n";
    _soilConfigJSON += "          \"3\":\"" + String(dry_calibration[2]) + "\",\n";
    _soilConfigJSON += "          \"4\":\"" + String(dry_calibration[3]) + "\"\n";
    _soilConfigJSON += "         },\n";
    _soilConfigJSON += "        \"wet\": { \n";
    _soilConfigJSON += "          \"1\":\"" + String(wet_calibration[0]) + "\",\n";
    _soilConfigJSON += "          \"2\":\"" + String(wet_calibration[1]) + "\",\n";
    _soilConfigJSON += "          \"3\":\"" + String(wet_calibration[2]) + "\",\n";
    _soilConfigJSON += "          \"4\":\"" + String(wet_calibration[3]) + "\"\n";
    _soilConfigJSON += "        }\n";
    _soilConfigJSON += "      }\n";

  } else if ( chip_id == "16044072" ) {
    _hostname = "hippo";

    //API_HOST = "10.5.1.147";
    //API_PORT = 3000;

    _phSensorEnabled = false;
    _enableTempProbes = true;
    _dhtSensorEnabled = false;

    useIOForSoilSensor = true;
    soilSensorPin1 = 0;
    soilSensorPin2 = -1;
    soilSensorPin3 = -1;

    // DRY    "1":"310",
    //         "2":"416",
    //         "3":"0",
    //         "4":"0"
    // WET        "1":"240",
    //         "2":"402",
    //         "3":"0",
    //         "4":"0"
    int dry[] = {246, 223, 281, 0};
    int wet[] = {265, 323, 321, 0};

    _soilSensorEnabled = true;
    soilSensorLabel[0] = "mint";
    soilSensorLabel[1] = "_";
    soilSensorLabel[2] = "_";

    _soilConfigJSON = ",\n      \"config\": { \n";
    _soilConfigJSON += "        \"io\": { \n";
    _soilConfigJSON += "          \"controller\":\"" + String(useIOForSoilSensor == true ? "mcp" : "gpio") + "\",\n";
    _soilConfigJSON += "          \"pins\": { \n";
    _soilConfigJSON += "            \"1\":\"" + String(soilSensorPin1) + "\",\n";
    _soilConfigJSON += "            \"2\":\"" + String(soilSensorPin2) + "\",\n";
    _soilConfigJSON += "            \"3\":\"" + String(soilSensorPin3) + "\",\n";
    _soilConfigJSON += "            \"4\":\"" + String(soilSensorPin4) + "\"\n";
    _soilConfigJSON += "           }\n";
    _soilConfigJSON += "         }\n";
    _soilConfigJSON += "       },\n";
    _soilConfigJSON += "      \"calibration\": { \n";
    _soilConfigJSON += "        \"label\": { \n";
    _soilConfigJSON += "          \"1\":\"" + soilSensorLabel[0] + "\",\n";
    _soilConfigJSON += "          \"2\":\"" + soilSensorLabel[1] + "\",\n";
    _soilConfigJSON += "          \"3\":\"" + soilSensorLabel[2] + "\",\n";
    _soilConfigJSON += "          \"4\":\"" + soilSensorLabel[3] + "\"\n";
    _soilConfigJSON += "         },\n";
    _soilConfigJSON += "        \"dry\": { \n";
    _soilConfigJSON += "          \"1\":\"" + String(dry[0]) + "\",\n";
    _soilConfigJSON += "          \"2\":\"" + String(dry[1]) + "\",\n";
    _soilConfigJSON += "          \"3\":\"" + String(dry[2]) + "\",\n";
    _soilConfigJSON += "          \"4\":\"" + String(dry[3]) + "\"\n";
    _soilConfigJSON += "         },\n";
    _soilConfigJSON += "        \"wet\": { \n";
    _soilConfigJSON += "          \"1\":\"" + String(wet[0]) + "\",\n";
    _soilConfigJSON += "          \"2\":\"" + String(wet[1]) + "\",\n";
    _soilConfigJSON += "          \"3\":\"" + String(wet[2]) + "\",\n";
    _soilConfigJSON += "          \"4\":\"" + String(wet[3]) + "\"\n";
    _soilConfigJSON += "        }\n";
    _soilConfigJSON += "      }\n";


//  } else if ( chip_id == "1770948" ) {
//    _hostname = "piru";
//    _soilSensorEnabled = true;
//    _phSensorEnabled = true;
//    _enableTempProbes = true;
//    _dhtSensorEnabled = true;

  } else if ( chip_id == "1607565" ) {
    _hostname = "piruWestGR1";
     _dhtSensorEnabled = true;
  _luxSensorEnabled = true;
      wifi_ssid = "Stone-AC";
    wifi_psk = "8056891059";
  API_HOST = "api-quadroponic.rhcloud.com";
  API_PORT = 80;

 } else if ( chip_id == "312335" ) {
    _hostname = "piruWestGR2";
     _dhtSensorEnabled = true;
  _luxSensorEnabled = true;
      wifi_ssid = "Stone-AC";
    wifi_psk = "8056891059";
  API_HOST = "api-quadroponic.rhcloud.com";
  API_PORT = 80;

} else if ( chip_id == "291630" ) {
    _hostname = "piruNorthGR1";
     _dhtSensorEnabled = true;
  _luxSensorEnabled = true;
      wifi_ssid = "flamingo";
    wifi_psk = "leemandell";
  API_HOST = "api-quadroponic.rhcloud.com";
  API_PORT = 80;

  

  } else if ( chip_id == "13382423" ) {

    _hostname = "fillmore";

  } else if ( chip_id == "1770948" ) {
    _hostname = "pirupower";
   _useIOForSwitchChannels = true;
    channel_pins[1] = 0;
    channel_pins[2] = 1 ;
    channel_pins[3] = 2;
    channel_pins[4] = 3;

    _dhtSensorEnabled = true;
    wifi_ssid = "flamingo";
    wifi_psk = "leemandell";


  } else if ( chip_id == "1658862" ) {
    _hostname = "insulin";
    _useIOForSwitchChannels = false;

    button_pins[1] = 12;
    //button_led_pins[1] = 15;
    channel_pins[3] = 15;

    //button_pins[2] = 14;
    //channel_pins[2] = 13;

  } else if ( chip_id == "1555028" ) {

    _hostname = "ford";

    _phSensorEnabled = false;
    _enableTempProbes = false;
    _flowCounterEnabled = false;


    _useIOForSwitchChannels = true;
    button_pins[1] = 2;
    channel_pins[1] = 3;//LED
    
    button_pins[2] = 4;
    channel_pins[2] = 5;//LED
    
    button_pins[3] = 6;
    channel_pins[3] = 7;//LED
    
    channel_pins[4] = 8;//LED
    button_pins[4] = 9;
    
    channel_pins[5] = 10;//LED
    button_pins[5] = 11;
    
    channel_pins[6] = 15;//LED
    button_pins[6] = 0;


    _dhtSensorEnabled = false;
    soilSensorPin1 = 0;
    soilSensorPin2 = -1;
    soilSensorPin3 = -1;
    _soilSensorEnabled = false;
    soilSensorLabel[0] = "yoyo1";

    int dry[] = {1343, 0, 0, 0};
    int wet[] = {15340, 0, 0, 0};

    _soilConfigJSON = ",\n      \"config\": { \n";
    _soilConfigJSON += "        \"io\": { \n";
    _soilConfigJSON += "          \"controller\":\"" + String(useIOForSoilSensor == true ? "mcp" : "gpio") + "\",\n";
    _soilConfigJSON += "          \"pins\": { \n";
    _soilConfigJSON += "            \"1\":\"" + String(soilSensorPin1) + "\",\n";
    _soilConfigJSON += "            \"2\":\"" + String(soilSensorPin2) + "\",\n";
    _soilConfigJSON += "            \"3\":\"" + String(soilSensorPin3) + "\",\n";
    _soilConfigJSON += "            \"4\":\"" + String(soilSensorPin4) + "\"\n";
    _soilConfigJSON += "           }\n";
    _soilConfigJSON += "         }\n";
    _soilConfigJSON += "       },\n";
    _soilConfigJSON += "      \"calibration\": { \n";
    _soilConfigJSON += "        \"label\": { \n";
    _soilConfigJSON += "          \"1\":\"" + soilSensorLabel[0] + "\",\n";
    _soilConfigJSON += "          \"2\":\"" + soilSensorLabel[1] + "\",\n";
    _soilConfigJSON += "          \"3\":\"" + soilSensorLabel[2] + "\",\n";
    _soilConfigJSON += "          \"4\":\"" + soilSensorLabel[3] + "\"\n";
    _soilConfigJSON += "         },\n";
    _soilConfigJSON += "        \"dry\": { \n";
    _soilConfigJSON += "          \"1\":\"" + String(dry[0]) + "\",\n";
    _soilConfigJSON += "          \"2\":\"" + String(dry[1]) + "\",\n";
    _soilConfigJSON += "          \"3\":\"" + String(dry[2]) + "\",\n";
    _soilConfigJSON += "          \"4\":\"" + String(dry[3]) + "\"\n";
    _soilConfigJSON += "         },\n";
    _soilConfigJSON += "        \"wet\": { \n";
    _soilConfigJSON += "          \"1\":\"" + String(wet[0]) + "\",\n";
    _soilConfigJSON += "          \"2\":\"" + String(wet[1]) + "\",\n";
    _soilConfigJSON += "          \"3\":\"" + String(wet[2]) + "\",\n";
    _soilConfigJSON += "          \"4\":\"" + String(wet[3]) + "\"\n";
    _soilConfigJSON += "        }\n";
    _soilConfigJSON += "      }\n";

  } else if ( chip_id == "13891932" ) {
    _hostname = "aqua";
    _soilSensorEnabled = false;
    _phSensorEnabled = true;
    _enableTempProbes = true;
    _flowCounterEnabled = false;

    _useIOForSwitchChannels = true;
    channel_pins[1] = 0;
    channel_pins[2] = 1;
    channel_pins[3] = 2;
    channel_pins[4] = 3;

    button_pins[1] = 4;
    button_pins[2] = 5;
    button_pins[3] = 6;
    button_pins[4] = 7;
    
  } else if ( chip_id == "13890934" ) {

    _hostname = "hermes";

    _dhtSensorEnabled = true;

    _phSensorEnabled = false;
    _enableTempProbes = false;
    _flowCounterEnabled = false;

    _useIOForSwitchChannels = false;
    channel_pins[1] = 13;
    channel_pins[2] = 3 ;
    channel_pins[3] = 12;
    channel_pins[4] = 14;

  } else if ( chip_id == "1223") {
    _hostname = "corn";
    _buttonBoardConnected = false;
    _useIOForSwitchChannels = true;
    ch1_label = "Ch 1";
    ch2_label = "Ch 2";
    ch3_label = "Ch 3";
    ch4_label = "Ch 4";

    channel_pins[1] = 0;
    channel_pins[2] = 1;
    channel_pins[3] = 2;
    channel_pins[4] = 3;

    ch1_pin = 0;
    ch2_pin = 1;
    ch3_pin = 2;
    ch4_pin = 3;

    ledPins[1] = 8;
    ledPins[2] = 9;
    ledPins[3] = 10;
    ledPins[4] = 11;

    _dhtSensorEnabled = true;

  } else if ( chip_id == "8870018" ) {
    _hostname = "potato";

    _buttonBoardConnected = true;

    _soilSensorEnabled = true;
    useIOForSoilSensor = true;
    soilSensorPin1 = 12;

    _phSensorEnabled = false;
    _enableTempProbes = true;
    _dhtSensorEnabled = false;

    _useIOForSwitchChannels = true;

    channel_pins[1] = 0;
    channel_pins[2] = 1;
    channel_pins[3] = 2;
    channel_pins[4] = 3;
    channel_pins[5] = 4;
    channel_pins[6] = 5;
    channel_pins[7] = 6;
    channel_pins[8] = 7;

    //    ch1_label = "Fan [0]";
    //    ch1_pin = 0;
    //
    //    ch2_label = "Pump [1]";
    //    ch2_pin = 1;
    //
    //    ch3_label = "Drain [2]";
    //    ch3_pin = 2;
    //
    //    ch4_label = "Bench Light [1]";
    //    ch4_pin = 3;
    //
    //    ch5_label = "Bench Light [2]";
    //    ch5_pin = 4;
    //
    //    ch6_label = "Ceiling Light [1]";
    //    ch6_pin = 5;
    //
    //    ch7_label = "Fan";
    //    ch7_pin = 6;
    //
    //    ch8_label = "Bench 2";
    //    ch8_pin = 7;

  } else if ( chip_id == "14558901" ) {
    _hostname = "pepper";

    useIOForSoilSensor = true;
    _soilSensorEnabled = true;
    soilSensorLabel[0] = "Fuchia";
    soilSensorLabel[1] = "Dome_Succulent";
    soilSensorLabel[2] = "Bunny_Succulent";
    soilSensorLabel[3] = "Other";

    _soilConfigJSON = ",\n      \"calibration\": { \n";
    _soilConfigJSON += "        \"label\": { \n";
    _soilConfigJSON += "          \"1\":\"" + soilSensorLabel[0] + "\",\n";
    _soilConfigJSON += "          \"2\":\"" + soilSensorLabel[1] + "\",\n";
    _soilConfigJSON += "          \"3\":\"" + soilSensorLabel[2] + "\",\n";
    _soilConfigJSON += "          \"4\":\"" + soilSensorLabel[3] + "\"\n";
    _soilConfigJSON += "         },\n";
    _soilConfigJSON += "        \"dry\": { \n";
    _soilConfigJSON += "          \"1\":\"909\",\n";
    _soilConfigJSON += "          \"2\":\"911\",\n";
    _soilConfigJSON += "          \"3\":\"868\",\n";
    _soilConfigJSON += "          \"4\":\"903\"\n";
    _soilConfigJSON += "         },\n";
    _soilConfigJSON += "        \"wet\": { \n";
    _soilConfigJSON += "          \"1\":\"294\",\n";
    _soilConfigJSON += "          \"2\":\"283\",\n";
    _soilConfigJSON += "          \"3\":\"239\",\n";
    _soilConfigJSON += "          \"4\":\"21\"\n";
    _soilConfigJSON += "        }\n";
    _soilConfigJSON += "      }\n";
    soilSensorPin1 = 3;
    soilSensorPin2 = 2;
    soilSensorPin3 = 0;
    soilSensorPin4 = -1;
    soilReportFrequencySeconds = 10 * 60;


    _phSensorEnabled = false;
    _enableTempProbes = false;
    _dhtSensorEnabled = true;

    _useIOForSwitchChannels = true;
    ch5_label = "Salt Lamp";
    ch5_pin = 4;

    ch6_label = "Plug 6";
    ch6_pin = 5;

    ch7_label = "Plug 7";
    ch7_pin = 6;

    ch8_label = "Plug 8";
    ch8_pin = 7;

    //  } else if ( chip_id == "1658862" ) {
    //    _hostname = "bean";
    //    _soilSensorEnabled = 88;
    //    _phSensorEnabled = false;
    //    _enableTempProbes = false;
    //    _dhtSensorEnabled = true;


  } else if ( chip_id == "13916356" ) {
    _hostname = "tempo";
    _soilSensorEnabled = false;
    bluetoothAvailable = true;

    _dhtSensorEnabled = true;
    _luxSensorEnabled = true;
    _BMP085Enabled = true;
    _enableTempProbes = false;

//    _useIOForSwitchChannels = false;
//
//    channel_pins[1] = 2;
//    button_pins[1] = 16;
//    channel_pins[2] = 0;
//    button_pins[2] = 15;

  } else if ( chip_id == "16044873" ) {
    _hostname = "taco";

    //useIOForSoilSensor = true;
    //    soilSensorPin1 = 12;
    //    soilSensorPin2 = 13;
    //    soilSensorPin1 = 0;
    //    soilSensorPin2 = 1;

    _soilSensorEnabled = false;
    //    soilSensorLabel[0] = "soil_sensor1";
    //    soilSensorLabel[1] = "soil_sensor2";

    _phSensorEnabled = false;
    _enableTempProbes = false;
    _dhtSensorEnabled = false;
    _useIOForSwitchChannels = false;

    channel_pins[1] = 15;
    channel_pins[2] = 15;
    channel_pins[3] = 15;
    channel_pins[4] = 15;

  } else if ( chip_id == "1626288" ) {
    _hostname = "dino";
    _soilSensorEnabled = false;
    useIOForSoilSensor = false;

    _phSensorEnabled = false;
    _enableTempProbes = false;
    _dhtSensorEnabled = false;
    // hack  to have bluetooth enabled
    //bluetoothAvailable = true;

    _useIOForSwitchChannels = true;
    //    ch1_label = "Ch 1";
    //    ch2_label = "Ch 2";
    //    ch3_label = "Ch 3";
    //    ch4_label = "Ch 4";
    //
    channel_pins[1] = 1;
    channel_pins[2] = 2;
    channel_pins[3] = 3;
    channel_pins[4] = 4;


    channel_pins[5] = 12;
    channel_pins[6] = 13;
    channel_pins[7] = 14;
    channel_pins[8] = 15;


    button_pins[1] = 13;
    button_pins[2] = 3;
    button_pins[3] = 12;
    button_pins[4] = 14;
    //    ch1_pin = 0;
    //    ch2_pin = 1;
    //    ch3_pin = 2;
    //    ch4_pin = 3;

    ledPins[1] = -1;
    ledPins[2] = -1;
    ledPins[3] = -1;
    ledPins[4] = -1;


  } else {
    _hostname = "ESP_" + chip_id;
  }

  writeConfigToFilesystem();

}
