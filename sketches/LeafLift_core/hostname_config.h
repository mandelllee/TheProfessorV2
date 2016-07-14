


void configureHostname() {

  chip_id = String( ESP.getChipId() );
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

  } else if ( chip_id == "16044072" ) {
    _hostname = "hippo";
    _soilSensorEnabled = true;
    _phSensorEnabled = false;
    _enableTempProbes = false;
    _dhtSensorEnabled = true;

  } else if ( chip_id == "13891932" ) {
    _hostname = "aqua";
    _soilSensorEnabled = false;
    _phSensorEnabled = true;
    _enableTempProbes = true;

  } else if ( chip_id == "13890934" ) {
    _hostname = "potato";
    _soilSensorEnabled = false;
    _phSensorEnabled = false;
    _enableTempProbes = false;
    _dhtSensorEnabled = true;

  } else if ( chip_id == "14558901" ) {
    _hostname = "pepper";
    useIOForSoilSensor = true;
    _soilSensorEnabled = true;
    _soilConfigJSON = ",\n      \"calibration\": { \n";
    _soilConfigJSON+= "        \"dry\": { \n";
    _soilConfigJSON+= "          \"1\":\"909\",\n";
    _soilConfigJSON+= "          \"2\":\"911\",\n";
    _soilConfigJSON+= "          \"3\":\"868\",\n";
    _soilConfigJSON+= "          \"4\":\"903\"\n";
    _soilConfigJSON+= "         },\n";
    _soilConfigJSON+= "        \"wet\": { \n";
    _soilConfigJSON+= "          \"1\":\"294\",\n";
    _soilConfigJSON+= "          \"2\":\"283\",\n";
    _soilConfigJSON+= "          \"3\":\"239\",\n";
    _soilConfigJSON+= "          \"4\":\"21\"\n";
    _soilConfigJSON+= "        }\n";
    _soilConfigJSON+= "      }\n";
    soilSensorPin1 = 3;
    soilSensorPin2 = 2;
    soilSensorPin3 = 0;
    soilSensorPin4 = 1;
    _phSensorEnabled = false;
    _enableTempProbes = false;
    _dhtSensorEnabled = false;



    //  } else if ( chip_id == "1658862" ) {
    //    _hostname = "bean";
    //    _soilSensorEnabled = 88;
    //    _phSensorEnabled = false;
    //    _enableTempProbes = false;
    //    _dhtSensorEnabled = true;

  } else if ( chip_id == "1770948" ) {
    _hostname = "piru";
    _soilSensorEnabled = true;
    _phSensorEnabled = true;
    _enableTempProbes = true;
    _dhtSensorEnabled = true;

  } else if ( chip_id == "13916356" ) {
    _hostname = "tempo";
    _soilSensorEnabled = false;
    _dhtSensorEnabled = true;

  } else if ( chip_id == "16044873" ) {
    _hostname = "taco";
    _soilSensorEnabled = true;
    _phSensorEnabled = false;
    _enableTempProbes = true;
    _dhtSensorEnabled = false;

  } else if ( chip_id == "1626288" ) {
    _hostname = "dino";
    _soilSensorEnabled = true;
    _phSensorEnabled = false;
    _enableTempProbes = false;
    _dhtSensorEnabled = false;
    // hack  to have bluetooth enabled
    //bluetoothAvailable = true;

  } else {
    _hostname = "ESP_" + chip_id;
  }

}
