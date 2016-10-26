


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
    int dry[] = {246,223,281,0};
    int wet[] = {265,323,321,0};

    _soilSensorEnabled = true;
    soilSensorLabel[0] = "mint";
    soilSensorLabel[1] = "_";
    soilSensorLabel[2] = "_";

    _soilConfigJSON = ",\n      \"config\": { \n";
    _soilConfigJSON+= "        \"io\": { \n";
    _soilConfigJSON+= "          \"controller\":\""+String(useIOForSoilSensor==true?"mcp":"gpio")+"\",\n";
    _soilConfigJSON+= "          \"pins\": { \n";
    _soilConfigJSON+= "            \"1\":\""+String(soilSensorPin1)+"\",\n";
    _soilConfigJSON+= "            \"2\":\""+String(soilSensorPin2)+"\",\n";
    _soilConfigJSON+= "            \"3\":\""+String(soilSensorPin3)+"\",\n";
    _soilConfigJSON+= "            \"4\":\""+String(soilSensorPin4)+"\"\n";
    _soilConfigJSON+= "           }\n";
    _soilConfigJSON+= "         }\n";
    _soilConfigJSON+= "       },\n";
    _soilConfigJSON+= "      \"calibration\": { \n";
    _soilConfigJSON+= "        \"label\": { \n";
    _soilConfigJSON+= "          \"1\":\""+soilSensorLabel[0]+"\",\n";
    _soilConfigJSON+= "          \"2\":\""+soilSensorLabel[1]+"\",\n";
    _soilConfigJSON+= "          \"3\":\""+soilSensorLabel[2]+"\",\n";
    _soilConfigJSON+= "          \"4\":\""+soilSensorLabel[3]+"\"\n";
    _soilConfigJSON+= "         },\n";
    _soilConfigJSON+= "        \"dry\": { \n";
    _soilConfigJSON+= "          \"1\":\""+String(dry[0])+"\",\n";
    _soilConfigJSON+= "          \"2\":\""+String(dry[1])+"\",\n";
    _soilConfigJSON+= "          \"3\":\""+String(dry[2])+"\",\n";
    _soilConfigJSON+= "          \"4\":\""+String(dry[3])+"\"\n";
    _soilConfigJSON+= "         },\n";
    _soilConfigJSON+= "        \"wet\": { \n";
    _soilConfigJSON+= "          \"1\":\""+String(wet[0])+"\",\n";
    _soilConfigJSON+= "          \"2\":\""+String(wet[1])+"\",\n";
    _soilConfigJSON+= "          \"3\":\""+String(wet[2])+"\",\n";
    _soilConfigJSON+= "          \"4\":\""+String(wet[3])+"\"\n";
    _soilConfigJSON+= "        }\n";
    _soilConfigJSON+= "      }\n";


  } else if ( chip_id == "1770948" ) {
    _hostname = "piru";
    _soilSensorEnabled = false;
    _phSensorEnabled = true;
    _enableTempProbes = true;
    _dhtSensorEnabled = true;
    _luxSensorEnabled = true;
    _ECSensorEnabled = true;
    _BMP085Enabled = true;

  } else if ( chip_id == "13382423" ) {

    _hostname = "EastVillage";
    _luxSensorEnabled = true;
    _dhtSensorEnabled = true;
  
  } else if ( chip_id == "12601523" ) {
    _hostname = "pirupower";


  } else if ( chip_id == "1555028" ) {
    _hostname = "ford";
    
    _phSensorEnabled = false;
    _enableTempProbes = true;
    _flowCounterEnabled = false;

    _dhtSensorEnabled = true;
    soilSensorPin1 = 0;
    soilSensorPin2 = -1;
    soilSensorPin3 = -1;
    _soilSensorEnabled = true;
    soilSensorLabel[0] = "yoyo1";

    int dry[] = {1343,0,0,0};
    int wet[] = {15340,0,0,0};

    _soilConfigJSON = ",\n      \"config\": { \n";
    _soilConfigJSON+= "        \"io\": { \n";
    _soilConfigJSON+= "          \"controller\":\""+String(useIOForSoilSensor==true?"mcp":"gpio")+"\",\n";
    _soilConfigJSON+= "          \"pins\": { \n";
    _soilConfigJSON+= "            \"1\":\""+String(soilSensorPin1)+"\",\n";
    _soilConfigJSON+= "            \"2\":\""+String(soilSensorPin2)+"\",\n";
    _soilConfigJSON+= "            \"3\":\""+String(soilSensorPin3)+"\",\n";
    _soilConfigJSON+= "            \"4\":\""+String(soilSensorPin4)+"\"\n";
    _soilConfigJSON+= "           }\n";
    _soilConfigJSON+= "         }\n";
    _soilConfigJSON+= "       },\n";
    _soilConfigJSON+= "      \"calibration\": { \n";
    _soilConfigJSON+= "        \"label\": { \n";
    _soilConfigJSON+= "          \"1\":\""+soilSensorLabel[0]+"\",\n";
    _soilConfigJSON+= "          \"2\":\""+soilSensorLabel[1]+"\",\n";
    _soilConfigJSON+= "          \"3\":\""+soilSensorLabel[2]+"\",\n";
    _soilConfigJSON+= "          \"4\":\""+soilSensorLabel[3]+"\"\n";
    _soilConfigJSON+= "         },\n";
    _soilConfigJSON+= "        \"dry\": { \n";
    _soilConfigJSON+= "          \"1\":\""+String(dry[0])+"\",\n";
    _soilConfigJSON+= "          \"2\":\""+String(dry[1])+"\",\n";
    _soilConfigJSON+= "          \"3\":\""+String(dry[2])+"\",\n";
    _soilConfigJSON+= "          \"4\":\""+String(dry[3])+"\"\n";
    _soilConfigJSON+= "         },\n";
    _soilConfigJSON+= "        \"wet\": { \n";
    _soilConfigJSON+= "          \"1\":\""+String(wet[0])+"\",\n";
    _soilConfigJSON+= "          \"2\":\""+String(wet[1])+"\",\n";
    _soilConfigJSON+= "          \"3\":\""+String(wet[2])+"\",\n";
    _soilConfigJSON+= "          \"4\":\""+String(wet[3])+"\"\n";
    _soilConfigJSON+= "        }\n";
    _soilConfigJSON+= "      }\n";
    
  } else if ( chip_id == "13891932" ) {
    _hostname = "aqua";
    _soilSensorEnabled = false;
    _phSensorEnabled = true;
    _enableTempProbes = true;
    _flowCounterEnabled = true;

  } else if ( chip_id == "13890934" ) {
    _hostname = "potato";
    _soilSensorEnabled = true;
    useIOForSoilSensor = true;
    soilSensorPin1 = 12;
    
    _phSensorEnabled = false;
    _enableTempProbes = true;
    _dhtSensorEnabled = false;

    _useIOForSwitchChannels = true;
    ch1_label = "Light [0]";
    ch1_pin = 0;

    ch2_label = "Pump [1]";
    ch2_pin = 1;

    ch3_label = "Drain [2]";
    ch3_pin = 2;

    ch4_label = "Doser Pump [3]";
    ch4_pin = 3;


  } else if ( chip_id == "14558901" ) {
    _hostname = "pepper";
    
    useIOForSoilSensor = true;
    _soilSensorEnabled = true;
    soilSensorLabel[0] = "Fuchia";
    soilSensorLabel[1] = "Dome_Succulent";
    soilSensorLabel[2] = "Bunny_Succulent";
    soilSensorLabel[3] = "Other";
    
    _soilConfigJSON = ",\n      \"calibration\": { \n";
    _soilConfigJSON+= "        \"label\": { \n";
    _soilConfigJSON+= "          \"1\":\""+soilSensorLabel[0]+"\",\n";
    _soilConfigJSON+= "          \"2\":\""+soilSensorLabel[1]+"\",\n";
    _soilConfigJSON+= "          \"3\":\""+soilSensorLabel[2]+"\",\n";
    _soilConfigJSON+= "          \"4\":\""+soilSensorLabel[3]+"\"\n";
    _soilConfigJSON+= "         },\n";
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
    soilSensorPin4 = -1;
    soilReportFrequencySeconds = 10 * 60;

    
    _phSensorEnabled = false;
    _enableTempProbes = false;
    _dhtSensorEnabled = true;



    //  } else if ( chip_id == "1658862" ) {
    //    _hostname = "bean";
    //    _soilSensorEnabled = 88;
    //    _phSensorEnabled = false;
    //    _enableTempProbes = false;
    //    _dhtSensorEnabled = true;

  
  } else if ( chip_id == "13916356" ) {
    _hostname = "tempo";
    _soilSensorEnabled = false;
    
    _dhtSensorEnabled = true;
    _luxSensorEnabled = true;
    _BMP085Enabled = true;
  
  } else if ( chip_id == "16044873" ) {
    _hostname = "taco";
    
    useIOForSoilSensor = true;
//    soilSensorPin1 = 12;
//    soilSensorPin2 = 13;
    soilSensorPin1 = 0;
    soilSensorPin2 = 1;

    _soilSensorEnabled = true;
    soilSensorLabel[0] = "soil_sensor1";
    soilSensorLabel[1] = "soil_sensor2";

    _phSensorEnabled = false;
    _enableTempProbes = false;
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
