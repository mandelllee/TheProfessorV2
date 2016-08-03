

bool _useIOForSwitchChannels = true;

String ch1_label = "";
String ch2_label = "";
String ch3_label = "";
String ch4_label = "";
String ch5_label = "";
String ch6_label = "";
String ch7_label = "";
String ch8_label = "";

int ch1_pin = -1;
int ch2_pin = -1;
int ch3_pin = -1;
int ch4_pin = -1;
int ch5_pin = -1;
int ch6_pin = -1;
int ch7_pin = -1;
int ch8_pin = -1;

String getSwitchJSON() {

String _json =  "{\n";

  _json += "  \"hostname\":\"" + _hostname + "\"";
  _json += ",\n  \"core_version\":\"" + VERSION + "\"";
//  if ( msg.length() > 0) _json += ",\n  \"msg\": \"" + msg + "\"";
  _json += ",\n";

  _json += "  \"now\":\"" + String(_now) + "\",\n";
  _json += "  \"switches\": [\n";
  
  _json += "    {\n";
  _json += "      \"controller\":\"" + String(_useIOForSwitchChannels== true ? "mcp23017" : "gpio") + "\",\n";
  _json += "      \"channels\": { \n";
  _json += "        \"1\": { \"pin\": " + String(ch1_pin) + ", \"label\":\""+ch1_label+"\" },\n";
  _json += "        \"2\": { \"pin\": " + String(ch2_pin) + ", \"label\":\""+ch2_label+"\" },\n";
  _json += "        \"3\": { \"pin\": " + String(ch3_pin) + ", \"label\":\""+ch3_label+"\" },\n";
  _json += "        \"4\": { \"pin\": " + String(ch4_pin) + ", \"label\":\""+ch4_label+"\" },\n";
  _json += "        \"5\": { \"pin\": " + String(ch5_pin) + ", \"label\":\""+ch5_label+"\" },\n";
  _json += "        \"6\": { \"pin\": " + String(ch6_pin) + ", \"label\":\""+ch6_label+"\" },\n";
  _json += "        \"7\": { \"pin\": " + String(ch7_pin) + ", \"label\":\""+ch7_label+"\" },\n";
  _json += "        \"8\": { \"pin\": " + String(ch8_pin) + ", \"label\":\""+ch8_label+"\" }\n";
  _json += "      }\n";
  _json += "    }\n";
  
  _json += "  ]\n";
  _json += "}\n";

  return _json;
}
