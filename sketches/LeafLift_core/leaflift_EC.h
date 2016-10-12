#define ec_sensor_address  100 //0x64

byte code = 0;                   //used to hold the I2C response code.
char ec_data[48];                //we make a 48 byte character array to hold incoming data from the EC circuit.
byte in_char = 0;                //used as a 1 byte buffer to store in bound bytes from the EC Circuit.
byte i = 0;                      //counter used for ec_data array.

char *ec;                        //char pointer used in string parsing.
char *tds;                       //char pointer used in string parsing.
char *sal;                       //char pointer used in string parsing.
char *sg;                        //char pointer used in string parsing.

float ec_float;                  //float var used to hold the float value of the conductivity.
float tds_float;                 //float var used to hold the float value of the TDS.
float sal_float;                 //float var used to hold the float value of the salinity.
float sg_float;                  //float var used to hold the float value of the specific gravity.

void string_pars() {                  //this function will break up the CSV string into its 4 individual parts. EC|TDS|SAL|SG.
  //this is done using the C command “strtok”.

  ec = strtok(ec_data, ",");          //let's pars the string at each comma.
  tds = strtok(NULL, ",");            //let's pars the string at each comma.
  sal = strtok(NULL, ",");            //let's pars the string at each comma.
  sg = strtok(NULL, ",");             //let's pars the string at each comma.

  Serial.print("EC:");                //we now print each value we parsed separately.

  Serial.println(ec);                 //this is the EC value.

  Serial.print("TDS:");               //we now print each value we parsed separately.
  Serial.println(tds);                //this is the TDS value.

  Serial.print("SAL:");               //we now print each value we parsed separately.
  Serial.println(sal);                //this is the salinity value.

  Serial.print("SG:");               //we now print each value we parsed separately.
  Serial.println(sg);                //this is the specific gravity.

    ec_float=atof(ec);
    tds_float=atof(tds);
    sal_float=atof(sal);
    sg_float=atof(sg);
}

void readECSensor () {
  Serial.println("Reading EC Sensor");
     Wire.beginTransmission(ec_sensor_address);            //call the circuit by its ID number.
    Wire.write("R");                   //transmit the command that was sent through the serial port.
    Wire.endTransmission();                     //end the I2C data transmission.
    delay (1000);

         Wire.requestFrom(ec_sensor_address, 48, 1);          //call the circuit and request 48 bytes (this is more than we need)
      code = Wire.read();                        //the first byte is the response code, we read this separately.

      while (Wire.available()) {                  //are there bytes to receive.
        in_char = Wire.read();                    //receive a byte.
        ec_data[i] = in_char;                     //load this byte into our array.
        i += 1;                                   //incur the counter for the array element.
        if (in_char == 0) {                       //if we see that we have been sent a null command.
          i = 0;                                  //reset the counter i to 0.
          Wire.endTransmission();                 //end the I2C data transmission.
          break;                                  //exit the while loop.
        }
      }

          switch (code) {                           //switch case based on what the response code is.
        case 1:                                 //decimal 1.
          Serial.println("Success");            //means the command was successful.
          break;                                //exits the switch case.

        case 2:                                 //decimal 2.
          Serial.println("Failed");             //means the command has failed.
          break;                                //exits the switch case.

        case 254:                               //decimal 254.
          Serial.println("Pending");            //means the command has not yet been finished calculating.
          break;                                //exits the switch case.

        case 255:                               //decimal 255.
          Serial.println("No Data");            //means there is no further data to send.
          break;                                //exits the switch case.
      }

      Serial.println(ec_data);                  //print the data.
      string_pars();


}



