#define powerOn 13
#define referenceLightIndicator 7

//##########GSM Variables#############//
#include <SoftwareSerial.h>
#define rxPin 2  //Gizduino pin tied to RX pin of the GSM/GPRS module.
#define txPin 3  //Gizduino pin tied to TX pin of the GSM/GPRS module.
SoftwareSerial mySerial(rxPin,txPin);
/////////////receiving variables////////
String messageRecieve;  //Message Received from API
char msgToSend[30]="";
char c = 0;
char senderNumber[14];
byte isStart = 0;
boolean isSmsReady = false;

/////////////sending variables///////////
char Rx_data[50];
unsigned char Rx_index = 0;
char msg[160];  //message to send

//#########Ultrasonic Variables#########//
#include <NewPing.h>
#define TRIGGER_PIN  5  // Gizduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     4  // Gizduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 219 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 300-350cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
int arraysize = 9;  //quantity of values to find the median (sample size). Needs to be an odd number
int distanceSamples[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0};  //declare an array to store the samples
int mode = 0;  //variable for mode identification
int index = 0;  //used for loop iterations
int smsFrequency = 1;  //it pertains to the inch difference to determine when it will send an update to the server
int minExpectedValue = 0;  //expected min of detected mode
int maxExpectedValue = 1;  //expected max of detected mode, initialized
int reportedFloodLevel = 0;
int abruptChangeTimer = 0;  //determiner of 1 minute abrupt change in water level
boolean firstModeDetermination = true;
int unitHeight = 2.19; //in meters
int floodHeightReference = 0;  //maximum height of unit in inches during installation process, flood free.


void setup() {
  //floodheight reference calculation
  floodHeightReference = abs((unitHeight*100)/2.54);
  
  ////////////receive setup ////////////
  Serial.begin(9600);
  mySerial.begin(9600);
  pinMode(powerOn,OUTPUT);  //pin 13 arduino
  pinMode(referenceLightIndicator, OUTPUT);  //pin 7 arduino

  digitalWrite(powerOn,HIGH); 
  delay(4000);
  digitalWrite(powerOn,LOW);
  delay(5000);

 Serial.println("executing AT+CMGF=1");
  mySerial.println("AT+CMGF=1"); // set sms mode to text  Human Read able Text Format.
  delay(200);
  Serial.println("executing AT+CLIP=1");
  mySerial.println("AT+CLIP=1"); // set sms mode to text
  delay(200);
 Serial.println("executing AT+CNMI=2,1,0,0");  
  mySerial.println("AT+CNMI=2,1,0,0"); // sms received noti
  delay(200);
 Serial.println("executing AT+CMGD=1,4");  
  mySerial.println("AT+CMGD=1,4"); // delete all SMS
  delay(200);
  
  ///////////sending SMS/////////
  
  initGSM();
  send_msg("21583567", "TEST 9275628107");
}

void loop() {
  readSMS();
  
  
  //########start of ultrasonic looping codes######//  
  if(index == 9){
    index = 0;
    isort(distanceSamples, arraysize);
    printArray(distanceSamples, arraysize);
    mode = getMode(distanceSamples,arraysize);

    //if it is first time for mode determination
    if(firstModeDetermination){
      firstModeDetermination = false;
      minExpectedValue = mode - smsFrequency;
      maxExpectedValue = mode + smsFrequency;
      reportedFloodLevel = mode;
      //send new update to server
      String smsUpdate = "FLUPDATE " ;
      smsUpdate.concat(reportedFloodLevel);
      char charSmsMessage[130];
      smsUpdate.toCharArray(charSmsMessage, 130);
      send_msg("21583567", charSmsMessage);
    }
    else{      
      if(mode == minExpectedValue || mode == maxExpectedValue){
        minExpectedValue = mode - smsFrequency;
        maxExpectedValue = mode + smsFrequency;
        //send new update to server
        reportedFloodLevel = mode;
        String smsUpdate = "FLUPDATE " ;
        smsUpdate.concat(reportedFloodLevel);
        char charSmsMessage[130];
        smsUpdate.toCharArray(charSmsMessage, 130);
        send_msg("21583567", charSmsMessage);
      }
    }
    
    if(minExpectedValue < 0){
      minExpectedValue = 0;
    }
    
    Serial.print("min: ");
    Serial.print(minExpectedValue);
    Serial.println();
    Serial.print("max: ");
    Serial.print(maxExpectedValue);
    Serial.println();
    Serial.print("Reported Flood Level: ");
    Serial.print(reportedFloodLevel);
    Serial.println();
    Serial.print("The mode/median is: ");
    Serial.print(mode);
    Serial.println();
  }
  
  if(abruptChangeTimer == 65){
   abruptChangeTimer = 0;
   reportedFloodLevel = mode;      //assigns new mode based on abrupt change in 65 sec
   minExpectedValue = mode - smsFrequency;
   maxExpectedValue = mode + smsFrequency;
   reportedFloodLevel = mode;
   //send new update to server
   String smsUpdate = "FLUPDATE " ;
   smsUpdate.concat(reportedFloodLevel);
   char charSmsMessage[130];
   smsUpdate.toCharArray(charSmsMessage, 130);
   send_msg("21583567", charSmsMessage);
  }
  Serial.println();
  Serial.print("abruptChangeTimer: ");
  Serial.println(abruptChangeTimer);
  Serial.println();
  
  delay(1000);
  
  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
  int distance = abs((uS / US_ROUNDTRIP_CM)/2.54);  //get distance in terms of inches
  distanceSamples[index] = floodHeightReference - distance;
  index = index + 1;
  
  if(distance == floodHeightReference){
    digitalWrite(referenceLightIndicator,HIGH);  
  }
  else{
    digitalWrite(referenceLightIndicator,LOW);
  }
  
  delay(10);
  Serial.print("Ping: ");
  Serial.print(distance); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  Serial.println("inches");
  
  if(distance < minExpectedValue){
    abruptChangeTimer = abruptChangeTimer + 1;
  }
  else if(distance > maxExpectedValue){
    abruptChangeTimer = abruptChangeTimer + 1;
  }
  else{
    abruptChangeTimer = 0;
  }
  //#####end of Ultrasonic Looping codes ###//
}

void send_msg(char *number, char *msgToSend)
{
  char at_cmgs_cmd[30] = {'\0'};
  char msg1[160] = {'\0'};
  char ctl_z = 0x1A;

  sprintf(msg1, "%s%c", msgToSend, ctl_z);
  sprintf(at_cmgs_cmd, "AT+CMGS=\"%s\"\r\n",number);
  
  sendGSM(at_cmgs_cmd);
  delay(100);
  delay(100);
  delay(100);
  sendGSM(msg1);
  delay(100);
}

void sendGSM(char *string){
  mySerial.write(string);
  delay(90);
}

void clearString(char *strArray) {
  int j;
  for (j = 100; j > 0; j--)
    strArray[j] = 0x00;
}

void send_cmd(char *at_cmd, char clr){
  char *stat = '\0';
  while(stat){
    sendGSM(at_cmd);
    delay(90);
    readSerialString(Rx_data);
    
    stat = strstr(Rx_data, "OK");
  }
  if (clr){
    clearString(Rx_data);
    delay(200);
    stat = '\0';
  }
}

void initGSM(){
  
  send_cmd("AT\r\n",1);						
//  send_cmd("ATE0\r\n",1); // Turn off automatic echo of the GSM Module	
	
  send_cmd("AT+CMGF=1\r\n",1);			// Set message format to text mode
  //Sucess
  
  Serial.println("Success");
	
  delay(1000);
  delay(1000);
  delay(1000);
}

void readSerialString (char *strArray) {
  
  if(!mySerial.available()) {
    Serial.println("not available");
    return;
  }
  
  while(mySerial.available()) {
    strArray[i] = mySerial.read();
    i++;
  }
}

/////////////////receive SMS///////////////////
void readSMS(){
  if(mySerial.available() > 0){
   Serial.println("read");
    while(mySerial.available() > 0){
      
      c = mySerial.read();
      Serial.print(c);
      if(c=='O'){
        
        c = mySerial.read();
        Serial.print(c);
        if(c=='K'){
        
          c = mySerial.read();
          Serial.print(c);
          isStart++; 
          
          if(isStart==3){
            isStart = 0;
            Serial.println("SMS is READY");
            isSmsReady = true;
            delay(500);
            
          }        
        }       
      }
      
      else if(c=='+'){
        
        c = mySerial.read();
        Serial.print(c);
        if(c == 'C'){
          c = mySerial.read();
          Serial.print(c);
          if(c == 'M'){
            c = mySerial.read();
            Serial.print(c);
            if(c == 'T'){
              c = mySerial.read();
              Serial.print(c);
              if(c == 'I'){
                for(int i = 0; i < 8; i++){
                
                  c = mySerial.read();
                  Serial.print(c);
                  delay(10);
                  
                }
              }
              Serial.println();
              mySerial.print("AT+CMGR=");
              mySerial.println(c);
              if(mySerial.available() > 0){
                
                c = mySerial.read();
                Serial.print(c);
                while(c != '+'){
                  c = mySerial.read();
                  delay(10);
                }
                c = mySerial.read();
                Serial.print(c);
                if(c=='C'){
                  c = mySerial.read();
                  Serial.print(c);
                  delay(10);
                  if(c == 'M'){
                    c = mySerial.read();
                    Serial.print(c);
                    delay(10);
                    
                    if(c == 'G'){
                     
                      c = mySerial.read();
                      Serial.print(c);
                      delay(10);
                      if(c == 'R'){
                      while(c != ','){
                        c = mySerial.read();
                        Serial.print(c);
                      }
                      Serial.print(c);
                      delay(10);
                      for(int readNumber = 0; readNumber<14; readNumber++){
                        c = mySerial.read();
                        Serial.print(c);
                        
                        senderNumber[readNumber] = c;
                      
                      }
                      
                    checkCommand();
                      
                      }
                    }
                  }
                }
              }    // end of AT+CMGR  

            }           
          }
        }      
      }         // end of +CMTI  
      
    } // end of while
  } // end of if mySerial
  
  
}   


void checkCommand(){

  while(c != '\r'){
    c = mySerial.read();
    Serial.print(c);
  }
  int cnt = 0;
  while(mySerial.available() > 0){
  
    c = mySerial.read();
    Serial.print(c); 
    msg[cnt] = c;
    cnt++;
  }
  
  messageRecieve = msg;
  messageRecieve.trim();
  messageRecieve.toUpperCase();  
 
  Serial.println("length is: ");
  Serial.println(messageRecieve.length());
  

  delay(1000);

  Serial.println();
  Serial.print("Messsage: ");
  Serial.print(messageRecieve);
  Serial.println("From");
  Serial.println(senderNumber);
  delay(100);
  
  if(messageRecieve.equals("RESEND")){
    Serial.println("Sending Status");
    //send new update to server
    String smsUpdate = "FLUPDATE " ;
    smsUpdate.concat(reportedFloodLevel);
    char charSmsMessage[130];
    smsUpdate.toCharArray(charSmsMessage, 130);
    send_msg("21583567", charSmsMessage);
  }
  else if(messageRecieve.equals("START")){
    digitalWrite(powerOn,HIGH); 
    delay(4000);
    digitalWrite(powerOn,LOW);
    delay(5000);
    send_msg("21583567", "STARTED");
  }
  else if(messageRecieve.equals("SHUTDOWN")){
    digitalWrite(powerOn,HIGH); 
    delay(4000);
    digitalWrite(powerOn,LOW);
    delay(5000);
    send_msg("21583567", "SHUTDOWN");
  }
  else{
  Serial.println("Invalid Keyword");
  delay(10);
  delSMS();
  }
}


void delSMS(){
  mySerial.println("AT+CMGD=1,4");
  delay(200);
}


//##########functions for ultrasonic##########//
//Function to print the arrays.
void printArray(int *a, int n) {

  for (int i = 0; i < n; i++)
  {
    Serial.print(a[i], DEC);
    Serial.print(' ');
  }

  Serial.println();

}

//Sorting function
// sort function (Author: Bill Gentles, Nov. 12, 2010)
void isort(int *a, int n){
// *a is an array pointer function

  for (int i = 1; i < n; ++i)
  {
    int j = a[i];
    int k;
    for (k = i - 1; (k >= 0) && (j < a[k]); k--)
    {
      a[k + 1] = a[k];
    }
    a[k + 1] = j;
  }

}

//Mode function, returning the mode or median.
int getMode(int *x,int n){

  int i = 0;
  int count = 0;
  int maxCount = 0;
  int mode = 0;
  int bimodal;
  int prevCount = 0;

  while(i<(n-1)){

    prevCount=count;
    count=0;

    while(x[i]==x[i+1]){

      count++;
      i++;
    }

    if(count>prevCount&count>maxCount){
      mode=x[i];
      maxCount=count;
      bimodal=0;
    }
    if(count==0){
      i++;
    }
    if(count==maxCount){//If the dataset has 2 or more modes.
      bimodal=1;
    }
    if(mode==0||bimodal==1){//Return the median if there is no mode.
      mode=x[(n/2)];
    }
    return mode;
  }

} 