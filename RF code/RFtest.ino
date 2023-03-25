#include <SoftwareSerial.h> 

SoftwareSerial mySerial(2,3);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(2, INPUT);
  pinMode(3, OUTPUT); 
  mySerial.begin(9600);
  delay(100);
  int bw = 1; // bandwidth in KHz ( 0= 12.5KHz or 1= 25KHz )
  float ftx = 144.9000; // tx frequency in MHz (134.0000 - 174.0000)
  float frx = 144.9000; // rx frequency in MHz (134.0000 - 174.0000)
  String tx_ctcss = "0000"; // ctcss frequency ( 0000 - 0038 ); 0000 = "no CTCSS" 
  String rx_ctcss = "0000"; // ctcss frequency ( 0000 - 0038 ); 0000 = "no CTCSS" 
  int squ = 0; // squelch level ( 0 - 8 ); 0 = "open" 

  Serial.print("AT+DMOSETGROUP="); // begin message
  Serial.print(bw,1);
  Serial.print(",");
  Serial.print(ftx,4);
  Serial.print(",");
  Serial.print(frx,4);
  Serial.print(",");
  Serial.print(tx_ctcss);
  Serial.print(",");
  Serial.print(squ);
  Serial.print(",");
  Serial.println(rx_ctcss);
  delay(100);
  /*
  mySerial.print("AT+DMOSETGROUP="); // begin message
  mySerial.print(bw,1);
  mySerial.print(",");
  mySerial.print(ftx,4);
  mySerial.print(",");
  mySerial.print(frx,4);
  mySerial.print(",");
  mySerial.print(tx_ctcss);
  mySerial.print(",");
  mySerial.print(squ);
  mySerial.print(",");
  mySerial.println(rx_ctcss);*/ 
  char nameVar;
  while (Serial.available() > 0 ) {   
  
    nameVar = Serial.read(); 
    //Serial.print(nameVar);
    //delay(100);
  }

  Serial.println("AT+DMOSETVOLUME=8"); 

  //mySerial.listen();
  delay(1000);
  
  while (Serial.available() > 0 ) {   
  
    nameVar = Serial.read(); 
    Serial.print(nameVar);
    //delay(100);
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  
  
  
  //Serial.println("AT+DMOCONNECT");
  //Serial.println();
  
  //Serial.println();
}
