unsigned char left_line__track_Sensor=0;        //state of left sensor
unsigned char middle_line__track_Sensor=0;        //state of middle sensor
unsigned char right_line__track_Sensor=0;        //state of right sensor

void setup(){
  Serial.begin(9600);
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  pinMode(11, INPUT);
}

void loop(){
  left_line__track_Sensor = digitalRead(9);
  middle_line__track_Sensor = digitalRead(10);
  right_line__track_Sensor= digitalRead(11);
  Serial.print("left_line__track_Sensor = ");
  Serial.println(left_line__track_Sensor);
  Serial.print("middle_line__track_Sensor = ");
  Serial.println(middle_line__track_Sensor);
  Serial.print("right_line__track_Sensor = ");
  Serial.println(right_line__track_Sensor);
  Serial.print("\n");
  delay(1000);
}

