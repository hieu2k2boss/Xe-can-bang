//PID constants Cai dat tham so PID
double kp = 50 ;
double ki = 0.00 ;
double kd = 0.00075;

// dT la khoang thoi gian de lay tinh tich phan hoac dao ham cua error
double dT = 20;
// thoi gian hien tai va thoi gian truoc do (de xet loi o hai thoi diem)
unsigned long currentTime, previousTime;
double elapsedTime = 0;
double error;
double lastError;
double input, output, setPoint;
// cumError : loi tich luy theo thoi gian, rateError la toc do loi thay doi
double cumError, rateError = 0;

int motor1pin1 = 2;
int motor1pin2 = 3;

int motor2pin1 = 4;
int motor2pin2 = 5;
void setup ()
{
  // set setpoint in which the pendulum is vertical (Cai dat setpoint vi tri con lac dung yen)
  setPoint = 585;
  previousTime = millis();
  // set up for input of potentionmetter
  Serial.begin(9600);
  pinMode(A0, INPUT);
  // set up pins of two motors (cai dat cac chan motor dc)
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
}

void loop ()
{
  // read from the potentionmeter (doc thong so tu chiet ap)
  input = analogRead(A0);
  Serial.print(input);
  Serial.print("\t");
  // tra ve output khi dua input vao cong thuc tinh PID
  output = computePID(input);
  Serial.println(output);
  // Xet cac truong hop de quyet dinh xe tien hay lui giup can bang con lac
  if(output >= 0)
    backward(output);
  if(output < 0)
    forward(-output);
}

double computePID(double inp)
{
  // Tinh thoi gian troi di, neu bang dT thi chuyen den buoc tinh cumError va rateError
  elapsedTime = millis()-previousTime;
  //thoat vong lap khi thoi gian troi qua lon hon hoac bang dT
  while(elapsedTime < dT)
      elapsedTime = millis()-previousTime;
  // Tinh loi
  error = setPoint - inp;
  //Tinh tich phan cua loi
  cumError += error * dT;
  // Tinh dao ham cua loi
  rateError = 1000.0*(error - lastError)/(dT);
  //Tinh output voi cac tham so PID
  double out=kp*error + ki*cumError + kd*rateError;
  // chuyen loi thanh loi cu va tiep tuc do loi moi
  lastError = error;
  // reset lai thoi gian 
  previousTime = millis();
  // Neu out vuot qua gia tri cho phep gan cho out gia tri max hoac min
  if(out > 255)
    out = 255;
  else if(out <-255)
    out = -255;
  return out;
}

// Ham dieu khien dong co cho xe chay ve phia truoc co bien v xac dinh toc do dong co
void forward(int v)
{
  digitalWrite(motor1pin1, v);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, v);
  digitalWrite(motor2pin2, LOW);
}
// Ham dieu khien dong co cho xe chay ve phia sau
void backward(int v)
{
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, v);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, v);
}
// Coded by Le Ngoc Thanh
