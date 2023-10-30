int dir1 = 2, pwm1 = 3, dir2 = 40, pwm2 = 41;
// ****************************** Setup ******************************
void setup()
{
    pinMode(2 , OUTPUT);
    pinMode(3 , OUTPUT);
    pinMode(4 , OUTPUT);
    pinMode(5 , OUTPUT);
}

void forward(int vel) // -------------- Forward
{
    analogWrite(pwm1, vel);
    analogWrite(pwm1, vel);

    digitalWrite(dir1, 0);
    digitalWrite(dir2, 1);
}

void backward(int vel) // -------------- Backward
{
    analogWrite(pwm1, vel);
    analogWrite(pwm1, vel);

    digitalWrite(dir1, 1);
    digitalWrite(dir2, 0);
}

void left(int vel) // -------------- Left
{
    analogWrite(pwm1, vel);
    analogWrite(pwm1, vel);

    digitalWrite(dir1, 1);
    digitalWrite(dir2, 1);
}

void right(int vel) // -------------- Right
{
    analogWrite(pwm1, vel);
    analogWrite(pwm1, vel);

    digitalWrite(dir1, 0);
    digitalWrite(dir2, 0);
}

void stop(int vel) // -------------- Stop
{
    analogWrite(pwm1, 0);
    analogWrite(pwm1, 0);
}
// ****************************** Loop ******************************
void loop()
{
    forward(128);
    delay(3000);

    backward(128);
    delay(3000);

    left(128);
    delay(3000);

    right(128);
    delay(3000);

    stop(0);
    delay(3000);
}