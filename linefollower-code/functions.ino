uint16_t average(Pin a) {
  const int sample = 10;
  unsigned int sum = 0;
  for (int i = 0; i < sample; i++) sum += a.getAnalogValue();
  return sum / sample;
}

//9 - левый мотор, 10 - правый мотор
Pin pwmA = Pin(9);  Pin pwmB = Pin(10);
void motorInit() {
  pwmA.setOutput(); pwmB.setOutput();
  pwmA.setDutyCycle(0); pwmB.setDutyCycle(0);
}

void motorSpeed(int speedA, int speedB) {
  // ограничение максимальной скорости и исключение работы мотора в обратном направлении
  speedA = constrain(speedA, 0, 255);
  speedB = constrain(speedB, 0, 255);

  pwmA.setDutyCycle(speedA);
  pwmB.setDutyCycle(speedB);
}
