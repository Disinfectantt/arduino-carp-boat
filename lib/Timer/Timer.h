#ifndef TIMER_H_
#define TIMER_H_

class Timer {
public:
  Timer() {}
  Timer(uint32_t nperiod) { start(nperiod); }
  void start(uint32_t nperiod) {
    period = nperiod;
    start();
  }
  void start() {
    timer = millis();
    if (!timer)
      timer = 1;
  }
  void stop() { timer = 0; }
  bool ready() {
    if (timer && millis() - timer >= period) {
      start();
      return true;
    }
    return false;
  }

private:
  uint32_t timer = 0, period = 0;
};

#endif
