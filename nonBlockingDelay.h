#ifndef NON_BLOCKING_DELAY
#define NON_BLOCKING_DELAY

// Class that provides non-blocking delay functionality
class nblock_delay {
public:
  nblock_delay(const unsigned long interval)
    : interval_(interval),
      last_seen_(millis()) {}

  // Method that returns false if the specified interval has elapsed since the last time it was called,
  // otherwise returns true
  bool trigger() {
    if (millis() - last_seen_ > interval_) {
      last_seen_ = millis();
      return false;
    } else {
      return true;
    }
  }
  
  // Destructor
  ~nblock_delay() {}  
private:
  const unsigned long interval_;
  unsigned long last_seen_;
};

#endif
