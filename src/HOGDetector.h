
#ifndef HOGDETECTOR_H
#define HOGDETECTOR_H

namespace HOGDetector

struct Square {
  int x;
  int y;
  int w;
  int h;
};

class DetectorResult {
 public:
  DetectorResult();
  ~DetectorResult();

  void add(Square result);

  Square* get();
  int length();

 private:
  std::vector<DetectorResult> results;
};

class HOGDetector {

 public:
  HOGDetector();
  ~HOGDetector();

  

 private:

};

#endif
