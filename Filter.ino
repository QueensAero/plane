//public class KalmanFilter {

  float Q = 0.000001;
  float R = 0.0001;
  float P = 1, X = 0, K;

  //public KalmanFilter() {
    //P = 1.0f;
    //X = 0.0f;
    //K = 0.0f;
  //}

  void measurementUpdate() {
    K = (P + Q) / (P + Q + R);
    P = R * (P + Q) / (R + P + Q);
  }

  float updateAltitudeFtFilter(float measurement) {
    measurementUpdate();
    float result = X + (measurement - X) * K;
    X = result;
    return result;
  }

//}
