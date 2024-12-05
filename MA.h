const int windowSize = 10;  // Set the size of the moving average window


struct MovingAverage {
  int dataBuffer[windowSize];  // Buffer to store the last 'windowSize' data points
  int dataIndex = 0;           // Index to keep track of the current position in the buffer
  int sum = 0;                 // Variable to store the sum of the data points in the buffer

  float calculate(int newValue) {
    // Subtract the oldest value from the sum
    sum -= dataBuffer[dataIndex];

    // Add the new value to the sum
    sum += newValue;

    // Store the new value in the buffer
    dataBuffer[dataIndex] = newValue;

    // Move to the next position in the buffer
    dataIndex = (dataIndex + 1) % windowSize;

    // Calculate and return the moving average
    return static_cast<float>(sum) / windowSize;
  }
};

