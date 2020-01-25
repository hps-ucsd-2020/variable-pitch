// Declare like Circularbuffer<type, size> var_name
template <typename Data, int Size>
class CircularBuffer{
public:
  CircularBuffer(): bufpos(0), buffer_full(false){}

  // adds the parameter to the buffer as the newest element
  // removes and returns the oldest element
  Data add(Data to_add) {
    Data removed = buf[bufpos];
    buf[bufpos] = to_add;
    bufpos++;
    // check for loop
    if(bufpos >= Size) {
      bufpos = 0;
      buffer_full = true;
    }
    return removed;
  }

  // sets all values in the buffer to 0, and sets the buffer to not full
  void clear_buffer() {
    for(int i = 0; i < Size; i++){
      buf[i] = 0;
    }
    bufpos = 0;
    buffer_full = false;
  }

  // returns the average of all the data in the buffer
  double average(){
    int data_points = buffer_full? Size : bufpos;
    double sum = 0;

    for(int i = 0; i < data_points; i++) {
      sum += buf[i];
    }

    return (data_points == 0)? 0 : sum / (double)data_points;
  }

private:
  Data buf[Size] = {}; //values initialized to NULL
  int bufpos; // position in the buffer
  bool buffer_full; // have we filled all the values in the buffer?
};

// set TEST to 1 to compile the test suite
#define TEST 0
#if TEST
  #define SERIAL_BAUD 9600
  CircularBuffer<int, 5> buf;
  void setup(){
    Serial.begin(SERIAL_BAUD);
    Serial.print("Circular Buffer TEST\n\n");
  }
  
  void loop(){
    bool test_result = false;
    // Test average when empty
    test_result = 0.0 == buf.average();
    Serial.print("Test average when empty: ");
    Serial.println(test_result);
    // Test add first element
    test_result = 0 == buf.add(4);
    Serial.print("Test add one element: ");
    Serial.println(test_result);
    // Test average with one element
    test_result = 4.0 == buf.average();
    Serial.print("Test average with one element: ");
    Serial.println(test_result);
    // Test average with two elements
    buf.add(2); // buf is now 4, 2
    test_result = 3.0 == buf.average();
    Serial.print("Test average with two elements: ");
    Serial.println(test_result);
    // Add three more elements, test average with full buffer
    buf.add(6);
    buf.add(0);
    buf.add(-2);
    // buf is now 4, 2, 6, 0, -2. Average is 10/5 = 2
    test_result = 2.0 == buf.average();
    Serial.print("Test average with full buffer: ");
    Serial.println(test_result);
    // Test add removes and returns the correct value, test average to make sure it adds the value correctly
    test_result = 4 == buf.add(-1);
    Serial.print("Test add removes correct value: ");
    Serial.println(test_result);
    // -1, 2, 6, 0, -2. Average is 5/5 = 1
    test_result = 1.0 == buf.average();
    Serial.print("Test average returns correct value after full rotation: ");
    Serial.println(test_result);
    // Test add and average twice more
    test_result = 2 == buf.add(7);
    Serial.print("Test add removes correct value 2: ");
    Serial.println(test_result);
    // -1, 7, 6, 0, -2. Average is 10/5 = 2
    test_result = 2.0 == buf.average();
    Serial.print("Test average returns correct value after full rotation 2: ");
    Serial.println(test_result);
    test_result = 6 == buf.add(1);
    Serial.print("Test add removes correct value 3: ");
    Serial.println(test_result);
    // -1, 7, 1, 0, -2. Average is 5/5 = 1
    test_result = 1.0 == buf.average();
    Serial.print("Test average returns correct value after full rotation 3: ");
    Serial.println(test_result);
    // test clearbuf
    buf.clear_buffer();
    test_result = 0.0 == buf.average();
    Serial.print("test clearbuf: ");
    Serial.println(test_result);
    while(1);
  }
#else
  void setup(){}
  void loop(){}
#endif
