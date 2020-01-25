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
