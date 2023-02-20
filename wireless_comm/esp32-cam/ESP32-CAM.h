#ifndef ESP32CAM_h
#define ESP32CAM_h

class camclass
{

  public:
    camclass();
    void take_picture();
    void color_2_gray();
    void gray_2_color();
    void rotate_180();
    void spec_filt();
    void remove_filt();

};

extern camclass espcam;

#endif