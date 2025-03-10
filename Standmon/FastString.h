//#include <Arduino.h>
#include <WString.h>

class FastString : public String
{
  public:
    FastString(char *buff, int len, bool copy)
    {
        init();
        setBuffer(buff);
        setCapacity(len);
        setLen(len);
    }
    virtual ~FastString()
    {
        nullify();
    }
    void nullify()
    {
        setBuffer(nullptr);
        setCapacity(0);
    }
};