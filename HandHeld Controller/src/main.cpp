#include <SPI.h>
#include <RH_RF95.h>
#include <TinyGPSPlus.h>
#include <Adafruit_BMP3XX.h>

RH_RF95 rf95(10, 8); // Singleton instance of the radio driver
TinyGPSPlus gps;
bool position = false;
uint32_t printTime = 0;
bool initialized = false;

bool keyk = false;
bool keyq = false;
bool keya = false;
bool keyw = false;
bool keys = false;
bool keye = false;
bool keyd = false;
bool keyr = false;
bool keyf = false;
bool keyspace = false;
int incomingByte = 0;

void setup()
{
    Serial.begin(9600);
    Serial1.begin(9600);
    if (rf95.init())
    {
        initialized = true;
    }                           // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
    rf95.setTxPower(20, false); // The default transmitter power is 13dBm, using PA_BOOST. - If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then you can set transmitter powers from 2 to 20 dBm:
    printTime = micros();
}

void loop()
{
    uint32_t time = micros();
    // while (Serial1.available() > 0)
    // {
    //     gps.encode(Serial1.read());
    //}

    while (Serial.available() > 0)
    {
        incomingByte = Serial.read();
        {
            Serial.println(char(incomingByte));
            if (incomingByte == 'k')
            {
                Serial.println("key:k");
                keyk = true;
            }
            if (incomingByte == ' ')
            {
                Serial.println("spacebar");
                keyspace = true;
            }
            if (incomingByte == 'q')
            {
                Serial.println("key:q");
                keyq = true;
            }
            if (incomingByte == 'a')
            {
                Serial.println("key:a");
                keya = true;
            }
            if (incomingByte == 'w')
            {
                Serial.println("key:w");
                keyw = true;
            }
            if (incomingByte == 's')
            {
                Serial.println("key:s");
                keys = true;
            }
            if (incomingByte == 'e')
            {
                Serial.println("key:e");
                keye = true;
            }
            if (incomingByte == 'd')
            {
                Serial.println("key:d");
                keyd = true;
            }
            if (incomingByte == 'r')
            {
                Serial.println("key:r");
                keyr = true;
            }
            if (incomingByte == 'f')
            {
                Serial.println("key:f");
                keyf = true;
            }
        }
    }

    if (keyk == true)
    {
        uint8_t data[] = "key:k";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
        Serial.print("Sent key:k to rf95_server: ");
        keyk = false;
    }

    if (keyspace == true)
    {
        uint8_t data[] = "key: ";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
        Serial.print("Sent key:space to rf95_server: ");
        keyspace = false;
    }

    if (keyq == true)
    {
        uint8_t data[] = "key:q";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
        Serial.print("Sent key:q to rf95_server: ");
        keyq = false;
    }
    if (keya == true)
    {
        uint8_t data[] = "key:a";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
        Serial.print("Sent key:a to rf95_server: ");
        keya = false;
    }
    if (keyw == true)
    {
        uint8_t data[] = "key:w";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
        Serial.print("Sent key:w to rf95_server: ");
        keyw = false;
    }
    if (keys == true)
    {
        uint8_t data[] = "key:s";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
        Serial.print("Sent key:s to rf95_server: ");
        keys = false;
    }
    if (keye == true)
    {
        uint8_t data[] = "key:e";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
        Serial.print("Sent key:e to rf95_server: ");
        keye = false;
    }
    if (keyd == true)
    {
        uint8_t data[] = "key:d";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
        Serial.print("Sent key:d to rf95_server: ");
        keyd = false;
    }
    if (keyr == true)
    {
        uint8_t data[] = "key:r";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
        Serial.print("Sent key:r to rf95_server: ");
        keyr = false;
    }
    if (keyf == true)
    {
        uint8_t data[] = "key:f";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
        Serial.print("Sent key:f to rf95_server: ");
        keyf = false;
    }

    if (rf95.available())
    {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);

        if (rf95.recv(buf, &len))
        {
            Serial.println((char *)buf);
        }
    }
}
