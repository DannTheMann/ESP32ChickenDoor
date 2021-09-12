
#include <WiFi.h>
#include <WiFiUdp.h>
#include <DoorHandler.h>
#include "EEPROM.h"

/* Network credentials */
#define BACK_UP_WIFI        "Farm WiFi"
#define NETWORK_SSID        "Goats"
#define SSID_KEY            "windacres"
#define TARGET              "192.168.1.20"
#define UDP_PORT            3333
#define ONBOARDLED          2
/* Loop constraints */
#define POLLING_DELAY       1000        // Constant derived to a second
#define POLLING_PERIOD      5           // 5 seconds - frequency to check door
#define HEARTBEAT_PERIOD    15          // 15 seconds - frequency to check heartbeats
#define AUTOMATION_DELAY_X  900         // 900 seconds, aka 15m - default time to disable automation when door moves remotely
#define MAX_16BIT           65535
#define MAX_TIMEOUT_BEFORE_RESTART 5    // Attempts 5 times to connect to WiFi before soft resetting ESP32

/* ------------------------------------------ */
#define DEBUG 1
// end definitions
#ifdef DEBUG
    #define debug(x)   Serial.print (x)
    #define debugln(x) Serial.println (x)
    #define debugUpdate(x) update(x)
#else
    #define debugUpdate(x)
    #define debug(x)
    #define debugln(x)
#endif
/* ------------------------------------------ */

static bool eepromFailure   = false;
static bool automationDelay = true;
static uint16_t counter = 1; // Will count to 65,500.

static WiFiUDP     udp;//33,26,18,17
static DoorHandler door(25,32,34,36,35);

#define ILLEGAL_COMMAND '`' // Utilised in ParameterBuffer

/* Protective wrapper class that aims to abstract and handle
   complex parameter passing from the UDPSocket.            */
class ParameterBuffer
{

    public:
        ParameterBuffer(char* buf, int len)
        {
            m_command = len > 0 ? buf[0] : ILLEGAL_COMMAND;
            /* If this command has no parameters */
            if(len <= 1)
            {
                m_valid = false;
                m_argument = 0;
                m_length = 0;
            }
            /* Command has parameters            */
            else
            {
                /* Pre-emptive setup for parsing */
                m_valid = true;
                /* If value exceeds 255 (3 char) then max length at 3*/
                m_length = len-1 > 3 ? 3 : len-1;

                /* Find values, fill buffer and parse */
                char tmpBuf[m_length];
                for( uint8_t i = 1 ; i < len; i++)
                    tmpBuf[i-1] = buf[i];
                /* Parse the buffer into a uint8_t    */
                m_argument = atoi(tmpBuf);
            }
        }

        uint8_t getArgument() {return m_argument;                  }
        uint8_t getLength()   {return m_length;                    }
        char    getCommand()  {return m_command;                   }
        char*   getBuffer()   {return m_buf;                       }
        bool    hasParameter(){return m_valid;                     }
        bool    hasCommand()  {return m_command != ILLEGAL_COMMAND;}

    private:
        uint8_t m_argument;
        uint8_t m_length;
        char    m_command;
        char    m_buf[5];
        bool    m_valid;

};

/* Forward Declaration */
bool acknowledge();
void update(const char*);
bool update(const char*, uint8_t);
bool interpretPacketCommand(ParameterBuffer);
void morseFlash(const char*);
void connectToNetwork();

void setup()
{

    #ifdef DEBUG
        Serial.begin(115200);
        Serial.print("Setup: Starting... ");
        Serial.print(__TIME__);
        Serial.print(" ");
        Serial.print(__DATE__);
        Serial.println();
    #endif

    debugln("Setup()");

    pinMode(ONBOARDLED, OUTPUT);

    debugln("Setup() Attempting to connect to network.");
    // delete old config
    WiFi.disconnect(true);

    delay(1000);
    connectToNetwork();

    if (!EEPROM.begin(64))
    {
      debugln("Setup() failed to initialise EEPROM"); 
      eepromFailure = true;
      return;
    }

    door.loadSettings();
    DoorHandler::Response r = door.getState();

    debugln(r.getResponse());
    debugln("Setup() EEPROM setup");
    
    update("Setup() Initialised - Door Controller starting.");

    // LDR
    pinMode(4, INPUT);  

    // Once connected configure NTP
    door.configureNTP();

}

bool pollDoor()
{
    debugln("pollDoor()");
    /* If the automation is disabled then return */
    if( (counter % AUTOMATION_DELAY_X != 0) && automationDelay)
    {
        debug("pollDoor(), automationDelay=1, time until automation enabled=");
        debugln(AUTOMATION_DELAY_X-(counter%AUTOMATION_DELAY_X));
        char tmp[200];
        sprintf(tmp, "pollDoor() automationDelay=1, time until automation enabled=%d", AUTOMATION_DELAY_X-(counter%AUTOMATION_DELAY_X));
        debugUpdate(tmp);
        return false;
    }
    else if (automationDelay) automationDelay = false;

    bool doorMoved = door.poll();

    debugUpdate(doorMoved ? "pollDoor() Door has moved" : "pollDoor() Door has not moved");
    debug("pollDoor() Door moved = ");
    debugln(doorMoved ? "true":"false");

    if(doorMoved)
    {
        debugUpdate(door.isClosed() ? "pollDoor() Door has closed just now." : "pollDoor() Door has opened just now.");
        debugln    (door.isClosed() ? "pollDoor() Door has closed just now." : "pollDoor() Door has opened just now.");
        if(door.isClosed()) update("pollDoor() DM:0");
        if(door.isOpen())   update("pollDoor() DM:1");

        // Light may cause problems, we don't want the door flinging open and closed every 30 seconds
        if(door.ldrEnabled() && !door.timeEnabled())
        {
          counter = 1;
          automationDelay = true;
        }
    }

    return doorMoved;
}

bool pollNetwork()
{ 
    debug("pollNetwork() DeviceIP: ");
    debugln(WiFi.localIP());
    int packetLength = udp.parsePacket();

    /* Have we received messages */
    if(packetLength > 0)
    {

        debug("pollNetwork() Message received, length=");
        debugln(packetLength);
        debugUpdate("pollNetwork() Message received.");
        /* Retrieve packet, flush the socket and parse */
        char incomingPacket[packetLength];
        udp.read(incomingPacket, packetLength);

        interpretPacketCommand(ParameterBuffer(incomingPacket, packetLength));
    }
    else
    {
        debugln("pollNetwork() No new messages.");
        debugUpdate("pollNetwork() No new messages.");
    }
    udp.flush();
    return packetLength > 0;
}

void moveDoor(bool direction)
{
    /* Regardless of direction, did the door move? */
    debug("moveDoor() - ");
    debug(direction ? "Opening":"Closing");
    bool result = door.moveDoor(direction);
    debug("moveDoor() Door moved: ");
    debugln(result ? "true":"false");
    if(result)
    {
        /* Manual moved, as such delay automation from
           forcing the door back in a previous state  */
        counter = 1;
        automationDelay = true;
    }
}

/* Main body of code, called continiously */
void loop()
{
    debug("loop() ");
    debug(counter);

    if(eepromFailure)
    {
        debugln("loop() eeprom has failed.");
        debugUpdate("EEPROM has failed.");
        morseFlash(".---.---");
        return;  
    }
    
    if(door.ldrEnabled())
    {
      debug(" - Light: ");
      debugln(door.getLightLevel());  
    }
    else  debugln("");  
    
    if(WiFi.status() == WL_CONNECTED)
    {
        if( counter % POLLING_PERIOD   == 0 ) pollDoor();

        if( counter % POLLING_PERIOD   == 0 ) pollNetwork();

        if( counter % HEARTBEAT_PERIOD == 0 ) acknowledge();

    }
    else
    {
        // Attempt reconnect if not connected
        if( counter % HEARTBEAT_PERIOD == 0 )
        {
          debugln("loop() Currently disconnected. Attempting a reconnect");
          connectToNetwork();
        }
        if(door.ldrEnabled())
        {
            pollDoor();
        }
    }
    /* Don't spam the server */
    delay(POLLING_DELAY);

    /* Wrap the counter between 0 - MAX_16BIT */
    counter++;
    if(counter >= MAX_16BIT) counter=1;
}

bool interpretPacketCommand(ParameterBuffer pb)
{
    debug("interpretPacketCommand(");
    debug("cmd=");
    debug(pb.getCommand());
    debug(", hasParam=");
    debug(pb.hasParameter());
    debug(", arg=");
    debug(pb.getArgument());
    debugln(")");

    char tmp[100];
    sprintf(tmp, "interpretPacketCommand() Parsing command: %u, hasParam: %d - %d", pb.getCommand(), pb.hasParameter(), pb.getArgument());
    debugUpdate(tmp);
    
    switch(pb.getCommand())
    {
        case '0': // disable/enable automation
            if(pb.hasParameter()) door.setAutomated(pb.getArgument());
            break;
        case '1': // disable/enable LDR
             if(pb.hasParameter()) door.setAutomated(pb.getArgument());
            break;
        case '2': // move door
             if(pb.hasParameter()) moveDoor(pb.getArgument());
            break;
        case '4': // set motor top position
            if(pb.hasParameter()) door.setTopPosition(pb.getArgument());
            break;
        case '5': // set door lower light threshold
            if(pb.hasParameter()) door.setLightLowerThreshold(pb.getArgument());
            break;
        case '6': // set door upper light threshold
            if(pb.hasParameter()) door.setLightUpperThreshold(pb.getArgument());
            break;
        case '7': // set door ID
            if(pb.hasParameter()) door.setDoorId(pb.getArgument());
            break;
        case '8': // add time to door closing time
            if(pb.hasParameter()) door.setDoorCloseTime(pb.getArgument());
            break;
            break;
        case 'a': // disable automation delay (Door will 'refresh')
            automationDelay = 0;
            break;
        case 'm': // Do we save the motors position in EEPROM?
            if(pb.hasParameter()) door.setMotorSaved(pb.getArgument());
            break;
        case 'n': // Motor move speed - not currently used
            if(pb.hasParameter()) door.setMotorMoveSpeed(pb.getArgument());
            break;
        case 'f': // Factory reset
            door.factoryReset();
        case 'h': // Help
            debugUpdate("interpretPacketCommand() Displaying help. \n1 [1:0]=LDR on\n2 [1:0]=MV Door\n4 [0-255]=MTR Top\n5 [0-255]=LWR Light\n6 [0-255]=UPR Light\n7 [0-255]=ID\n8 [0-255]=Open Time\n9 [0-255]=Close Time\na=Disable Automation delay\nm [1:0]=SaveMTRPos\nf=Reset\no=Open\nc=Close\n");
            break;
        case 'o': // Set door to open
            debugln("interpretPacketCommand() Forcing door to be open.");
            door.forcedOpen();
            counter = 1;
            automationDelay = true;
            break;
        case 'c': // Set door to closed
            debugln("interpretPacketCommand() Forcing door to be closed.");
            door.forcedClosed();
            counter = 1;
            automationDelay = true;
            break;
        case 'p': // reserved
            break;
        case 'd': // reserved
            break;
        case 'l': // Light enable/disable
             if(pb.hasParameter()) door.setLDREnabled(pb.getArgument());
            break;
        case 't': // Time enable/disable
             if(pb.hasParameter()) door.setTimeEnabled(pb.getArgument());
            break;
        case 'r': // Restart ESP32
             debugln("interpretPacketCommand() restart issued.");
             delay(1000);
             ESP.restart();
             break;
        case ILLEGAL_COMMAND:  // Unrecongised command
            debugln("interpretPacketCommand() Command received was invalid.");
            return false;
        default: // Failure case
            debugln("interpretPacketCommand() Unrecognised command!");
            return false;
    }
    return true;
}

/* Simple ack function */
bool acknowledge()
{   
    debugln(WiFi.status() == WL_CONNECTED ? "acknowledge() We're connected." : "acknowledge() !We're disconnected!");
    if(WiFi.status() == WL_CONNECTED)
    {
        debugUpdate("acknowledge() Acknowledging host.");
        DoorHandler::Response response = door.getState();
        debug("acknowledge() Response: ");
        debugln(response.getResponse());
        return update(response.getResponse(), response.getLength());
    }
    return false;
}

void update(const char* strBufffer)
{
    update(strBufffer, strlen(strBufffer));
}

bool update(const char* strBufffer, uint8_t len)
{
    uint8_t id = door.getID();
    char tmp[len+10]; // Prepare to expand buffer to fit ID into
    sprintf(tmp, "(ID:%d)-%s", id, strBufffer);

    if(WiFi.status() == WL_CONNECTED)
    {
        udp.beginPacket(TARGET, UDP_PORT);
        udp.printf(tmp);
        udp.endPacket();
    }
    return WiFi.status() == WL_CONNECTED;
}

void morseFlash(const char* message)
{
  uint8_t len = strlen(message);
  for(uint8_t i = 0; i < len; i++)
  {
    if(message[i] == '-')
    {
        digitalWrite(ONBOARDLED, HIGH);
        delay(1000);
    }
    else if(message[i] == '.')
    {
      digitalWrite(ONBOARDLED, HIGH);
      delay(500);  
    }
    digitalWrite(ONBOARDLED, LOW);
    delay(750);
  }  
}

/*
-----------------------------------------------------------
----------------- WIFI        -----------------------------
-----------------------------------------------------------
*/
void connectToNetwork()
{
    // If after so many attempts we can't connect to the defacto WiFi then try Farm WiFi
    static uint8_t alternativeWiFi = 1;
    
    debug("connectToNetwork() alternativeWiFiCounter=");
    debug(alternativeWiFi);
    debug(" Awaiting to connect to - ");
    
    digitalWrite(ONBOARDLED, HIGH);
    // delete old config
    WiFi.disconnect();

    delay(2500);
    
    digitalWrite(ONBOARDLED, LOW);

    if(alternativeWiFi >= MAX_TIMEOUT_BEFORE_RESTART)
    {
      // This will perform a soft restart, will not restart hardware peripherals or I/O though.
      debugln("connectToNetwork() giving up, performing soft reset.");
      delay(2500);
      ESP.restart();
    }
    else if(alternativeWiFi % 3 == 0)
    {
      // If after 3 attempts of connecting to the defacto wifi, then try an alternative one
      WiFi.begin(BACK_UP_WIFI, SSID_KEY);
      debug(" (BACK_UP) ");
      debug(BACK_UP_WIFI);
    }
    else
    {
      WiFi.begin(NETWORK_SSID, SSID_KEY);
      debug(NETWORK_SSID);
    }
    debug(" ");

    delay(2500);

    /* Allow the network a second to adjust */
    for(uint8_t i = 0; i < 5 && WiFi.status() != WL_CONNECTED ; i++)
    {
      delay(500);
      digitalWrite(ONBOARDLED, HIGH);
      debug(".");
      delay(500);   
      digitalWrite(ONBOARDLED, LOW);
    }
    debugln("");
    if(WiFi.status() == WL_CONNECTED)
    { 
      debug("connectToNetwork() Connected, IP address: ");
      debugln(WiFi.localIP());
      udp.begin(WiFi.localIP(),UDP_PORT);
      digitalWrite(ONBOARDLED, HIGH);
      update("connectToNetwork() Connected.");
      alternativeWiFi = 1;
    }
    else    
    {
      digitalWrite(ONBOARDLED, LOW);
      debugln("connectToNetwork() Couldn't connect to WiFi.");
      alternativeWiFi++;
    }
}
