#include <DoorHandler.h>
#include <EEPROM.h>
#include <Dusk2Dawn.h>
#include <ctime>
#include "time.h"

#define MIN_DIFF_IN_LIGHT   5
#define MOTOR_STEP_DELAY    1    // 100 millisecond delay
#define ENCODER_MULTIPLIER  3000

/* Macros for struct time       */
#define SECOND              0
#define MINUTE              1
#define HOUR                2
#define MONTH               3
#define YEAR                4
#define TDAY                5
#define DST                 6

/* Macros for setting saving     */
#define EEPROM_SIZE             64
#define DOOR_ID                 0
#define MTR_POSITION            1
#define MTR_STOP_TOP            2
#define LIGHT_THRESHOLD_TOP     3
#define LIGHT_THRESHOLD_BOTTOM  4
#define OPEN_OFFSET             5
#define AUTOMATION_ENABLE       6
#define LDR_ENABLE              7
#define MOTOR_SAVED_ENABLE      8
#define MOTOR_MOVE_TIME         9
#define TIME_ENABLE             10
#define RESET                   99

/* Macros for door and time      */
#define DAY                     true
#define NIGHT                   false
#define OPEN_DOOR               true
#define CLOSE_DOOR              false

/* Default settings, written to EEPROM on init */
#define D_MTR_STOP_TOP            10
#define D_LIGHT_THRESHOLD_TOP     37    // Assumed with 200k~ LDR and 10k resistor in voltage divider
#define D_LIGHT_THRESHOLD_BOTTOM  25
#define D_OPEN_OFFSET             0
#define D_AUTOMATION_ENABLE       true
#define D_LDR_ENABLE              false
#define D_MOTOR_SAVED_ENABLE      true
#define D_MOTOR_MOVE_TIME         75 // This value represents n*100ms where ms is milliseconds
#define D_TIME_ENABLE             true

#define DEBUG 1
// end definitions
#ifdef DEBUG
    #define debug(x)   Serial.print (x)
    #define debugln(x) Serial.println (x)
#else
    #define debug(x)
    #define debugln(x)
#endif

/* Latitude and Longititude */
static float latitude    = 51.1497923;
static float longititude = -0.23745;

//51.1497923,-0.23745

/* Network Time Protocol (NTP) Parameters */
static const char*  ntpServer = "pool.ntp.org";
static const long   gmtOffset_sec = 0;
static const int    daylightOffset_sec = 3600;

static Dusk2Dawn movingTime(51.1497923,-0.23745, 0);

/* Forward Declarations */
void saveSetting(int);

/* -------------------------------------------------------------------*/
/* ------------------------ PUBLIC FUNCTIONS -------------------------*/
/* -------------------------------------------------------------------*/


/* --------------------------- CONSTRUCTOR ---------------------------*/

/* Creates a doorhandler which aims to provide functionality */
DoorHandler::DoorHandler(uint8_t mtrPin1, uint8_t mtrPin2, uint8_t encoderPin1, uint8_t encoderPin2, uint8_t ldrPin)
: m_mtrPin1(mtrPin1),
  m_mtrPin2(mtrPin2),
  m_encoder(encoderPin1, encoderPin2),
  m_ldrPin(ldrPin)
{
    /* Set these to 'off' by default until EEPROM is ready */
    m_ldrEnabled  = 0;
    m_timeEnabled = 0;
    //seedRandomNumberGenerator();
    // /* EEPROM loading */
    //loadSettings();

    /* GPIO Pin setup */
    m_mtrPin1 = mtrPin1;
    m_mtrPin2 = mtrPin2;
    m_ldrPin = ldrPin;
    pinMode(m_mtrPin1, OUTPUT);
    pinMode(m_mtrPin2, OUTPUT);

    // /* GPIO for LDR if used */
    pinMode(m_ldrPin, INPUT);

    m_closed = true;
    m_eepromNeedsSaving = false;
    /* If the door lost power or became stuck try to close */
    // if( isMoving() )
    //     moveDoor(false);

    // calculateTimeToMove();
}

void DoorHandler::configureNTP()
{
    /* NTP Configuration */
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    calculateTimeToMove();
}

/* ----------------------------- SETTERS -----------------------------*/
uint8_t DoorHandler::setLightUpperThreshold(uint8_t value)
{
    /* If the value suggested is inadequate then stop here. */
    if(value <= 0 || value <= m_lightLowerThreshold || value >= 255)
    {
        return m_lightUpperThreshold;
    }
    else
    {
        // Need signed so we don't underflow
        int8_t difference = value - m_lightLowerThreshold;
        if(difference >= MIN_DIFF_IN_LIGHT)
        {
            saveSetting(LIGHT_THRESHOLD_TOP);
            m_lightUpperThreshold = value;
        }
        return m_lightUpperThreshold;
    }
}

uint8_t DoorHandler::setLightLowerThreshold(uint8_t value)
{
    /* If the value suggested is inadequate then stop here. */
    if(value > 255 || value >= m_lightUpperThreshold)
    {
        return m_lightLowerThreshold;
    }
    else
    {
        int8_t difference = m_lightUpperThreshold - value;
        if(difference >= MIN_DIFF_IN_LIGHT)
        {
            saveSetting(LIGHT_THRESHOLD_BOTTOM);
            m_lightLowerThreshold = value;
        }
        return m_lightLowerThreshold;
    }
}

uint8_t DoorHandler::setTopPosition(uint8_t value)
{
    /* Setting an incredibly high value can be dangerous - caution advised */
    if(value > 0 && value < 255)
    {
        m_motorTopPosition = value;
        saveSetting(MTR_STOP_TOP);
    }
    return m_motorTopPosition;
}

uint8_t DoorHandler::setDoorId(uint8_t value)
{
    /* Setting an incredibly high value can be dangerous - caution advised */
    if(value > 0 && value < 255)
    {
        m_id = value;
        saveSetting(DOOR_ID);
    }
    return m_id;
}

uint8_t DoorHandler::setMotorMoveSpeed(uint8_t value)
{
    if(value == 0) value = 1;
    m_motorMoveTime = value;
    return m_motorMoveTime;
}

void DoorHandler::setDoorOpenTime(uint8_t value)
{
    int16_t tmp = m_minuteOffset + value;
    if(value > 255)     m_minuteOffset = 255;
    else if(value < 0)  m_minuteOffset = 0;
    else                m_minuteOffset = value;
}

void DoorHandler::setTimeEnabled(bool flag)
{
    if(m_ldrEnabled != flag)
    {
        m_timeEnabled = flag;
        saveSetting(TIME_ENABLE);
    }
}

void DoorHandler::setLDREnabled(bool flag)
{
    if(m_ldrEnabled != flag)
    {
        m_ldrEnabled = flag;
        saveSetting(LDR_ENABLE);
    }
}

void DoorHandler::setMotorSaved(bool flag)
{
    if(flag == m_motorPositionSaved) return;

    /* Disabling saving mtr position */
    if(!flag)
    {
        debug("setMotorSaved() Disabling motor saving...");
        debug(" - Door closed: ");
        m_closed = isClosed();
        debugln(m_closed);
    }
    else // Enabling
    {
        debug("setMotorSaved() Enabling motor saving...");
        if(m_closed)

        {
            m_motorPosition = 0;
        }
        else
        {
            m_motorPosition = m_motorTopPosition;
        }
        debug(" m_closed=");
        debug(m_closed);
        debug(" - MTR_POS: ");
        debugln(m_motorPosition);
    }
    m_motorPositionSaved=flag;
}

void DoorHandler::forcedOpen()
{
    m_closed = false;
    m_motorPosition = m_motorTopPosition;
    m_encoder.write(m_motorPosition*ENCODER_MULTIPLIER);
    saveSettings();
}

void DoorHandler::forcedClosed()
{
    m_closed = true;
    m_motorPosition = 0;
    m_encoder.write(0);
    saveSettings();   
}

/* ------------------------ GENERAL FUNCTIONS ------------------------*/

bool DoorHandler::moveDoor(bool direction)
{

    if(
        (  direction && isOpen() )
            ||
        ( !direction && isClosed() ) )
    {

        /* Door is already open/closed */
        return false;

    }

    /* direction=true (Open door), direction=false (Close door) */

    /* Power Motor */
    digitalWrite(m_mtrPin1, direction);
    digitalWrite(m_mtrPin2, !direction);

    /* Takes 1.3 seconds to RPM once going DOWN */
    /* Because of torque, it takes 1.444r seconds to RPM going UP */

    int32_t top = m_motorTopPosition*ENCODER_MULTIPLIER;
    int32_t tmp = m_motorPosition;
    /* Are we saving the position of the motor ? */
    if(m_motorPositionSaved)
    {
        if(direction)   // Opening door
        {
            while( m_encoder.read() < top )
            {
                //moveMotor(MOTOR_STEP_DELAY);
                debugln(m_encoder.read());
                m_motorPosition = m_encoder.read()/ENCODER_MULTIPLIER;  
                // if(tmp == m_motorPosition)
                // {
                //     debugln("Motor failed to move, encoder is not responding.");
                //     break;
                // }           
            }
            m_motorPosition = m_motorTopPosition;
        }
        else            // Closing door
        {

            while( m_encoder.read() > 0 )
            {
                //moveMotor(MOTOR_STEP_DELAY);
                debugln(m_encoder.read());
                m_motorPosition = m_encoder.read()/ENCODER_MULTIPLIER;  
            }

            m_motorPosition = 0;
        }

    }
    /* We're not saving the motor position to EEPROM, upon power
        failure, if the door/motor did not finish moving, we must
        assume a failure. */
    else
    {
        tmp = m_encoder.read();
        debugln("moveDoor() Moving motor without EEPROM Seconds");
        while(direction && m_encoder.read() < top)
        {
            if(tmp == m_encoder.read())
            {
                debugln("moveDoor() Motor failed to move, encoder is not responding.");
                break;
            }
        }
        while(!direction && m_encoder.read() > 0)
        {
            if(tmp == m_encoder.read())
            {
                debugln("moveDoor() Motor failed to move, encoder is not responding.");
                break;
            }
        }
        m_closed = !m_closed;
    }

    /* Depower Motor*/
    digitalWrite(m_mtrPin1, LOW);
    digitalWrite(m_mtrPin2, LOW);
    delay(2500);
    debugln("moveDoor() Finished Moving Motor");
    saveSettings();

    return true;
}

void DoorHandler::factoryReset()
{
    saveSetting(RESET);
}

DoorHandler::Response DoorHandler::getState()
{    
    return Response(
        m_id,
        getDoorState(),
        m_motorPosition,
        m_motorTopPosition,
        m_lightUpperThreshold,
        m_lightLowerThreshold,
        getLight(),
        m_automationEnabled,
        m_ldrEnabled,
        m_timeEnabled,
        m_motorPositionSaved,
        m_motorMoveTime,
        m_minuteToClose,
        m_minuteToOpen,
        m_minuteOffset
    );
}

bool DoorHandler::poll()
{
    //printLocalTime();

    // debug("Date: ");
    // debug(getTimeValue(TDAY));
    // debug("/");
    // debug(getTimeValue(MONTH));
    // debug("/");
    // debug(getTimeValue(YEAR));
    // debug(" ");    
    // debug(getTimeValue(HOUR));
    // debug(":");
    // debug(getTimeValue(MINUTE));
    // debug(":");
    // debug(getTimeValue(SECOND));

    if(getDoorState() == 2)
    {
        debugln("poll() Door was 'stuck', forcefully closed it");
        moveDoor(false);
        return true;
    }

    calculateTimeToMove();

    /* If automation is disabled OR neither sensor/time is enabled */
    if( ! isAutomated() || ( !m_ldrEnabled && !m_timeEnabled ))
    {
        debug( ! isAutomated() ? "poll() Automation is disabled " : "poll()  LDR/TIME are both disabled. ");
        if(!m_ldrEnabled && !m_timeEnabled)
        {
            debug("LDR: ");
            debug(m_ldrEnabled);
            debug(" TIME: ");
            debug(m_timeEnabled);
        }
        debugln("");
        return false;
    }

    debug("poll() Door is ");
    debug(isClosed() ? "Closed" : "Open");
    debug(" -> ");

    if(isOpen())
    {
        bool darkOutside = !m_ldrEnabled || getLight() <= m_lightLowerThreshold; 
        bool bedtime = !m_timeEnabled || checkTime(NIGHT);

        debug("Light: ");
        debug(darkOutside);
        debug(" - Time: ");
        debugln(bedtime);

        if(darkOutside && bedtime)
        {
            return moveDoor(CLOSE_DOOR);
        }
    }
    else if(isClosed())
    {
        bool lightOutside = !m_ldrEnabled || getLight() >= m_lightUpperThreshold;
        bool wakeup = !m_timeEnabled || checkTime(DAY);

        debug("Light: ");
        debug(lightOutside);
        debug(" - Time: ");
        debugln(wakeup);

        if(lightOutside && wakeup)
        {
            return moveDoor(OPEN_DOOR);
        }

    }
    return false;
}

/* -------------------------------------------------------------------*/
/* ------------------------ PRIVATE FUNCTIONS ------------------------*/
/* -------------------------------------------------------------------*/

bool DoorHandler::isClosed()
{
    if(m_motorPositionSaved)
    {
        return m_motorPosition == 0;
    }
    else
    {
        return m_closed;
    }
}

bool DoorHandler::isOpen()
{
    if(m_motorPositionSaved)
    {
        return m_motorPosition == m_motorTopPosition;
    }
    else
    {
        return !m_closed;
    }
}

bool DoorHandler::isMoving()
{
    if(m_motorPositionSaved)
    {
        return !isClosed() && !isOpen();
    }
    else
    {   // Without position saving, we don't know.
        return false;
    }
}

uint8_t DoorHandler::getDoorState()
{
    if( isClosed() ) return 0;
    if( isOpen()   ) return 1;
    if( isMoving() ) return 2;
                     return 3; // 'Default' unknown case
}

#define ANALOG_DIVISOR 3 // performs average
uint8_t DoorHandler::getLight()
{
    /* Take 100 samples over a second and return the average */
    uint32_t average = 0;
    for(int i = 0; i < ANALOG_DIVISOR; i++)
    {
        average += analogRead(m_ldrPin);
        delay(10);
    }
    return (average / ANALOG_DIVISOR) / 16;
}

bool DoorHandler::checkTime(bool dayOrNight)
{

    // dayOrNight == DAY == true
    // dayOrNight == NIGHT == false

    uint8_t currentHour     = getTimeValue(HOUR);
    uint8_t attempts        = 0;

    /* If time cannot be retrived, attempt for one minute
        if a failure continues, then return true if LDR enabled */
    while(currentHour == 255)
    {
        if(attempts >= 6) return m_ldrEnabled;

        attempts++;

        currentHour = getTimeValue(HOUR);
        delay(SECOND*10);
    }

    /* Work out how many minutes have passed since midnight */
    uint16_t currentMinute   = getTimeValue(MINUTE) + (getTimeValue(HOUR) * 60);

    if(dayOrNight == DAY)
    {
        /* e.g is 11AM greater than 7AM and less than 5PM */
        /* sunrisemins <= time <= sunsetmins */
        return m_minuteToOpen <= currentMinute && currentMinute <= m_minuteToClose;
    }
    else if(dayOrNight == NIGHT)
    {
        /* e.g is 2AM less than 7AM, 6PM is greater than 5PM */
        /* sunrisemins > time > sunsetmins */
        return m_minuteToOpen > currentMinute || currentMinute > m_minuteToClose;
    }
}

void DoorHandler::moveMotor(int motorDelay)
{
    delay(motorDelay);
}

int DoorHandler::getTimeValue(int choice)
{
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo))
    {
        // Return an 'error' number.
        return 255;
    }
    
    switch(choice)
    {
        case SECOND : return timeinfo.tm_sec;
        case MINUTE : return timeinfo.tm_min;
        case HOUR   : return timeinfo.tm_hour;
        case TDAY   : return timeinfo.tm_mday;
        case MONTH  : return timeinfo.tm_mon + 1; // Add one as jan = 0
        case YEAR   : return timeinfo.tm_year + 1900; // Years from 1900
        case DST    : return timeinfo.tm_isdst;
        default     : return 1;
    }
}

void DoorHandler::printLocalTime()
{
    #ifdef DEBUG
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
        Serial.println("Failed to obtain time");
        return;
    }
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
    #endif
}

void DoorHandler::calculateTimeToMove()
{

    int year  = getTimeValue(YEAR);
    int month = getTimeValue(MONTH);
    int day   = getTimeValue(TDAY);
    int dst   = getTimeValue(DST);

    /* Allows for variance in opening time to stop loud chickens */
    if(m_minuteOffset > 125)
    {
        /* Add (value * 2) minutes to opening time */
        m_minuteToOpen = movingTime.sunrise(year, month, day, dst) +(m_minuteOffset-100)*2;
    }
    else if(m_minuteOffset < 100)
    {
        /* Remove (value * 2) minutes to closing time */
        m_minuteToOpen = movingTime.sunrise(year, month, day, dst) -(m_minuteOffset*2);
    }
    else
    {
        /* Value remains the same */
        m_minuteToOpen = movingTime.sunrise(year, month, day, dst) ;   
    }

    // char buf[6];
    // movingTime.min2str(buf, m_minuteToOpen);
    // debug("Sunrise: (");
    // debug(m_minuteToOpen);
    // debug(") ");
    // debugln(buf);

    /* Sunset, add 30 as this is the offset to allow later sleepers */
    m_minuteToClose = movingTime.sunset(year, month, day, dst);
    // movingTime.min2str(buf, m_minuteToClose);
    // debug("Sunset: (");
    // debug(m_minuteToClose);
    // debug(") ");
    // debugln(buf);

    // debugln(buf);
}

void DoorHandler::saveSettings()
{
    debugln("saveSettings() Saving all settings.");
    EEPROM.write(0, m_id);
    EEPROM.write(1, m_motorPosition);
    EEPROM.write(2, m_motorTopPosition);
    EEPROM.write(3, m_lightUpperThreshold);
    EEPROM.write(4, m_lightLowerThreshold);
    EEPROM.write(5, m_minuteOffset);
    EEPROM.write(6, m_automationEnabled);
    EEPROM.write(7, m_ldrEnabled);
    EEPROM.write(8, m_motorPositionSaved);
    EEPROM.write(9, m_motorMoveTime);
    EEPROM.write(10, m_timeEnabled);
    EEPROM.commit();
}

void DoorHandler::saveSetting(int choice)
{
    debug("Saving setting: ");
    debugln(choice);
    switch(choice)
    {
        case DOOR_ID:
            EEPROM.write(0, m_id);
            break;
        case MTR_POSITION:
            EEPROM.write(1, m_motorPosition);
            break;
        case MTR_STOP_TOP:
            EEPROM.write(2, m_motorTopPosition);
            break;
        case LIGHT_THRESHOLD_TOP:
            EEPROM.write(3, m_lightUpperThreshold);
            break;
        case LIGHT_THRESHOLD_BOTTOM:
            EEPROM.write(4, m_lightLowerThreshold);
            break;
        case OPEN_OFFSET:
            EEPROM.write(5, m_minuteOffset);
            break;
        case AUTOMATION_ENABLE:
            EEPROM.write(6, m_automationEnabled);
            break;
        case LDR_ENABLE:
            EEPROM.write(7, m_ldrEnabled);
            break;
        case MOTOR_SAVED_ENABLE:
            EEPROM.write(8, m_motorPositionSaved);
            break;
        case MOTOR_MOVE_TIME:
            EEPROM.write(9, m_motorMoveTime);
            break;
        case TIME_ENABLE:
            EEPROM.write(10, m_timeEnabled);
            break;
        case RESET:
            /* Reset door to closed position ( 0 ), then reset EEPROM. */
            //if(m_motorPositionSaved)
            debugln("saveSetting() Resetting EEPROM.");
            flash();
            break;
        default:
            return;
    }
    EEPROM.commit();
}

void DoorHandler::loadSettings()
{

    m_id                  = EEPROM.read(0);

    /* if true, this device has not been flashed before */
    if(m_id == 255)
    {
        flash();
        return;
    }

    m_motorPosition       = EEPROM.read(1);
    m_motorTopPosition    = EEPROM.read(2);
    m_lightUpperThreshold = EEPROM.read(3);
    m_lightLowerThreshold = EEPROM.read(4);
    m_minuteOffset        = EEPROM.read(5);
    m_automationEnabled   = EEPROM.read(6);
    m_ldrEnabled          = EEPROM.read(7);
    m_motorPositionSaved  = EEPROM.read(8);
    m_motorMoveTime       = EEPROM.read(9);
    m_timeEnabled         = EEPROM.read(10);

    m_encoder.write(m_motorPosition*ENCODER_MULTIPLIER);
}

/* No true rng, too heavy - utilising psuedo */
uint8_t DoorHandler::generateUniqueID()
{
    /* 255 is reserved for a non-flashed device */
    // Number between 1-254
    return random(1, 254);
}

void DoorHandler::seedRandomNumberGenerator()
{
    /* Not 'proper' random, however
    should be sufficient for different boards */
    int seed = 0;
    for(int i = 0; i < 100; i++)
        seed += analogRead(0);
        
    randomSeed(seed);
}

void DoorHandler::flash()
{
    debugln("Flashing EEPROM.");
    EEPROM.write(0, generateUniqueID());
    EEPROM.write(1, 0); // Door position
    EEPROM.write(2, D_MTR_STOP_TOP);
    EEPROM.write(3, D_LIGHT_THRESHOLD_TOP);
    EEPROM.write(4, D_LIGHT_THRESHOLD_BOTTOM);
    EEPROM.write(5, D_OPEN_OFFSET);
    EEPROM.write(6, D_AUTOMATION_ENABLE);
    EEPROM.write(7, D_LDR_ENABLE);
    EEPROM.write(8, D_MOTOR_SAVED_ENABLE);
    EEPROM.write(9, D_MOTOR_MOVE_TIME);
    EEPROM.write(10, D_TIME_ENABLE);

    m_motorPosition       = 0;
    m_motorTopPosition    = D_MTR_STOP_TOP;
    m_lightUpperThreshold = D_LIGHT_THRESHOLD_TOP;
    m_lightLowerThreshold = D_LIGHT_THRESHOLD_BOTTOM;
    m_minuteOffset        = D_OPEN_OFFSET;
    m_automationEnabled   = D_AUTOMATION_ENABLE;
    m_ldrEnabled          = D_LDR_ENABLE;
    m_motorPositionSaved  = D_MOTOR_SAVED_ENABLE;
    m_motorMoveTime       = D_MOTOR_MOVE_TIME;
    m_timeEnabled         = D_TIME_ENABLE;

    EEPROM.commit();
    
}

