
#ifndef DOOR_HANDLER
#define DOOR_HANDLER 1

#include <stdint.h> // Precise type allocation
#include <stdio.h> // Sprintf

#include <Encoder.h> // For motor

#define RESPONSE_LENGTH 250

/* Singleton wrapper */
class DoorHandler{

    public:    // Inner class declaration

        /* Wrapper for Char* UDP packet */
        class Response{

            public:
                Response(uint8_t id,
                        uint8_t state,
                        uint8_t mtrPos,
                        uint8_t mtrTopPos,
                        uint8_t mtrUpperLight,
                        uint8_t mtrLowerLight,
                        uint8_t currentLight,
                        uint8_t automated,
                        uint8_t ldrEnabled,
                        uint8_t timeEnabled,
                        uint8_t motorSaved,
                        uint8_t motorTime,
                        uint16_t closingTimeMinute,
                        uint16_t openingTimeMinute,
                        uint8_t minuteOffset)
                {

                    m_length = sprintf(m_buffer, "!ID=%d,STATE=%d,MTR_POS=%d,TOPPOS=%d,UL=%d,LL=%d,LIT=%d,AUTO=%d,LDR=%d,TIME=%d,MTRSAVE=%d,MTRTIME=%d,CLOSE=%d,OPEN=%d,MOFF=%d",
                     id, state, mtrPos, mtrTopPos, mtrUpperLight, mtrLowerLight,
                     currentLight, automated, ldrEnabled, timeEnabled, motorSaved, motorTime,
                     closingTimeMinute, openingTimeMinute, minuteOffset*2);

                }
                char* getResponse(){return m_buffer;}
                uint8_t getLength(){return m_length;}

            private:
                char m_buffer[RESPONSE_LENGTH];
                uint8_t m_length;

        };

        /* Main class declarations, members and functions */
        DoorHandler(uint8_t mtrPin1, uint8_t mtrPin2, uint8_t encoderPin1, uint8_t encoderPin2, uint8_t ldrPin);

        /* Getters */
        uint8_t getPosition()           {return m_motorPosition;}
        uint8_t getTopPosition()        {return m_motorTopPosition;}
        uint8_t getLightUpperThreshold(){return m_lightUpperThreshold;}
        uint8_t getLightLowerThreshold(){return m_lightLowerThreshold;}
        uint8_t getOpenTime()           {return m_minuteOffset;}
	    uint8_t getLightLevel()		    {return getLight();}
        uint8_t getID()                 {return m_id;}

        /* Queries */
        bool isAutomated  (){return m_automationEnabled;}
        bool isMotorSaved (){return m_motorPositionSaved;}
        bool isClosed     ();
        bool isOpen       ();
        bool isMoving     ();
        bool ldrEnabled   (){return m_ldrEnabled;}
        bool timeEnabled(){return m_timeEnabled;}

        /* Setters */
        void setLDREnabled (bool flag);
        void setTimeEnabled(bool flag);
        void setAutomated (bool flag){m_automationEnabled=flag;}
        void setMotorSaved(bool flag);
        void setDoorOpenTime(uint8_t time);
        void forcedOpen();
        void forcedClosed();

        uint8_t setLightUpperThreshold(uint8_t value);
        uint8_t setLightLowerThreshold(uint8_t value);
        uint8_t setTopPosition(uint8_t value);
        uint8_t setDoorId(uint8_t value);
        uint8_t setMotorMoveSpeed(uint8_t value);

        /* General functions */
        void     loadSettings();
        void     configureNTP();
        void     printLocalTime();
        Response getState();
        bool     moveDoor(bool direction);
        bool     poll();
        void     factoryReset();

        void     saveSettings();
        void     saveSetting(int choice);

    private:
        /* Used for networking */
        uint8_t m_id;

        /* Pins used on board */
        uint8_t m_mtrPin1;
        uint8_t m_mtrPin2;
        Encoder m_encoder;
        uint8_t m_ldrPin; // Must be analog

        /* EEPROM Values */
        uint8_t m_motorPosition;
        uint8_t m_motorTopPosition;
        uint8_t m_lightUpperThreshold;
        uint8_t m_lightLowerThreshold;
        uint8_t m_minuteOffset;         // The offset value from opening, default is 100 (1 step equals 2 minutes, i.e 10 = -3hours from opening)
        bool    m_automationEnabled;
        bool    m_motorPositionSaved;

        /* Time values for open/close */
        uint16_t m_minuteToOpen;
        uint16_t m_minuteToClose;

        /* At least one of these MUST be true */
        bool    m_ldrEnabled;
        bool    m_timeEnabled;

        /* If m_motorPositionSaved = false, these members become used */
        uint8_t m_motorMoveTime;
        bool    m_closed;
        bool    m_eepromNeedsSaving;

        /* Private functions */
        uint8_t  getDoorState();
        uint8_t  getClosingTime();
        uint8_t  getOpeningTime();
        uint8_t  getLight();
        bool     checkTime(bool dayOrNight);
        void     moveMotor(int delay);
        int      getTimeValue(int choice);
        void     calculateTimeToMove();
        uint8_t  generateUniqueID();
        void     seedRandomNumberGenerator();
        void     flash();
        

};

#endif

