#pragma once

/* UbloxM8Q_GPS library by Lynd Wieman
 *   For use with the Particle Electron AssetTracker 2 hardware and 
 *   AssetTracker2 library.
 *   See README.md
 */

#include "Particle.h"

// return value from driver routines
enum ubxReturn {
    ubxSUCCESS = 0,
    ubxPENDING,
    ubxBAD_LENGTH,
    ubxBAD_CHECK_SUM,
    ubxNO_ACK
};

// UBX message indexes and defines
#define CLASS_IDX 2
#define ID_IDX 3
#define LENGTH_IDX 4
#define PAYLOAD_IDX 6

// Some #defines to make it easier to identify the types of messages
// Not all defined types are supported.

// ubx classes           types                          pg# in ublox spec
#define UBX_ACK 0x05
#define                  ACK_TYPE ((UBX_ACK<<8)|(0x01))       // 142
#define                  NAK_TYPE ((UBX_ACK<<8)|(0x00))       // 142
#define UBX_AID 0x0B
#define UBX_CFG 0x06
#define                  CFG_ANT_TYPE ((UBX_CFG<<8)|(0x13))   // 153
#define                  CFG_CFG_TYPE ((UBX_CFG<<8)|(0x09))   // 154
#define                  CFG_INF_TYPE ((UBX_CFG<<8)|(0x02))   // 166
#define                  CFG_MSG_TYPE ((UBX_CFG<<8)|(0x01))   // 171
#define                  CFG_NAV5_TYPE ((UBX_CFG<<8)|(0x24))  // 171
#define                  CFG_ODO_TYPE ((UBX_CFG<<8)|(0x1E))   // 186
#define                  CFG_PM2_TYPE ((UBX_CFG<<8)|(0x3B))   // 188
#define                  CFG_PMS_TYPE ((UBX_CFG<<8)|(0x86))   // 192
#define                  CFG_PRT_TYPE ((UBX_CFG<<8)|(0x00))   // 193
#define                  CFG_PWR_TYPE ((UBX_CFG<<8)|(0x57))   // 204
#define                  CFG_RATE_TYPE ((UBX_CFG<<8)|(0x08))  // 204
#define UBX_ESF 0x10
#define UBX_HNR 0x28
#define                  HNR_PVT_TYPE ((UBX_HNR<<8)|(0x00))   // 228
#define UBX_INF 0x04
#define UBX_LOG 0x21
#define UBX_MGA 0x13
#define UBX_MON 0x0A
#define UBX_NAV 0x01
#define                  NAV_ODO_TYPE ((UBX_NAV<<8)|(0x09))       // 290
#define                  NAV_RESETODO_TYPE ((UBX_NAV<<8)|(0x10))  // 299
#define                  NAV_POSLLH_TYPE ((UBX_NAV<<8)|(0x02))    // 293
#define                  NAV_PVT_TYPE ((UBX_NAV<<8)|(0x07))       // 293
#define UBX_RXM 0x02
#define UBX_SEC 0x27
#define UBX_TIM 0x0D
#define UBX_UPD 0x14

class UbloxM8Q_GPS
{
public:
  UbloxM8Q_GPS();

  void common_init(void);
  enum ubxReturn begin(void);
  void sendCommand(uint8_t *msg); // Won't return until ACK or error
  void read(); // update gps info

  // set the measurement rate and navigation solution rate. Also sets the
  // ubxWatchdog timeout
  // The receiver also allows setting update rates per message for
  // each output port. In this system I don't see a reason to do fewer 
  // sends then measurements. If the rate for all messages configured is 
  // greater than one, the ubxWatchdog timer may be too short and the system
  // will reboot before messages are generated.
  // 
  void setRate(uint16_t measurement, int navigationSol);
  // measurement - ms. Time between position measurements
  // navigationSol - cycles. Number of measuremnts for each navigation solution

  // reset the trip odometer to 0.
  void resetODO(void);

  // Set static hold threshold.
  // speed is one byte cm/s, distance is 2 bytes m
  void setThresh(int speed, uint16_t distance);

  bool initComplete;

  uint8_t hour, minute, seconds, month, day;
  uint16_t year;
  int32_t milliseconds, tow;
  bool dateValid;
  bool timeValid;
  // Floating point latitude and longitude value in degrees.
  float latitude, longitude;
  float latitudeDegrees, longitudeDegrees;
  float geoidheight, altitude;
  float speed; // m/s - its just gSpeed/1000.0
  float headingMot; // heading of motion, degrees 
  float headingVeh; // heading of vehicle, degrees 
  bool headVehValid; // heading of vehicle valid,
  bool fix;
  uint8_t fixquality, satellites;
  uint32_t horzAcc, vertAcc; // mm
  int32_t velN, velE, velD; // north, east, down velocity, mm/s
  int32_t gSpeed; // ground speed, mm/s
  int32_t odoTrip; // odometer since last time commanded reset, m
  int32_t odoTotal; // odometer since last power up, m
  int32_t odoAcc; // odometer accuracy, m (1-sigma)


private:

  // set when receiver enabled
  bool gpsOn;
  // length is 2 bytes, little endian
  int ubx_length(uint8_t *m) {
        return( ((m[LENGTH_IDX])+(m[LENGTH_IDX+1]*256)));}
  // type is the class and id combined
  int ubxMsgType(uint8_t *m) { return((m[CLASS_IDX]<<8)|(m[ID_IDX]));}
  // For extracting type from the payload of msg, parameter points into payload
  int ubxMsgParm(uint8_t *m) { return((m[0]<<8)|(m[1]));}
  enum ubxReturn writeUBX(uint8_t *msg); // write one msg to HW
  // read one message from HW
  enum ubxReturn readUBXmsg(uint8_t *msg, int *id, int *length);
  void parse(uint8_t *msg, int id, int length);
  void parsePOSLLH(uint8_t *msg);
  void parsePVT(uint8_t *msg);
  void clearGpsSerial(void); // flush receiver output
  // Calculate check sum
  void ubxCS(uint8_t *msg, uint8_t *ck_a, uint8_t *ck_b);
  // add the CS to a message
  void ubxAddCS(uint8_t *msg);
  void showUbx(uint8_t *msg);
  // routines to access various sizes and types of data from ubx messages
  int      getInt(uint8_t *m);
  uint16_t getUInt(uint8_t *m);
  uint32_t getULong(uint8_t *m);
  int32_t  getLong(uint8_t *m);
  float    getFloat(uint8_t *m);
  void readVer(void); // used only by begin
  void reportError(int state, enum ubxReturn ret); // used only by begin
  void delayButProcess(uint32_t time);
  void parseODO(uint8_t *msg);
};


// a class to support detecting ACK/NAK
#define ACKCNT 10 // 10 should be plenty
class AckedMsg
{
public:
    AckedMsg();
    int setAck(int);
    void clearAck(int);
    bool isAcked(int);
private:
    int ackList[ACKCNT];
};

