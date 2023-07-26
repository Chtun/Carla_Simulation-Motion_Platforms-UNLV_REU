/*=============================================================================
                  BlueTiger Application Programming Interface
                Copyright (C) 2008 MMI Development Group, Inc.
         Programming by MMI Development Group, Inc. - info@mmidev.com
===============================================================================

===============================================================================
                                 BTTelem.h                                   
===============================================================================
   DESCRIPTION: Interface to functions and data related to the BlueTiger
		Shared Memory Telemetry API.
                                                                             
   REVISIONS:   11/24/09 - SRJ - Genesis                                     
=============================================================================*/
#ifndef BTTELEM_H
#define BTTELEM_H

/*=============================================================================
    System Include Files                                                     
=============================================================================*/

/*=============================================================================
    Local Include Files                                                      
=============================================================================*/

/*=============================================================================
    Definitions and Constants                                                
=============================================================================*/

#define BTAPI_VERSION 101

#define BT_ERROR_NONE		    0
#define BT_ERROR_COMM_FAIL	    1
#define BT_ERROR_GET_IP_VALUE	    2
#define BT_ERROR_SET_IP_VALUE	    3
#define BT_ERROR_SET_MAC_VALUE	    4
#define BT_ERROR_SET_NETMASK_VALUE  5
#define BT_ERROR_SET_ROUTER_VALUE   6
#define BT_ERROR_CREATE_REG_KEY	    7
#define BT_ERROR_OPEN_REG_KEY	    8
#define BT_ERROR_PARAM_OUT_OF_RANGE 9
#define BT_ERROR_NOT_SUPPORTED	    10
#define BT_ERROR_NO_CONNECT	    11

/*=============================================================================
    Typedefs and Structures                                                  
=============================================================================*/

typedef unsigned char	tBTMAC[6];
typedef unsigned char	tBTIP[4];

typedef struct
{
    float   x;
    float   y;
    float   z;
} tBTVector3;

typedef struct
{
    unsigned short  duration;	    /* Length of movement in milliseconds */
    tBTVector3	    positionVector; /* Translational Offset */
    tBTVector3	    rotationVector; /* Rotation vector */
    float	    rotationAngle;  /* Rotation angle in radians */
} tBTEffect;

typedef struct
{
    char	    company[41];    /* Company name (40 char max) */
    char	    product[41];    /* Product name (40 char max) */
    char	    version[41];    /* Revision number */
    unsigned short  apiVersion;	    /* Supported API version */
    unsigned char   reserved[12];   /* Do not use */
} tBTDeviceInfo;

#ifdef __cplusplus
extern "C" {  // only need to export C interface if
              // used by C++ source code
#endif

/*=============================================================================
    Public Function Declarations                             
=============================================================================*/

unsigned short BTInit(char* company, char* product, char* version);
unsigned short BTPitchRollData(float pitch, float roll);
unsigned short BTRotationVectorData(float xRotation, float yRotation, float zRotation, float aRotation);
unsigned short BTAccelerationData(float xAccel, float yAccel, float zAccel, float xRotAccel, float yRotAccel, float zRotAccel,
				  float xForward, float yForward, float zForward, float xRight, float yRight, float zRight);
unsigned short BTDefineEffect(unsigned char effectIndex, tBTEffect* effectScript, unsigned char numEntries);
unsigned short BTPlayEffect(unsigned char effectIndex, unsigned char amplitude, unsigned char repetitions);
unsigned short BTStartEffect(unsigned char effectIndex, unsigned char amplitude);
unsigned short BTStopEffect(unsigned char effectIndex);
unsigned short BTAbsoluteMovementData(float xPosition, float yPosition, float zPosition,
				      float xRotation, float yRotation, float zRotation, float aRotation);
unsigned short BTRelativeMovementData(float xPosition, float yPosition, float zPosition,
				      float xRotation, float yRotation, float zRotation, float aRotation);
unsigned short BTPause(void);
unsigned short BTResume(void);
unsigned short BTShutdown(void);
unsigned short BTStatus(void);
unsigned short BTAPIVersion(void);

/*=============================================================================
    Deprecated Public Function Declarations                             
=============================================================================*/

unsigned short BTGetAddress(tBTMAC mac, tBTIP ip, tBTIP netmask, tBTIP router, BOOL* dynamic);
unsigned short BTSetAddress(tBTMAC mac, tBTIP ip, tBTIP netmask, tBTIP router, BOOL dynamic);
unsigned short BTGetMotionLimitRange(unsigned char* range);
unsigned short BTSetMotionLimitRange(unsigned char range);
unsigned short BTGetAccelerationPitch(signed short* accelerationPitch);
unsigned short BTSetAccelerationPitch(signed short accelerationPitch);
unsigned short BTGetAccelerationRoll(signed short* accelerationRoll);
unsigned short BTSetAccelerationRoll(signed short accelerationRoll);
unsigned short BTGetPositionPitch(signed short* positionPitch);
unsigned short BTSetPositionPitch(signed short positionPitch);
unsigned short BTGetPositionRoll(signed short* positionRoll);
unsigned short BTSetPositionRoll(signed short positionRoll);
unsigned short BTGetEffectScaling(signed short* effectScaling);
unsigned short BTSetEffectScaling(signed short effectScaling);
unsigned short BTStartDiagnostic(unsigned short diagnosticID);
unsigned short BTDiagnosticStatus(unsigned short* diagnosticID, unsigned short* diagnosticStatus);
unsigned short BTDeviceInformation(tBTDeviceInfo* deviceInfo);
unsigned short BTSetAccelerationHeave(signed short accelerationHeave);
unsigned short BTSetPositionHeave(signed short positionHeave);
unsigned short BTSetTipRounding(unsigned char tipRounding);

#ifdef __cplusplus
}
#endif

#endif /* BTAPI_H */