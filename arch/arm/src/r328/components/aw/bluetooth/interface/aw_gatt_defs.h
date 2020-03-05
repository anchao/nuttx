/*
 * Copyright (C) 2017 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the
 *       distribution.
 *    3. Neither the name of XRADIO TECHNOLOGY CO., LTD. nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __AW_GATT_DEFS_H__
#define __AW_GATT_DEFS_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t ble_gatts_err_t;

#define XR_GATT_ILLEGAL_UUID               0       // GATT INVALID UUID
#define XR_GATT_ILLEGAL_HANDLE             0       // GATT INVALID HANDLE
#define XR_GATT_ATTR_HANDLE_MAX            100     // GATT attribute max handle
#define XR_GATT_MAX_READ_MULTI_HANDLES     10      /* Max attributes to read in one request */

#define XR_GATT_UUID_IMMEDIATE_ALERT_SVC           0x1802          /*  Immediate alert Service*/
#define XR_GATT_UUID_LINK_LOSS_SVC                 0x1803          /*  Link Loss Service*/                             
#define XR_GATT_UUID_TX_POWER_SVC                  0x1804          /*  TX Power Service*/
#define XR_GATT_UUID_CURRENT_TIME_SVC              0x1805          /*  Current Time Service Service*/
#define XR_GATT_UUID_REF_TIME_UPDATE_SVC           0x1806          /*  Reference Time Update Service*/
#define XR_GATT_UUID_NEXT_DST_CHANGE_SVC           0x1807          /*  Next DST Change Service*/
#define XR_GATT_UUID_GLUCOSE_SVC                   0x1808          /*  Glucose Service*/
#define XR_GATT_UUID_HEALTH_THERMOM_SVC            0x1809          /*  Health Thermometer Service*/
#define XR_GATT_UUID_DEVICE_INFO_SVC               0x180A          /*  Device Information Service*/
#define XR_GATT_UUID_HEART_RATE_SVC                0x180D          /*  Heart Rate Service*/
#define XR_GATT_UUID_PHONE_ALERT_STATUS_SVC        0x180E          /* Phone Alert Status Service*/
#define XR_GATT_UUID_BATTERY_SERVICE_SVC           0x180F          /* Battery Service*/
#define XR_GATT_UUID_BLOOD_PRESSURE_SVC            0x1810          /* Blood Pressure Service*/
#define XR_GATT_UUID_ALERT_NTF_SVC                 0x1811          /* Alert Notification Service*/
#define XR_GATT_UUID_HID_SVC                       0x1812          /* HID Service*/
#define XR_GATT_UUID_SCAN_PARAMETERS_SVC           0x1813          /* Scan Parameters Service*/
#define XR_GATT_UUID_RUNNING_SPEED_CADENCE_SVC     0x1814          /* Running Speed and Cadence Service*/
#define XR_GATT_UUID_CYCLING_SPEED_CADENCE_SVC     0x1816          /* Cycling Speed and Cadence Service*/
#define XR_GATT_UUID_CYCLING_POWER_SVC             0x1818          /* Cycling Power Service*/
#define XR_GATT_UUID_LOCATION_AND_NAVIGATION_SVC   0x1819          /* Location and Navigation Service*/
#define XR_GATT_UUID_USER_DATA_SVC                 0x181C          /* User Data Service*/
#define XR_GATT_UUID_WEIGHT_SCALE_SVC              0x181D          /* Weight Scale Service*/

#define XR_GATT_UUID_PRI_SERVICE                   0x2800
#define XR_GATT_UUID_SEC_SERVICE                   0x2801
#define XR_GATT_UUID_INCLUDE_SERVICE               0x2802
#define XR_GATT_UUID_CHAR_DECLARE                  0x2803          /*  Characteristic Declaration*/

#define XR_GATT_UUID_CHAR_EXT_PROP                 0x2900          /*  Characteristic Extended Properties */
#define XR_GATT_UUID_CHAR_DESCRIPTION              0x2901          /*  Characteristic User Description*/
#define XR_GATT_UUID_CHAR_CLIENT_CONFIG            0x2902          /*  Client Characteristic Configuration */
#define XR_GATT_UUID_CHAR_SRVR_CONFIG              0x2903          /*  Server Characteristic Configuration */
#define XR_GATT_UUID_CHAR_PRESENT_FORMAT           0x2904          /*  Characteristic Presentation Format*/
#define XR_GATT_UUID_CHAR_AGG_FORMAT               0x2905          /*  Characteristic Aggregate Format*/
#define XR_GATT_UUID_CHAR_VALID_RANGE              0x2906          /*  Characteristic Valid Range */
#define XR_GATT_UUID_EXT_RPT_REF_DESCR             0x2907
#define XR_GATT_UUID_RPT_REF_DESCR                 0x2908

/* GAP Profile Attributes */
#define XR_GATT_UUID_GAP_DEVICE_NAME               0x2A00
#define XR_GATT_UUID_GAP_ICON                      0x2A01
#define XR_GATT_UUID_GAP_PREF_CONN_PARAM           0x2A04
#define XR_GATT_UUID_GAP_CENTRAL_ADDR_RESOL        0x2AA6

/* Attribute Profile Attribute UUID */
#define XR_GATT_UUID_GATT_SRV_CHGD                 0x2A05

/* Link XR_Loss Service */
#define XR_GATT_UUID_ALERT_LEVEL                   0x2A06          /* Alert Level */
#define XR_GATT_UUID_TX_POWER_LEVEL                0x2A07          /* TX power level */

/* Current Time Service */
#define XR_GATT_UUID_CURRENT_TIME                  0x2A2B          /* Current Time */
#define XR_GATT_UUID_LOCAL_TIME_INFO               0x2A0F          /* Local time info */
#define XR_GATT_UUID_REF_TIME_INFO                 0x2A14          /* reference time information */

/* Network availability Profile */
#define XR_GATT_UUID_NW_STATUS                     0x2A18          /* network availability status */
#define XR_GATT_UUID_NW_TRIGGER                    0x2A1A          /* Network availability trigger */

/* Phone alert */
#define XR_GATT_UUID_ALERT_STATUS                  0x2A3F          /* alert status */
#define XR_GATT_UUID_RINGER_CP                     0x2A40          /* ringer control point */
#define XR_GATT_UUID_RINGER_SETTING                0x2A41          /* ringer setting */

/* Glucose Service */
#define XR_GATT_UUID_GM_MEASUREMENT                0x2A18
#define XR_GATT_UUID_GM_CONTEXT                    0x2A34
#define XR_GATT_UUID_GM_CONTROL_POINT              0x2A52
#define XR_GATT_UUID_GM_FEATURE                    0x2A51

/* device information characteristic */
#define XR_GATT_UUID_SYSTEM_ID                     0x2A23
#define XR_GATT_UUID_MODEL_NUMBER_STR              0x2A24
#define XR_GATT_UUID_SERIAL_NUMBER_STR             0x2A25
#define XR_GATT_UUID_FW_VERSION_STR                0x2A26
#define XR_GATT_UUID_HW_VERSION_STR                0x2A27
#define XR_GATT_UUID_SW_VERSION_STR                0x2A28
#define XR_GATT_UUID_MANU_NAME                     0x2A29
#define XR_GATT_UUID_IEEE_DATA                     0x2A2A
#define XR_GATT_UUID_PNP_ID                        0x2A50

/* HID characteristics */
#define XR_GATT_UUID_HID_INFORMATION               0x2A4A
#define XR_GATT_UUID_HID_REPORT_MAP                0x2A4B
#define XR_GATT_UUID_HID_CONTROL_POINT             0x2A4C
#define XR_GATT_UUID_HID_REPORT                    0x2A4D
#define XR_GATT_UUID_HID_PROTO_MODE                0x2A4E
#define XR_GATT_UUID_HID_BT_KB_INPUT               0x2A22
#define XR_GATT_UUID_HID_BT_KB_OUTPUT              0x2A32
#define XR_GATT_UUID_HID_BT_MOUSE_INPUT            0x2A33

 /// Heart Rate Measurement
#define    XR_GATT_HEART_RATE_MEAS                 0x2A37
/// Body Sensor Location
#define    XR_GATT_BODY_SENSOR_LOCATION            0x2A38
/// Heart Rate Control Point
#define    XR_GATT_HEART_RATE_CNTL_POINT           0x2A39

/* Battery Service characteristics */
#define XR_GATT_UUID_BATTERY_LEVEL                 0x2A19

/* Sensor Service */
#define XR_GATT_UUID_SC_CONTROL_POINT              0x2A55
#define XR_GATT_UUID_SENSOR_LOCATION               0x2A5D

/* Runners speed and cadence service */
#define XR_GATT_UUID_RSC_MEASUREMENT               0x2A53
#define XR_GATT_UUID_RSC_FEATURE                   0x2A54

/* Cycling speed and cadence service */
#define XR_GATT_UUID_CSC_MEASUREMENT               0x2A5B
#define XR_GATT_UUID_CSC_FEATURE                   0x2A5C

/* Scan XR_Parameter characteristics */
#define XR_GATT_UUID_SCAN_INT_WINDOW               0x2A4F
#define XR_GATT_UUID_SCAN_REFRESH                  0x2A31

typedef enum {
   	XR_GATT_OK                     =   0x0,                    /* relate to BTA_GATT_OK in bta/bta_gatt_api.h */
    XR_GATT_INVALID_HANDLE         =   0x01,   /* 0x0001 */    /* relate to BTA_GATT_INVALID_HANDLE in bta/bta_gatt_api.h */
    XR_GATT_READ_NOT_PERMIT        =   0x02,   /* 0x0002 */    /* relate to BTA_GATT_READ_NOT_PERMIT in bta/bta_gatt_api.h */
    XR_GATT_WRITE_NOT_PERMIT       =   0x03,   /* 0x0003 */    /* relate to BTA_GATT_WRITE_NOT_PERMIT in bta/bta_gatt_api.h */
    XR_GATT_INVALID_PDU            =   0x04,   /* 0x0004 */    /* relate to BTA_GATT_INVALID_PDU in bta/bta_gatt_api.h */
    XR_GATT_INSUF_AUTHENTICATION   =   0x05,   /* 0x0005 */    /* relate to BTA_GATT_INSUF_AUTHENTICATION in bta/bta_gatt_api.h */
    XR_GATT_REQ_NOT_SUPPORTED      =   0x06,   /* 0x0006 */    /* relate to BTA_GATT_REQ_NOT_SUPPORTED in bta/bta_gatt_api.h */
    XR_GATT_INVALID_OFFSET         =   0x07,   /* 0x0007 */    /* relate to BTA_GATT_INVALID_OFFSET in bta/bta_gatt_api.h */
    XR_GATT_INSUF_AUTHORIZATION    =   0x08,   /* 0x0008 */    /* relate to BTA_GATT_INSUF_AUTHORIZATION in bta/bta_gatt_api.h */
    XR_GATT_PREPARE_Q_FULL         =   0x09,   /* 0x0009 */    /* relate to BTA_GATT_PREPARE_Q_FULL in bta/bta_gatt_api.h */
    XR_GATT_NOT_FOUND              =   0x0a,   /* 0x000a */    /* relate to BTA_GATT_NOT_FOUND in bta/bta_gatt_api.h */
    XR_GATT_NOT_LONG               =   0x0b,   /* 0x000b */    /* relate to BTA_GATT_NOT_LONG in bta/bta_gatt_api.h */
    XR_GATT_INSUF_KEY_SIZE         =   0x0c,   /* 0x000c */    /* relate to BTA_GATT_INSUF_KEY_SIZE in bta/bta_gatt_api.h */
    XR_GATT_INVALID_ATTR_LEN       =   0x0d,   /* 0x000d */    /* relate to BTA_GATT_INVALID_ATTR_LEN in bta/bta_gatt_api.h */
    XR_GATT_ERR_UNLIKELY           =   0x0e,   /* 0x000e */    /* relate to BTA_GATT_ERR_UNLIKELY in bta/bta_gatt_api.h */
    XR_GATT_INSUF_ENCRYPTION       =   0x0f,   /* 0x000f */    /* relate to BTA_GATT_INSUF_ENCRYPTION in bta/bta_gatt_api.h */
    XR_GATT_UNSUPPORT_GRP_TYPE     =   0x10,   /* 0x0010 */    /* relate to BTA_GATT_UNSUPPORT_GRP_TYPE in bta/bta_gatt_api.h */
    XR_GATT_INSUF_RESOURCE         =   0x11,   /* 0x0011 */    /* relate to BTA_GATT_INSUF_RESOURCE in bta/bta_gatt_api.h */

    XR_GATT_NO_RESOURCES           =   0x80,   /* 0x80 */    /* relate to BTA_GATT_NO_RESOURCES in bta/bta_gatt_api.h */
    XR_GATT_INTERNAL_ERROR         =   0x81,   /* 0x81 */    /* relate to BTA_GATT_INTERNAL_ERROR in bta/bta_gatt_api.h */
    XR_GATT_WRONG_STATE            =   0x82,   /* 0x82 */    /* relate to BTA_GATT_WRONG_STATE in bta/bta_gatt_api.h */
    XR_GATT_DB_FULL                =   0x83,   /* 0x83 */    /* relate to BTA_GATT_DB_FULL in bta/bta_gatt_api.h */
    XR_GATT_BUSY                   =   0x84,   /* 0x84 */    /* relate to BTA_GATT_BUSY in bta/bta_gatt_api.h */
    XR_GATT_ERROR                  =   0x85,   /* 0x85 */    /* relate to BTA_GATT_ERROR in bta/bta_gatt_api.h */
    XR_GATT_CMD_STARTED            =   0x86,   /* 0x86 */    /* relate to BTA_GATT_CMD_STARTED in bta/bta_gatt_api.h */
    XR_GATT_ILLEGAL_PARAMETER      =   0x87,   /* 0x87 */    /* relate to BTA_GATT_ILLEGAL_PARAMETER in bta/bta_gatt_api.h */
    XR_GATT_PENDING                =   0x88,   /* 0x88 */    /* relate to BTA_GATT_PENDING in bta/bta_gatt_api.h */
    XR_GATT_AUTH_FAIL              =   0x89,   /* 0x89 */    /* relate to BTA_GATT_AUTH_FAIL in bta/bta_gatt_api.h */
    XR_GATT_MORE                   =   0x8a,   /* 0x8a */    /* relate to BTA_GATT_MORE in bta/bta_gatt_api.h */
    XR_GATT_INVALID_CFG            =   0x8b,   /* 0x8b */    /* relate to BTA_GATT_INVALID_CFG in bta/bta_gatt_api.h */
    XR_GATT_SERVICE_STARTED        =   0x8c,   /* 0x8c */    /* relate to BTA_GATT_SERVICE_STARTED in bta/bta_gatt_api.h */
    XR_GATT_ENCRYPED_MITM          =   XR_GATT_OK,    /* relate to BTA_GATT_ENCRYPED_MITM in bta/bta_gatt_api.h */
    XR_GATT_ENCRYPED_NO_MITM       =   0x8d,   /* 0x8d */    /* relate to BTA_GATT_ENCRYPED_NO_MITM in bta/bta_gatt_api.h */
    XR_GATT_NOT_ENCRYPTED          =   0x8e,   /* 0x8e */    /* relate to BTA_GATT_NOT_ENCRYPTED in bta/bta_gatt_api.h */
    XR_GATT_CONGESTED              =   0x8f,   /* 0x8f */    /* relate to BTA_GATT_CONGESTED in bta/bta_gatt_api.h */
    XR_GATT_DUP_REG                =   0x90,   /* 0x90 */    /* relate to BTA_GATT_DUP_REG in bta/bta_gatt_api.h */
    XR_GATT_ALREADY_OPEN           =   0x91,   /* 0x91 */    /* relate to BTA_GATT_ALREADY_OPEN in bta/bta_gatt_api.h */
    XR_GATT_CANCEL                 =   0x92,   /* 0x92 */    /* relate to BTA_GATT_CANCEL in bta/bta_gatt_api.h */
    /* 0xE0 ~ 0xFC reserved for future use */
    XR_GATT_STACK_RSP              =   0xe0,   /* 0xe0 */    /* relate to BTA_GATT_STACK_RSP in bta/bta_gatt_api.h */
    XR_GATT_APP_RSP                =   0xe1,   /* 0xe1 */    /* relate to BTA_GATT_APP_RSP in bta/bta_gatt_api.h */
    //Error caused by customer application or stack bug
    XR_GATT_UNKNOWN_ERROR          =   0xef,   /* 0xef */    /* relate to BTA_GATT_UNKNOWN_ERROR in bta/bta_gatt_api.h */
    XR_GATT_CCC_CFG_ERR            =   0xfd,   /* 0xFD Client Characteristic Configuration Descriptor Improperly Configured */    /* relate to BTA_GATT_CCC_CFG_ERR in bta/bta_gatt_api.h */
    XR_GATT_PRC_IN_PROGRESS        =   0xfe,   /* 0xFE Procedure Already in progress */  /* relate to BTA_GATT_PRC_IN_PROGRESS in bta/bta_gatt_api.h */
    XR_GATT_OUT_OF_RANGE           =   0xff,   /* 0xFFAttribute value out of range */    /* relate to BTA_GATT_OUT_OF_RANGE in bta/bta_gatt_api.h */
} t_gatt_status;

/* relate to BTA_GATT_CONN_xxx in bta/bta_gatt_api.h */
/**
 * @brief Gatt Connection reason enum
 */
typedef enum {
    XR_GATT_CONN_UNKNOWN = 0,                      /*!< Gatt connection unknown */               /* relate to BTA_GATT_CONN_UNKNOWN in bta/bta_gatt_api.h */
    XR_GATT_CONN_L2C_FAILURE = 1,                  /*!< General L2cap failure  */                /* relate to BTA_GATT_CONN_L2C_FAILURE in bta/bta_gatt_api.h */
    XR_GATT_CONN_TIMEOUT = 0x08,                   /*!< Connection timeout  */                   /* relate to BTA_GATT_CONN_TIMEOUT in bta/bta_gatt_api.h */
    XR_GATT_CONN_TERMINATE_PEER_USER = 0x13,       /*!< Connection terminate by peer user  */    /* relate to BTA_GATT_CONN_TERMINATE_PEER_USER in bta/bta_gatt_api.h */
    XR_GATT_CONN_TERMINATE_LOCAL_HOST = 0x16,      /*!< Connection terminated by local host */    /* relate to BTA_GATT_CONN_TERMINATE_LOCAL_HOST in bta/bta_gatt_api.h */
    XR_GATT_CONN_FAIL_ESTABLISH = 0x3e,            /*!< Connection fail to establish  */         /* relate to BTA_GATT_CONN_FAIL_ESTABLISH in bta/bta_gatt_api.h */
    XR_GATT_CONN_LMP_TIMEOUT = 0x22,               /*!< Connection fail for LMP response tout */ /* relate to BTA_GATT_CONN_LMP_TIMEOUT in bta/bta_gatt_api.h */
    XR_GATT_CONN_CONN_CANCEL = 0x0100,             /*!< L2CAP connection cancelled  */           /* relate to BTA_GATT_CONN_CONN_CANCEL in bta/bta_gatt_api.h */
    XR_GATT_CONN_NONE = 0x0101                     /*!< No connection to cancel  */              /* relate to BTA_GATT_CONN_NONE in bta/bta_gatt_api.h */
} t_gatt_conn_reason;

/* relate to BTA_GATT_AUTH_REQ_xxx in bta/bta_gatt_api.h */
/**
 * @brief Gatt authentication request type
 */
typedef enum {
    XR_GATT_AUTH_REQ_NONE                  = 0,                                       /* relate to BTA_GATT_AUTH_REQ_NONE in bta/bta_gatt_api.h */
    XR_GATT_AUTH_REQ_NO_MITM               = 1,   /* unauthenticated encryption */    /* relate to BTA_GATT_AUTH_REQ_NO_MITM in bta/bta_gatt_api.h */
    XR_GATT_AUTH_REQ_MITM                  = 2,   /* authenticated encryption */      /* relate to BTA_GATT_AUTH_REQ_MITM in bta/bta_gatt_api.h */
    XR_GATT_AUTH_REQ_SIGNED_NO_MITM        = 3,                                       /* relate to BTA_GATT_AUTH_REQ_SIGNED_NO_MITM in bta/bta_gatt_api.h */
    XR_GATT_AUTH_REQ_SIGNED_MITM           = 4,                                       /* relate to BTA_GATT_AUTH_REQ_SIGNED_MITM in bta/bta_gatt_api.h */
} t_gatt_auth_req;

#define    XR_GATT_PERM_READ                  (1 << 0)   /* bit 0 -  0x0001 */    /* relate to BTA_GATT_PERM_READ in bta/bta_gatt_api.h */
#define    XR_GATT_PERM_READ_ENCRYPTED        (1 << 1)   /* bit 1 -  0x0002 */    /* relate to BTA_GATT_PERM_READ_ENCRYPTED in bta/bta_gatt_api.h */
#define    XR_GATT_PERM_READ_ENC_MITM         (1 << 2)   /* bit 2 -  0x0004 */    /* relate to BTA_GATT_PERM_READ_ENC_MITM in bta/bta_gatt_api.h */
#define    XR_GATT_PERM_WRITE                 (1 << 4)   /* bit 4 -  0x0010 */    /* relate to BTA_GATT_PERM_WRITE in bta/bta_gatt_api.h */
#define    XR_GATT_PERM_WRITE_ENCRYPTED       (1 << 5)   /* bit 5 -  0x0020 */    /* relate to BTA_GATT_PERM_WRITE_ENCRYPTED in bta/bta_gatt_api.h */
#define    XR_GATT_PERM_WRITE_ENC_MITM        (1 << 6)   /* bit 6 -  0x0040 */    /* relate to BTA_GATT_PERM_WRITE_ENC_MITM in bta/bta_gatt_api.h */
#define    XR_GATT_PERM_WRITE_SIGNED          (1 << 7)   /* bit 7 -  0x0080 */    /* relate to BTA_GATT_PERM_WRITE_SIGNED in bta/bta_gatt_api.h */
#define    XR_GATT_PERM_WRITE_SIGNED_MITM     (1 << 8)   /* bit 8 -  0x0100 */    /* relate to BTA_GATT_PERM_WRITE_SIGNED_MITM in bta/bta_gatt_api.h */
/* relate to BTA_GATT_CHAR_PROP_BIT_xxx in bta/bta_gatt_api.h */
/* definition of characteristic properties */
#define    XR_GATT_CHAR_PROP_BIT_BROADCAST    (1 << 0)       /* 0x01 */    /* relate to BTA_GATT_CHAR_PROP_BIT_BROADCAST in bta/bta_gatt_api.h */
#define    XR_GATT_CHAR_PROP_BIT_READ         (1 << 1)       /* 0x02 */    /* relate to BTA_GATT_CHAR_PROP_BIT_READ in bta/bta_gatt_api.h */
#define    XR_GATT_CHAR_PROP_BIT_WRITE_NR     (1 << 2)       /* 0x04 */    /* relate to BTA_GATT_CHAR_PROP_BIT_WRITE_NR in bta/bta_gatt_api.h */
#define    XR_GATT_CHAR_PROP_BIT_WRITE        (1 << 3)       /* 0x08 */    /* relate to BTA_GATT_CHAR_PROP_BIT_WRITE in bta/bta_gatt_api.h */
#define    XR_GATT_CHAR_PROP_BIT_NOTIFY       (1 << 4)       /* 0x10 */    /* relate to BTA_GATT_CHAR_PROP_BIT_NOTIFY in bta/bta_gatt_api.h */
#define    XR_GATT_CHAR_PROP_BIT_INDICATE     (1 << 5)       /* 0x20 */    /* relate to BTA_GATT_CHAR_PROP_BIT_INDICATE in bta/bta_gatt_api.h */
#define    XR_GATT_CHAR_PROP_BIT_AUTH         (1 << 6)       /* 0x40 */    /* relate to BTA_GATT_CHAR_PROP_BIT_AUTH in bta/bta_gatt_api.h */
#define    XR_GATT_CHAR_PROP_BIT_EXT_PROP     (1 << 7)       /* 0x80 */    /* relate to BTA_GATT_CHAR_PROP_BIT_EXT_PROP in bta/bta_gatt_api.h */

/// GATT maximum attribute length
#define XR_GATT_MAX_ATTR_LEN   600 //as same as GATT_MAX_ATTR_LEN

typedef struct
{
    uint16_t attr_max_len;                                  /*!<  attribute max value length */
    uint16_t attr_len;                                      /*!<  attribute current value length */
    uint8_t  *attr_value;                                   /*!<  the pointer to attribute value */
} t_attr_value;

typedef struct
{
#define XR_GATT_RSP_BY_APP             0
#define XR_GATT_AUTO_RSP               1
    /**
     * @brief if auto_rsp set to XR_GATT_RSP_BY_APP, means the response of Write/Read operation will by replied by application.
              if auto_rsp set to XR_GATT_AUTO_RSP, means the response of Write/Read operation will be replied by GATT stack automatically.
     */
    uint8_t auto_rsp;
} t_attr_control;

/// Gatt attribute value 
typedef struct {
    uint8_t           value[XR_GATT_MAX_ATTR_LEN];         /*!< Gatt attribute value */
    uint16_t          handle;                               /*!< Gatt attribute handle */
    uint16_t          offset;                               /*!< Gatt attribute value offset */
    uint16_t          len;                                  /*!< Gatt attribute value length */
    uint8_t           auth_req;                             /*!< Gatt authentication request */
} t_gatt_value;


/// GATT remote read request response type
typedef union {
    t_gatt_value attr_value;                            /*!< Gatt attribute structure */
    uint16_t            handle;                             /*!< Gatt attribute handle */
} t_gatt_rsp;

/**
 * @brief Gatt service id, include id
 *        (uuid and instance id) and primary flag
 */
typedef struct {
    t_bt_uuid		uuid;
    uint8_t			inst_id;
    bool            is_primary;             /*!< This service is primary or not */
} __attribute__((packed)) t_gatt_srvc_id;

typedef struct {
	uint16_t service_handle;
	t_gatt_srvc_id service_id;
} t_gatt_service_pty;

typedef struct {
	uint16_t char_handle;
	t_bt_uuid char_uuid;
}t_gatt_char_pty;

typedef struct {
	uint16_t descr_handle;
	t_bt_uuid desrc_uuid;
}t_gatt_char_descr_pty;

#ifdef __cplusplus
}
#endif

#endif /* __XR_GATT_DEFS_H__ */
