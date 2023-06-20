#define LCD_HRES 480
#define LCD_VRES 320
#define LVGL_LCD_BUF_SIZE (16*1024)
#include <Arduino.h>
#include <lcd_controller.h>
#include <lvgl.h>
#include <ui.h>

//================================================== MY STUFF =========================================
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "xbee/platform.h"
#include "xbee/atcmd.h"
#include "xbee/tx_status.h"
#include "xbee/user_data.h"

//#include "parse_serial_args.h"




//================================================== MY STUFF =========================================


static lv_disp_draw_buf_t disp_buf;  // contains internal graphic buffer(s) called draw buffer(s)
static lv_disp_drv_t disp_drv;       // contains callback functions
static lv_color_t *lv_disp_buf;
static lv_color_t *lv_disp_buf2;
static bool is_initialized_lvgl = false;

static bool lcd_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t* edata, void* user_ctx) {
    lv_disp_flush_ready(&disp_drv);
    return true;
}
static void lvgl_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    lcd_flush(offsetx1, offsety1, offsetx2 , offsety2 , color_map);
}

//================================================== DAVE'S STUFF =========================================*/
    static const char PING_REQUEST[] = "\x01PING";  // CSXB_MSG_PING_REQUEST=0x01 followed by arbitrary payload
    static const char MQTT_START_REQUEST[] = "\x16MQTT_START";  // CSXB_MSG_MQTT_START=0x16 followed by arbitrary payload
    char cmdstr[256];
    xbee_serial_t XBEE_SERPORT;
    int iface = XBEE_USER_DATA_IF_MICROPYTHON;
    int status;
//================================================== DAVE'S STUFF =========================================*/


//================================================== MY STUFF =========================================

bool USBSerialCommandReceived = false;
enum class InputType  {NONE,USBSERIALCOMMAND};
InputType ReceivedInput = InputType::NONE;

uint8_t SerialRetries = 0;
#define SERIALRETRY_MAX 3
int XBeeResponseTime_ms;
#define XBEE_RESPONSETIME_MAX 1000

String XBeeReceivedText;

enum class RXCode{OK = 0,ERR = -1};
RXCode CurrentResponse = RXCode::ERR;
String USBSerialRXstr;

String ReceiveString(String TXStr);
RXCode SendString(String TXstr);
bool WaitForOK();

uint32_t DebugCount;
String PrevDebugStr;
void DEBUG(String DebugStr);

xbee_dev_t my_xbee;

#include <limits.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "xbee/serial.h"

#define XBEE_SER_CHECK(ptr) \
    do { if (xbee_ser_invalid(ptr)) return -EINVAL; } while (0)


int xbee_ser_invalid( xbee_serial_t *serial)
{

   // DEBUG("'xbee_ser_invalid' called");
    if (serial)
    {
        return 0;
    }

    #define XBEE_SERIAL_VERBOSE
    #ifdef XBEE_SERIAL_VERBOSE
        
    #endif

    if (serial == NULL)
        {
            
            //printf( "%s: serial=NULL\n", __FUNCTION__);
        }
        else
        {
            //printf( "%s: serial=%p, serial->fd=%d (invalid)\n", __FUNCTION__,serial, serial->fd);
        }


    return 1;
}


const char *xbee_ser_portname( xbee_serial_t *serial)
{
    
    DEBUG("'xbee_ser_portname' called");

    if (serial == NULL)
    {
        return "(invalid)";
    }

    return serial->portname;
}


int xbee_ser_write( xbee_serial_t *serial, const void FAR *buffer,int length)
{
    //DEBUG("'xbee_ser_write' called");
    int result;
    const char* buf = (const char*)buffer;
    for(int i=0;i<length;++i ) {
        Serial.printf("%02x ",(int)buf[i]);
    }
    Serial.print("   ");
    for(int i=0;i<length;++i ) {
        if(isprint(buf[i])) {
            Serial.print(buf[i]);
        } else {
            Serial.print('.');
        }
    }
    if(length>0)
    {
        Serial.println();
    }
    /*if (SendString((const void)buffer) == RXCode::ERR)
    {
        DEBUG("Send String Failed");
    }
    else
    {
        DEBUG("Send String Success");
    }*/
    
    ((HardwareSerial*)serial->ser)->write((char*)buffer,length);
   // USBSerial.write((char*)buffer,length);


    return 0;
}


int xbee_ser_read( xbee_serial_t *serial, void FAR *buffer, int bufsize)
{
    for(int i = 0;i<1000;++i) {
        if(((HardwareSerial*)serial->ser)->available()>=bufsize) {
            break;
        }
        delay(1);
    }
    DEBUG(String("available count: ")+((HardwareSerial*)serial->ser)->available());
    int avail = min(bufsize,((HardwareSerial*)serial->ser)->available());
    int result = (int)((HardwareSerial*)serial->ser)->read((char*)buffer,avail);
    DEBUG("'xbee_ser_read' returned: " + String((char*)buffer,avail));
    return result;
}

int xbee_ser_putchar( xbee_serial_t *serial, uint8_t ch)
{
    if(0==((HardwareSerial*)serial->ser)->availableForWrite()) {
        return -ENOSPC;
    }
    DEBUG("'xbee_ser_putchar' called");
    int retval;

    retval = xbee_ser_write( serial, &ch, 1);
    if (retval == 1)
    {
        return 0;
    }
    else if (retval == 0)
    {
        return -ENOSPC;
    }
    else
    {
        return retval;
    }
}


int xbee_ser_getchar( xbee_serial_t *serial)
{
    DEBUG("'xbee_ser_getchar' called");
    int i;
    for(i = 0;i<1000;++i) {
        if(((HardwareSerial*)serial->ser)->available()) {
            break;
        }
        delay(1);
    }
    i= ((HardwareSerial*)serial->ser)->read();
    uint8_t ch = 0;
    if (0>i )
    {
        return -ENODATA;
    }

    return i;
}


int xbee_ser_tx_free( xbee_serial_t *serial)
{
    DEBUG("'xbee_ser_tx_free' called");
    return INT_MAX;
}


int xbee_ser_tx_used( xbee_serial_t *serial)
{
    DEBUG("'xbee_ser_tx_used' called");
    return 0;
}


int xbee_ser_tx_flush( xbee_serial_t *serial)
{
    ((HardwareSerial*)serial->ser)->flush();
    DEBUG("'xbee_ser_tx_flush' called");
    return 0;
}


int xbee_ser_rx_free( xbee_serial_t *serial)
{
    DEBUG("'xbee_ser_rx_free' called");
    return INT_MAX;
}


int xbee_ser_rx_used( xbee_serial_t *serial)
{
    DEBUG("'xbee_ser_rx_used' called");
   return ((HardwareSerial*)serial->ser)->available();
}


int xbee_ser_rx_flush( xbee_serial_t *serial)
{
    DEBUG("'xbee_ser_rx_flush' called");
    ((HardwareSerial*)serial->ser)->flush();
    return 0;
}


#define _BAUDCASE(b)        case b: baud = B ## b; break
int xbee_ser_baudrate( xbee_serial_t *serial, uint32_t baudrate)
{
    ((HardwareSerial*)serial->ser)->end();
    serial->baudrate = baudrate;
    ((HardwareSerial*)serial->ser)->begin(baudrate,SERIAL_8N1,serial->pin_rx,serial->pin_tx);
    DEBUG("'xbee_ser_baudrate' called");
    return 0;
}


int xbee_ser_open( xbee_serial_t *serial, uint32_t baudrate)
{
    if(baudrate!=0) {
        serial->baudrate = baudrate;
    }
    ((HardwareSerial*)serial->ser)->begin(serial->baudrate,SERIAL_8N1,serial->pin_rx,serial->pin_tx);
    //serial->ser.begin() baudrate,serial->pin_tx);
    DEBUG("'xbee_ser_open' called");
    return 0;
}


int xbee_ser_close( xbee_serial_t *serial)
{
    ((HardwareSerial*)serial->ser)->end();
    DEBUG("'xbee_ser_close' called");
    return 0;
}


int xbee_ser_break( xbee_serial_t *serial, int enabled)
{
    DEBUG("'xbee_ser_break' called");

    return 0;
}


int xbee_ser_flowcontrol( xbee_serial_t *serial, int enabled)
{
    DEBUG("'xbee_ser_flowcontrol' called");
    return 0;
}


int xbee_ser_set_rts( xbee_serial_t *serial, int asserted)
{
   DEBUG("'xbee_ser_set_rts' called");
    return 0;
}


int xbee_ser_get_cts( xbee_serial_t *serial)
{
    DEBUG("'xbee_ser_get_cts' called");
    return 1;
}

///@}

//================================================== MY STUFF =========================================*/

// function that handles received User Data frames
int user_data_rx(xbee_dev_t *xbee, const void FAR *raw,uint16_t length, void FAR *context)
{
    DEBUG("'user_data_rx' called");
    XBEE_UNUSED_PARAMETER(xbee);
    XBEE_UNUSED_PARAMETER(context);
    Serial1.write((const char*)raw,length);
    // const xbee_frame_user_data_rx_t *data = (const xbee_frame_user_data_rx_t*) raw;
    // int payload_length = length - offsetof(xbee_frame_user_data_rx_t,
    //                                        payload);

    // printf("Message from %s interface:\n",
    //     xbee_user_data_interface(data->source));

    // // If all characters of message are printable, just print it as a string
    // // with printf().  Otherwise use hex_dump() for non-printable messages.
    // int printable = TRUE;
    // for (size_t i = 0; printable && i < payload_length; ++i) {
    //     if (!isprint(data->payload[i])) {
    //         printable = FALSE;
    //     }
    // }

    // if (printable) {
    //     printf("%.*s\n", payload_length, data->payload);
    // } else {
    //     printf("%.*s\n", payload_length, data->payload);
    //     hex_dump(data->payload, payload_length, HEX_DUMP_FLAG_OFFSET);
    // }

    return 0;
}
int dump_tx_status(xbee_dev_t *xbee, const void FAR *frame, uint16_t length, void FAR *context)
{
    DEBUG("'dump_tx_status' called");
    XBEE_UNUSED_PARAMETER(xbee);
    XBEE_UNUSED_PARAMETER(length);
    XBEE_UNUSED_PARAMETER(context);

    const xbee_frame_tx_status_t *tx_status = (const xbee_frame_tx_status_t*) frame;
    char buffer[40];
    const char *status = NULL;

    // Provide descriptive strings for the only two errors we expect
    // from sending User Data Relay frames.
    switch (tx_status->delivery) {
        case XBEE_TX_DELIVERY_INVALID_INTERFACE:
            status = "invalid interface";
            break;
        case XBEE_TX_DELIVERY_INTERFACE_BLOCKED:
            status = "interface blocked";
            break;
        default:
            sprintf(buffer, "unknown status 0x%X", tx_status->delivery);
            status = buffer;
    }

    printf("Error on message id 0x%02X: %s\n", tx_status->frame_id, status);

    return 0;
}
const xbee_dispatch_table_entry_t xbee_frame_handlers[] =
{
    XBEE_FRAME_HANDLE_LOCAL_AT,
    { XBEE_FRAME_USER_DATA_RX, 0, user_data_rx, NULL },
    { XBEE_FRAME_TX_STATUS, 0, dump_tx_status, NULL },
    XBEE_FRAME_TABLE_END
};

int sendUserDataRelayAPIFrame(xbee_dev_t *xbee, const char *tx, const int num_tx)
{
    DEBUG("'sendUserDataRelayAPIFrame' called");
    // Note: It's safe to pass the pointer here (no need to create a deep copy) because xbee_user_data_relay_tx() will write out the bytes right away.
    int ret = xbee_user_data_relay_tx(xbee, XBEE_USER_DATA_IF_MICROPYTHON, tx, num_tx);
    if (ret < 0)
    {
        printf("%s: ERROR: Failed to send frame to the XBee via serial. Error code: %d\n", __func__, ret);
        return ret;  // The value is negative so it contains the error code.
    }
    return 0;
}

void setup() {
    
    lv_init();
    lv_disp_buf = (lv_color_t *)heap_caps_malloc(LVGL_LCD_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    lv_disp_buf2 = (lv_color_t *)heap_caps_malloc(LVGL_LCD_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    lv_disp_draw_buf_init(&disp_buf, lv_disp_buf, lv_disp_buf2, LVGL_LCD_BUF_SIZE);
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_HRES;
    disp_drv.ver_res = LCD_VRES;
    disp_drv.flush_cb = lvgl_flush;
    disp_drv.draw_buf = &disp_buf;
    lcd_color_trans_done_register_cb(lcd_flush_ready, &disp_drv);
    lcd_init(32*1024);  
    lv_disp_drv_register(&disp_drv);
    ui_init();
    is_initialized_lvgl = true;

    // your setup code here:
    Serial1.begin(9600, SERIAL_8N1, 18, 17);

    Serial.begin(115200);
    DEBUG("Booted");

//================================================== DAVE'S STUFF =========================================*/

    // static const char PING_REQUEST[] = "\x01PING";  // CSXB_MSG_PING_REQUEST=0x01 followed by arbitrary payload
    // static const char MQTT_START_REQUEST[] = "\x16MQTT_START";  // CSXB_MSG_MQTT_START=0x16 followed by arbitrary payload
    // char cmdstr[256];
    // xbee_serial_t XBEE_SERPORT;
    // int iface = XBEE_USER_DATA_IF_MICROPYTHON;
    // int status;
    
    // // initialize the serial and device layer for this XBee device
    // DEBUG("Calling xbee_dev_init()");
    // if (xbee_dev_init(&my_xbee, &XBEE_SERPORT, NULL, NULL)) {
    //     DEBUG("Failed to initialize device.\n");
    //     while(1);
    // }

    // // Initialize the AT Command layer for this XBee device and have the
    // // driver query it for basic information (hardware version, firmware version,
    // // serial number, IEEE address, etc.)
    // DEBUG("Calling xbee_cmd_init_device");
    // DEBUG("xbee_cmd_init_device returned:" + String(xbee_cmd_init_device(&my_xbee)));
    // DEBUG( "Waiting for driver to query the XBee device...\n");
    // do {
    //     xbee_dev_tick(&my_xbee);
    //     status = xbee_cmd_query_status(&my_xbee);
    // } while (status == -EBUSY);
    // if (status) {
    //     DEBUG( "Error: (" + String(status) + ") waiting for query to complete.\n");
    // }

    // // report on the settings
    // xbee_dev_dump_settings(&my_xbee, XBEE_DEV_DUMP_FLAG_DEFAULT);

//     printf("Enter messages for %s interface:\n",
//            xbee_user_data_interface(iface));

//     while (1) {
//         int linelen;
//         do {
//             linelen = xbee_readline(cmdstr, sizeof cmdstr);
//             if (linelen == -ENODATA) {
//                 while(1);
//             }
//             status = xbee_dev_tick(&my_xbee);
//             if (status < 0) {
//                printf("Error %d from xbee_dev_tick().\n", status);
//                while(1);
//             }
//         } while (linelen == -EAGAIN);

//         // blank line changes to next interface
//     //    if (*cmdstr == '\0') 
//     //	{
//     //        iface = XBEE_USER_DATA_IF_MICROPYTHON;
//     //        printf("Enter messages for %s interface:\n",
//     //               xbee_user_data_interface(iface));
//     //        continue;
//     //    }
//  status = sendUserDataRelayAPIFrame(&my_xbee, PING_REQUEST, sizeof PING_REQUEST);
//  status = sendUserDataRelayAPIFrame(&my_xbee, MQTT_START_REQUEST, sizeof PING_REQUEST);
//  //       status = xbee_user_data_relay_tx(&my_xbee, iface,
//  //                                        cmdstr, strlen(cmdstr));
//         if (status < 0) {
//             printf("error %d sending data\n", status);
//         } else {
//         }
//             printf("sent message id 0x%02X\n", status);
//             printf("sent message id 0x%s\n", PING_REQUEST);
//     }

//================================================== DAVE'S STUFF =========================================*/


}


void loop() {
        
    // Commands Received over USBSerial
    if (Serial.available())
    {
        DEBUG("USBSerial Input!");
        ReceivedInput = InputType::USBSERIALCOMMAND;
    }

    switch (ReceivedInput)
    {
    case InputType::USBSERIALCOMMAND:
        {
           
            USBSerialRXstr = Serial.readStringUntil('\n');
            DEBUG("USB Serial Command Received: " + USBSerialRXstr);


            // ==================== FW COMMAND ============================

            if(USBSerialRXstr.compareTo("FW\r") == 0)
            {
                DEBUG("Firmware Command Received");

                if (SendString("+++") == RXCode::ERR)
                {
                    DEBUG("Send String Failed");
                }
           
            XBeeReceivedText = ReceiveString("ATVR\r");
            
            lv_textarea_set_text(ui_TextArea1, XBeeReceivedText.c_str());
            }
            
            // ==================== XBEE Test COMMAND ============================

            if(USBSerialRXstr.compareTo("T\r") == 0)
            {
                DEBUG("XBee Test Command Received");

                // initialize the serial and device layer for this XBee device
                DEBUG("Calling xbee_dev_init()");
                XBEE_SERPORT.ser = &Serial1;
                XBEE_SERPORT.baudrate = 9600;
                strcpy(XBEE_SERPORT.portname,"Serial1");
                XBEE_SERPORT.pin_rx = 18;
                XBEE_SERPORT.pin_tx = 17;
                if (xbee_dev_init(&my_xbee, &XBEE_SERPORT, NULL, NULL)) {
                    DEBUG("Failed to initialize device.\n");
                    while(1);
                }

                    // Initialize the AT Command layer for this XBee device and have the
                    // driver query it for basic information (hardware version, firmware version,
                    // serial number, IEEE address, etc.)
                    DEBUG("Calling xbee_cmd_init_device");
                    DEBUG("xbee_cmd_init_device returned:" + String(xbee_cmd_init_device(&my_xbee)));
                    DEBUG( "Waiting for driver to query the XBee device...\n");
                    do {
                        xbee_dev_tick(&my_xbee);
                        status = xbee_cmd_query_status(&my_xbee);
                    } while (status == -EBUSY);
                    if (status) {
                        DEBUG( "Error: (" + String(status) + ") waiting for query to complete.\n");
                    }

                    // report on the settings
                    xbee_dev_dump_settings(&my_xbee, XBEE_DEV_DUMP_FLAG_DEFAULT);

                    
                    
                    while (1)
                    {
                    status = xbee_dev_tick(&my_xbee);
                    if (status < 0)
                    {
                        printf("Error %d from xbee_dev_tick().\n", status);
                        //return -1;
                    }

                    delay(3000);
                    status = sendUserDataRelayAPIFrame(&my_xbee, PING_REQUEST, sizeof PING_REQUEST);
                    if (status < 0) 
                    {
                        printf("error %d sending data\n", status);
                    }
                    else 
                    {

                    }

                    printf("sent message id 0x%02X\n", status);
                    printf("sent message id 0x%s\n", PING_REQUEST);
                    }

                    // status = sendUserDataRelayAPIFrame(&my_xbee, PING_REQUEST, sizeof PING_REQUEST);
                    // status = sendUserDataRelayAPIFrame(&my_xbee, MQTT_START_REQUEST, sizeof PING_REQUEST);
                    //        status = xbee_user_data_relay_tx(&my_xbee, iface,
                    //                                         cmdstr, strlen(cmdstr));
                    // if (status < 0) {
                    //     USBSerial.printf("error %d sending data\n", status);
                    // } else {
                    //     USBSerial.printf("sent message id 0x%02X\n", status);
                    //     USBSerial.printf("sent message id 0x%s\n", PING_REQUEST);
                    // }


            }



            ReceivedInput = InputType::NONE;
        }
        break;

    default:
        break;
    } 
    

    lv_timer_handler();
    delay(3);
}



RXCode SendString(String TXStr){
    
    DEBUG("Sending: " + TXStr);
    lv_textarea_set_text(ui_TextArea2,TXStr.c_str());
    //Serial1.print(TXstr);
    xbee_ser_write(nullptr,TXStr.c_str(),TXStr.length());
    if (WaitForOK() == false)
    {
        DEBUG("OK Not Recevied");
        return RXCode::ERR;
    }
    
    DEBUG("OK Recevied");
    return RXCode::OK;
}
char recv_buf[16384];
String ReceiveString(String TXStr)
{   

    DEBUG("Sending: " + TXStr);
    lv_textarea_set_text(ui_TextArea2,TXStr.c_str());
    //Serial1.print(TXStr);
    xbee_ser_write(nullptr,TXStr.c_str(),TXStr.length());
    DEBUG("Waiting for response");
    for(XBeeResponseTime_ms=0;XBeeResponseTime_ms<XBEE_RESPONSETIME_MAX && !Serial1.available();XBeeResponseTime_ms++) {delay(1);}

    if (XBeeResponseTime_ms>XBEE_RESPONSETIME_MAX)
    {
        DEBUG("TIMED OUT");
        return "TIMEOUT";
    }
    char ch = 0;
    *recv_buf=0;
    char* sz = recv_buf;
    while(ch!='\r') {
        int i = xbee_ser_getchar(nullptr);
        if(-1<i) {
            ch=i;
            *sz++ = ch;
            *sz=0;
        }
    }
    String RXstr(recv_buf);
    //while(Serial1.available()){Serial1.read();}

    DEBUG("Received: " + RXstr);
    
    return RXstr;
    
}

bool WaitForOK()
{   
    
    DEBUG("Waiting for response");
    for(XBeeResponseTime_ms=0;XBeeResponseTime_ms<XBEE_RESPONSETIME_MAX && !Serial1.available();XBeeResponseTime_ms++) {delay(1);}

    if (XBeeResponseTime_ms>XBEE_RESPONSETIME_MAX)
    {
        DEBUG("TIMED OUT");
        return false;
    }
    while(Serial1.available()<3) { delay(1);}
    xbee_ser_read(nullptr,recv_buf,sizeof(recv_buf));
    String RXstr(recv_buf);
    String OKstr = "OK\r";

    DEBUG("Received: " + RXstr);

    int count = RXstr.compareTo(OKstr);

    if ( count > 0)
    {
        DEBUG("Greater than Zero: " + String(count));
        return false;        
    }
    else if (count < 0)
    {
        DEBUG("Less than Zero: " + String(count));
        return false; 
    }
    else
    {
         DEBUG("EQUAL!: " + String(count));
         return true;
    }   
 
    
}

void DEBUG(String DebugStr)
{
    
    if (PrevDebugStr.compareTo(DebugStr) != 0)
    {
        if (DebugCount == 0)
        {
            PrevDebugStr = DebugStr;
            Serial.println(DebugStr);
        }else
        {
            Serial.println("REPEAT" + String(DebugCount) + ": " + PrevDebugStr);
            Serial.println(DebugStr);
            DebugCount = 0;
        }
    }
    else
    {
       DebugCount++;
    }
}


