/**
 * Copyright (c) 2016 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "app_error.h"
#include "app_uart.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_util.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "ble_nus_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define APP_BLE_CONN_CFG_TAG    1                                       /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO   3                                       /**< BLE observer priority of the application. There is no need to modify this value. */
#define UART_TX_BUF_SIZE        256                                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        256                                     /**< UART RX buffer size. */

#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */

#define ECHOBACK_BLE_UART_DATA  1                                       /**< Echo the UART data that is received over the Nordic UART Service (NUS) back to the sender. */         

//コマンド
enum COMMAND_EVTS
{
    COMMAND_START           = 0,
    COMMAND_SETMEASURE      = 1,
    COMMAND_GETMEASURE      = 2,
    COMMAND_STOP            = 3,
    COMMAND_SETPARAM        = 4,
    COMMAND_GETPARAM        = 5,
    COMMAND_SETTIME         = 6,
    COMMAND_GETTIME         = 7,
    COMMAND_DISCON          = 8
};
//コマンド総数と最長コマンド文字数,最大コマンド長パターン
#define COMMAND_TOTAL 9
#define MAX_COMMAND_LENGTH 11
#define MAX_COMMAND_PTN 3

BLE_NUS_C_ARRAY_DEF(m_ble_nus_c,NRF_SDH_BLE_CENTRAL_LINK_COUNT);                                             /**< BLE Nordic UART Service (NUS) client instance. */
NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_DB_DISCOVERY_ARRAY_DEF(m_db_disc,NRF_SDH_BLE_CENTRAL_LINK_COUNT);                                        /**< Database discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */
APP_TIMER_DEF(m_measurement_timer);

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

/** command for Transmitter board */
uint8_t def_command[][14] = {{0xA0,0x00,0x00},
                            {0x10,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
                            {0x20,0x00},
                            {0x30,0x00},
                            {0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
                            {0x50,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
                            {0x60,0x00},
                            {0x70,0x00}};

const uint8_t command_list[COMMAND_TOTAL][MAX_COMMAND_LENGTH] = {"start","setmeasure","getmeasure","stop","setparam","getparam","settime","gettime","discon"};
const uint8_t command_length[COMMAND_TOTAL][MAX_COMMAND_PTN] = {{5,9,22},{16,0,0},{10,0,0},{4,0,0},{24,26,27},{8,0,0},{20,0,0},{7,0,0},{8,0,0}};

static uint8_t measurement_time = 0;

static char const m_target_name[] = "hstpe00001";

static uint8_t sequence_num = 0x01;
static uint8_t module_table[NRF_SDH_BLE_CENTRAL_LINK_COUNT]; 
static uint8_t connect_device_cnt = 0;

static uint8_t enable_timerid;

typedef struct
{
    uint16_t an1_data;
    uint16_t an2_data;
    uint16_t an3_data;
    uint16_t an4_data;
    uint16_t an5_data;
    uint16_t gyro_x;
    uint16_t gyro_y;
    uint16_t gyro_z;
    uint16_t acceleration_x;
    uint16_t acceleration_y;
    uint16_t acceleration_z;
    uint16_t magnetic_x;
    uint16_t magnetic_y;
    uint16_t magnetic_z;

} recv_data_t;

/**@brief NUS UUID. */
static ble_uuid_t const m_nus_uuid =
{
    .uuid = BLE_UUID_NUS_SERVICE,
    .type = NUS_SERVICE_UUID_TYPE
};

/**@brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is only an example and is not meant for the final product. You need to analyze
 *          how your product is supposed to react in case of assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function for starting scanning. */
static void scan_start(void)
{
    ret_code_t ret;
    NRF_LOG_DEBUG("Scan Start!");
    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);

    ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(ret);
}


/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
         case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
         {
              err_code = p_scan_evt->params.connecting_err.err_code;
              APP_ERROR_CHECK(err_code);
         } break;

         case NRF_BLE_SCAN_EVT_CONNECTED:
         {
              ble_gap_evt_connected_t const * p_connected = 
                               p_scan_evt->params.connected.p_connected;
             // Scan is automatically stopped by the connection.
             NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x",
                      p_connected->peer_addr.addr[0],
                      p_connected->peer_addr.addr[1],
                      p_connected->peer_addr.addr[2],
                      p_connected->peer_addr.addr[3],
                      p_connected->peer_addr.addr[4],
                      p_connected->peer_addr.addr[5]
                      );
         } break;

         case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
         {
             NRF_LOG_INFO("Scan timed out.");
         } break;

         default:
             break;
    }
}

/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_nus_uuid);
    //err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &uuid_filter);
    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, m_target_name);
    APP_ERROR_CHECK(err_code);

    //err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, false);
    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER | NRF_BLE_SCAN_NAME_FILTER, false);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
}


/**@brief Function for handling characters received by the Nordic UART Service (NUS).
 *
 * @details This function takes a list of characters of length data_len and prints the characters out on UART.
 *          If @ref ECHOBACK_BLE_UART_DATA is set, the data is sent back to sender.
 */
static void ble_nus_chars_received_uart_print(uint8_t * p_data, uint16_t data_len)
{
    ret_code_t ret_val;

    NRF_LOG_DEBUG("Receiving data.");
    NRF_LOG_HEXDUMP_DEBUG(p_data, data_len);
    NRF_LOG_DEBUG("data length,%d",data_len);

    for (uint32_t i = 0; i < data_len; i++)
    {
        do
        {
            ret_val = app_uart_put(p_data[i]);
            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("app_uart_put failed for index 0x%04x.", i);
                APP_ERROR_CHECK(ret_val);
            }
        } while (ret_val == NRF_ERROR_BUSY);
    }
    if (p_data[data_len-1] == '\r')
    {
        while (app_uart_put('\n') == NRF_ERROR_BUSY);
    }
    if (ECHOBACK_BLE_UART_DATA)
    {
        // Send data back to the peripheral.
        do
        {
            for (uint8_t i = 0; i < connect_device_cnt; i++)
            {
                //Send only end module
                if(module_table[i] == 0x00)
                {
                    ret_val = ble_nus_c_string_send(&m_ble_nus_c[i], p_data, data_len);
                    if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
                    {
                        NRF_LOG_ERROR("Failed sending NUS message. Error 0x%x. ", ret_val);
                        APP_ERROR_CHECK(ret_val);
                    }
                }
            }
        } while (ret_val == NRF_ERROR_BUSY);
    }
}

/**@brief NUS で送られてきたコマンドを処理して命令用のバイナリを作る*/
static void creation_binary_from_command(uint8_t * p_data, uint16_t data_len, uint8_t * command)
{
    uint8_t i,j,com = -1,shift = 0;
    uint16_t hex;
    uint8_t hex_conversion[5] = {10000,1000,100,10,1};
    //uint8_t m_time = measurement_time;
    for(i=0; i<COMMAND_TOTAL; i++)
    {
        //コマンド名を確認
        if(strncmp(p_data,command_list[i],strlen(command_list[i])) == 0)
        {
            for(j=0; j<MAX_COMMAND_PTN; j++)
            {   
                //コマンド名とデータ長の組み合わせを確認
                if(command_length[i][j] == data_len)
                {
                    com = i;
                }
            }
        }
    }
    switch (com)
    {
    case COMMAND_START:
        //初期化
        memcpy(def_command[1],command,14);
        //簡易なら抜けて終わり
        if(data_len > 5)
        {
            switch(p_data[6]) //サンプリング種別
            {
            case 0: //High
                break;
            case 1: //Mid
                command[3] |= 0x01;
                break;
            case 2: //Low
                command[3] |= 0x02;
                break;
            }
            if(p_data[8] =! 0) //GAIN倍率200倍
            {
                command[3] |= 0x04;
            }
            if(data_len == 22)  //時刻指定あり
            {
                for(i=6; i<12; i++)
                {
                    command[i] = (p_data[i*2-1]<<4 | p_data[i*2]);
                }
            }
        }
        break;
    case COMMAND_SETMEASURE:
        //変換と代入
        for(i=0; i<5; i++)
        {
            measurement_time += p_data[11+i] - 0x30 * hex_conversion[i];    
        }
        break;
    case COMMAND_GETMEASURE:
    {
    uint8_t m_time = measurement_time;
        //初期化
        memcpy(def_command[0],command,3);
        //hexに変換
        while(m_time != 0)
        {
            hex |= (m_time % 16<<shift);
            m_time = m_time / 16;
            shift+=4;
        }
        //commandに入れる
        command[1] |= hex>>8;
        command[2] |= hex;
    }
        break;
    case COMMAND_STOP:
        //停止コマンド
        memcpy(def_command[2],command,2);
        break;
    case COMMAND_SETPARAM:
        //初期化
        memcpy(def_command[4],command,9);
        if(p_data[9] != 0)  //ファクトリーモードOFF
        {
            command[0] |= 0x01; //ファクトリーモードON
        }
        switch (p_data[11]) //動作モード
        {   
            case 0: //通常モード
                command[2] |= 0xFF;
                break;
            case 1: //スタンドアロンモード
                command[2] |= 0x01;
                break;
            case 2: //BLE同時書き込みモード
                command[2] |= 0x02;
                break;
            case 3: //USB接続モード
                command[2] |= 0x03;
                break;
            case 4: //OTAモード
                command[2] |= 0x80;
                break;
            default:
                break;
        }
        switch (p_data[13]) //自動電源OFF
        {
            case 0: //10分
                break;
            case 1: //120分
                command[3] |= 0x01;
                break;
            case 2: //なし
                command[3] |= 0x10;
                break;
            default:
                break;
        }
        switch (p_data[15]) //取得モード
        {
            //高速モード時のch数を記憶
            j = p_data[15];
            case 0: //通常モード
                break;
            case 1: //高速モード(1ch)
                command[4] |= 0x01;
                break;
            case 2: //高速モード(2ch)
                command[4] |= 0x10;
                break;
            case 3: //低速モード
                command[4] |= 0x11;
                break; 
            default:
                break;
        }
        //高速モードか
        if(data_len >= 26 )
        {
            //ch数ループ
            for(i=0; i<j; i++)
            {
                switch(p_data[17+i])
                {
                    case 1:
                        command[4] |= 0x04;
                        break;
                    case 2:
                        command[4] |= 0x08;
                        break;
                    case 3:
                        command[4] |= 0x10;
                        break;
                    case 4:
                        command[4] |= 0x20;
                        break;
                    case 5:
                        command[4] |= 0x40;
                        break;
                    default:
                        break;
                }
            }
        } else {
            j = 0;
        }
        //以降発生する高速モード2chによるズレは定義しておいたjを使う
        if(p_data[19+j] != 0)   //LED設定
        {
            command[5] |= 0x01;
        }
        //IDの入力
        uint8_t id_num = 0;
        uint32_t id_hex;
        for(i=0; i<5; i++)
        {
            id_num += p_data[21+i+j] - 0x30 * hex_conversion[i];    
        }
        //hexに変換
        while(id_num != 0)
        {
            id_hex |= (id_num % 16 << shift);
            id_num = id_num / 16;
            shift += 4;
        }
        //commandに入れる
        j=24;
        for(i=0; i<4; i++)
        {
            command[6+i] |= id_hex >> j;
            j -= 8;
        }
        break;
    case COMMAND_GETPARAM:
        //設定取得コマンド
        memcpy(def_command[3],command,2);
        break;
    case COMMAND_SETTIME:
        //初期化
        memcpy(def_command[5],command,2);
        for(i=0; i<12; i++)
        {
            //コマンドにデータを代入
            command[i+2] |= (p_data[i+8] - 0x30);
        }
        break;
    case COMMAND_GETTIME:
        //時刻取得コマンド
        memcpy(def_command[6],command,2);
        break;
    case COMMAND_DISCON:
        //初期化
        memcpy(def_command[7],command,2);
        switch(p_data[7])
        {
            case 0: //データ中継機器
                //デフォルト
                break;
            case 1: //データ測定機器
                command[1] = 0x01;
                break;
            case 2: //すべての機器
                command[1] = 0x02;
                break;
            default:
                break;
        }
        break;
    default:
        NRF_LOG_DEBUG("Invalid command");
        break;
    }
}

/**@brief Function for handling characters received by the Nordic UART Service (NUS).
 *
 * @details This function takes a list of characters of length data_len and prints the characters out on UART.
 *          If @ref ECHOBACK_BLE_UART_DATA is set, the data is sent back to sender.
 *
static void ble_nus_chars_received_uart_print(uint8_t * p_data, uint16_t data_len)
{
    ret_code_t ret_val;
    uint8_t ptr;
    recv_data_t data[5];
    uint16_t temperature;
    uint16_t relative_temperature;

    NRF_LOG_DEBUG("Receiving data.");
    if((p_data[0] & 0xF0) == 0x80)
    {
       switch(p_data[0]&0x03)
        {
            case 0x00:
                NRF_LOG_DEBUG("Sampling : Normal");
                break;
            case 0x01:
                NRF_LOG_DEBUG("Sampling : High Speed (1ch)");
                if(p_data[0]&0x02 != 0)
                {
                    NRF_LOG_DEBUG("CH : AN1 assign");
                }
                if(p_data[0]&0x04 != 0)
                {
                    NRF_LOG_DEBUG("CH : AN2 assign");
                }
                if(p_data[1]&0x01 != 0)
                {
                    NRF_LOG_DEBUG("CH : AN3 assign");
                }
                if(p_data[1]&0x02 != 0)
                {
                    NRF_LOG_DEBUG("CH : AN4 assign");
                }
                if(p_data[1]&0x08 != 0)
                {
                    NRF_LOG_DEBUG("CH : AN5 assign");
                }
            case 0x02:
                NRF_LOG_DEBUG("Sampling : High Speed (2ch)");
                if(p_data[0]&0x02 != 0)
                {
                    NRF_LOG_DEBUG("CH : AN1 assign");
                }
                if(p_data[0]&0x04 != 0)
                {
                    NRF_LOG_DEBUG("CH : AN2 assign");
                }
                if(p_data[1]&0x01 != 0)
                {
                    NRF_LOG_DEBUG("CH : AN3 assign");
                }
                if(p_data[1]&0x02 != 0)
                {
                    NRF_LOG_DEBUG("CH : AN4 assign");
                }
                if(p_data[1]&0x08 != 0)
                {
                    NRF_LOG_DEBUG("CH : AN5 assign");
                }
                break;
            case 0x03:
                NRF_LOG_DEBUG("Sampling : Low Speed");
                break;
            default:
                break;
        }
        if(p_data[1]&0x04 == 0){
            NRF_LOG_DEBUG("battery status : normal");
        }else{
            NRF_LOG_DEBUG("battery status : warning");
        }
        if(p_data[1]&0x10 == 0){
            NRF_LOG_DEBUG("flash write status : normal");
        }else{
            NRF_LOG_DEBUG("flash write status : error");
        }
        if(p_data[1]&0x20 == 0){
            NRF_LOG_DEBUG("BLE MTU status : normal");
        }else{
            NRF_LOG_DEBUG("BLE MTU status : error");
        }
        if(p_data[1]&0x40 == 0){
            NRF_LOG_DEBUG("flash status : writable");
        }else{
            NRF_LOG_DEBUG("flash status : full");
        }
        NRF_LOG_DEBUG("receive_sequence_number : 0x%x%x",p_data[2],p_data[3]);

        NRF_LOG_DEBUG("date : %d%d",p_data[4]>>4,p_data[4]&0x0F);
        NRF_LOG_DEBUG("       %d%d",p_data[5]>>4,p_data[5]&0x0F);
        NRF_LOG_DEBUG("       %d%d",p_data[6]>>4,p_data[6]&0x0F);
        NRF_LOG_DEBUG("       %d%d",p_data[7]>>4,p_data[7]&0x0F);
        NRF_LOG_DEBUG("       %d%d",p_data[8]>>4,p_data[8]&0x0F);
        
        ptr = 9;
        for(uint32_t i=0; i<6; i++)
        {
            data[i].an1_data = (p_data[ptr]<<8 | p_data[ptr+1]);
            data[i].an2_data = (p_data[ptr+2]<<8 | p_data[ptr+3]);
            data[i].an3_data = (p_data[ptr+4]<<8 | p_data[ptr+5]);
            data[i].an4_data = (p_data[ptr+6]<<8 | p_data[ptr+7]);
            data[i].an5_data = (p_data[ptr+8]<<8 | p_data[ptr+9]);
            NRF_LOG_DEBUG("an   0x%04x 0x%04x 0x%04x 0x%04x 0x%04x",data[i].an1_data,data[i].an2_data,data[i].an3_data,data[i].an4_data,data[i].an5_data);
            data[i].gyro_x = (p_data[ptr+10]<<8 | p_data[ptr+11]);
            data[i].gyro_y = (p_data[ptr+12]<<8 | p_data[ptr+13]);
            data[i].gyro_z = (p_data[ptr+14]<<8 | p_data[ptr+15]);
            NRF_LOG_DEBUG("jyro 0x%04x 0x%04x 0x%04x",data[i].gyro_x,data[i].gyro_y,data[i].gyro_z);
            data[i].acceleration_x = (p_data[ptr+16]<<8 | p_data[ptr+17]);
            data[i].acceleration_y = (p_data[ptr+18]<<8 | p_data[ptr+19]);
            data[i].acceleration_z = (p_data[ptr+20]<<8 | p_data[ptr+21]);
            NRF_LOG_DEBUG("acc  0x%04x 0x%04x 0x%04x",data[i].acceleration_x,data[i].acceleration_y,data[i].acceleration_z);
            data[i].magnetic_x = (p_data[ptr+22]<<8 | p_data[ptr+23]);
            data[i].magnetic_y = (p_data[ptr+24]<<8 | p_data[ptr+25]);
            data[i].magnetic_z = (p_data[ptr+26]<<8 | p_data[ptr+27]);
            NRF_LOG_DEBUG("mag  0x%04x 0x%04x 0x%04x",data[i].magnetic_x,data[i].magnetic_y,data[i].magnetic_z);
            if(i<=4)
            {
                ptr+=28;
            }
            else
            {
                temperature = (p_data[ptr+28]<<8 | p_data[ptr+29]);
                NRF_LOG_DEBUG("temperature_data : 0x%04x",temperature);
                relative_temperature = (p_data[ptr+30]<<8 | p_data[ptr+31]);
                NRF_LOG_DEBUG("relative_humidity_data : 0x%04x",relative_temperature);
            }
        }
    }
}*/


/**@brief   Function for handling app_uart events.
 *
 * @details This function receives a single character from the app_uart module and appends it to
 *          a string. The string is sent over BLE when the last character received is a
 *          'new line' '\n' (hex 0x0A) or if the string reaches the maximum data length.
 */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    
    static uint16_t index = 0;
    uint32_t ret_val;

    switch (p_event->evt_type)
    {
        /**@snippet [Handling data from UART] */
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (m_ble_nus_max_data_len)))
            {
                NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                do
                {
                    ret_val = ble_nus_c_string_send(&m_ble_nus_c, data_array, index);
                    if ( (ret_val != NRF_ERROR_INVALID_STATE) && (ret_val != NRF_ERROR_RESOURCES) )
                    {
                        APP_ERROR_CHECK(ret_val);
                    }
                } while (ret_val == NRF_ERROR_RESOURCES);

                index = 0;
            }
            break;

        /**@snippet [Handling data from UART] */
        case APP_UART_COMMUNICATION_ERROR:
            NRF_LOG_ERROR("Communication error occurred while handling UART.");
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            NRF_LOG_ERROR("Error occurred in FIFO module used by UART.");
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}


/**@brief Callback handling Nordic UART Service (NUS) client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS client handle. This identifies the NUS client.
 * @param[in]   p_ble_nus_evt Pointer to the NUS client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{
    ret_code_t err_code;
    uint8_t command;
    uint8_t destination = 0x01;

    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("Discovery complete.");
            err_code = ble_nus_c_handles_assign((p_ble_nus_c+p_ble_nus_evt->conn_handle), p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);

            err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c+p_ble_nus_evt->conn_handle);
            APP_ERROR_CHECK(err_code);
            
            NRF_LOG_INFO("Connected to device with Nordic UART Service.");
            connect_device_cnt++;
            
            //Resume Scan
            if(connect_device_cnt < NRF_SDH_BLE_CENTRAL_LINK_COUNT)
            {
                scan_start();
            }
            break;

        case BLE_NUS_C_EVT_NUS_TX_EVT:
            ble_nus_chars_received_uart_print(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);

            //物性研からの受信データでない
            if(! *p_ble_nus_evt->p_data & 0x80)
            {
                //コマンド作成
                creation_binary_from_command(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len,command);

                //測定時間通知の場合送信先を変える
                if(*p_ble_nus_evt->p_data == 0xA0)
                {
                    destination = 0x00;
                }
                else
                {
                //シーケンス番号付与
                set_sequence_number(&command);
                }
                //コマンド送信
                for (uint8_t i=0; i<sizeof(module_table); i++)
                {
                    //計測機器に送信
                    if(module_table[i] == destination)
                    {
                        err_code = ble_nus_c_string_send(&m_ble_nus_c[i], command, sizeof(command));
                        //測定時間設定済みかつタイマーstart
                        if((measurement_time != 0)&&(command == 0x10))
                        {
                            err_code = app_timer_start(m_measurement_timer,APP_TIMER_TICKS(measurement_time),m_measurement_timer);
                       }
                    }
                }
            } else {
                for(uint8_t i=0; i<sizeof(module_table); i++)
                {
                    //中継機器に送信
                    if(module_table[i] == 0x00)
                    {
                        err_code = ble_nus_c_string_send(&m_ble_nus_c[i],p_ble_nus_evt->p_data,p_ble_nus_evt->data_len);
                    }
                }
            }
            break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            connect_device_cnt--;
            scan_start();
            break;
    }
}
/**@snippet [Handling events from the ble_nus_c module] */


/**
 * @brief Function for handling shutdown events.
 *
 * @param[in]   event       Shutdown type.
 */
static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
            // Prepare wakeup buttons.
            err_code = bsp_btn_ble_sleep_mode_prepare();
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }

    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = ble_nus_c_handles_assign(&m_ble_nus_c[p_gap_evt->conn_handle], p_gap_evt->conn_handle,NULL);
            APP_ERROR_CHECK(err_code);

            //module_type Registration
            NRF_LOG_DEBUG("module type : %d",read_module_type());
            module_table[p_gap_evt->conn_handle] = read_module_type();

            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);

            // start discovery of services. The NUS Client waits for a discovery result
            err_code = ble_db_discovery_start(&m_db_disc[p_gap_evt->conn_handle],p_gap_evt->conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:

            NRF_LOG_INFO("Disconnected. conn_handle: 0x%x, reason: 0x%x",
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason);
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.");
            }
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported.
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("ATT MTU exchange completed.");

        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Ble NUS max data length set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_ble_nus_c->conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}

/**@brief Function for initializing the UART. */
static void uart_init(void)
{
    ret_code_t err_code;

    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Nordic UART Service (NUS) client. */
static void nus_c_init(void)
{
    ret_code_t       err_code;
    ble_nus_c_init_t init;
    for(uint8_t i=0; i<NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        init.evt_handler = ble_nus_c_evt_handler;
        err_code = ble_nus_c_init(&m_ble_nus_c[i], &init);
    }
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds. */
static void buttons_leds_init(void)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
}

void end_of_measurement_handler(void * p_context)
{
    ret_code_t err_code;
    uint8_t measure_stop ={0x20,0x00};
    set_sequence_number(&measure_stop);
    err_code = ble_nus_c_string_send(&m_ble_nus_c[enable_timerid], measure_stop, sizeof(measure_stop));
        if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) )
        {
            APP_ERROR_CHECK(err_code);
            NRF_LOG_DEBUG("Measure stop");
        }
}

/**@brief Function for initializing the timer. */
static void timer_init(void)
{
    /*static uint16_t*/
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&m_measurement_timer,APP_TIMER_MODE_SINGLE_SHOT,end_of_measurement_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for initializing the database discovery module. */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}

/** @brief Function for Sequence number control */
void set_sequence_number(uint8_t * p_data)
{
     //シーケンス番号のない命令を除外
    if((*p_data != 0x10)||(*p_data != 0x70))
    {
        *(p_data + 1) = sequence_num; 
        if(sequence_num >= 255){
            //シーケンス番号を範囲内に留める処理
            sequence_num = 1;
        }
        else
        {
            sequence_num++;
        }
    }
}

/**@brief Function for handling the idle state (main loop).
 * @details Handles any pending log operations, then sleeps until the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


int main(void)
{
    // Initialize.
    log_init();
    timer_init();
    uart_init();
    buttons_leds_init();
    db_discovery_init();
    power_management_init();
    ble_stack_init();
    gatt_init();
    nus_c_init();
    scan_init();

    // Start execution.
    printf("BLE UART central example started.\r\n");
    NRF_LOG_INFO("BLE UART central example started.");
    scan_start();
    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}