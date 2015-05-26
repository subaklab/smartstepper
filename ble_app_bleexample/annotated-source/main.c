// ![](http://subak.io/wp-content/uploads/2015/04/cropped-subak.png)
//
// # BLE 서비스 예제 분석 
// 일시: 20150526
// 작성: subak.io

/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the license.txt file.
 */

// [nAN-36](https://www.nordicsemi.com/eng/nordic/download_resource/24020/5/86719377) 참고
 
/**
 * This file is the main file for the application described in application note
 * nAN-36 Creating Bluetooth� Low Energy Applications Using nRF51822.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "boards.h"
#include "app_scheduler.h"
#include "softdevice_handler.h"
#include "app_timer_appsh.h"
#include "ble_error_log.h"
#include "app_gpiote.h"
#include "app_button.h"
#include "ble_debug_assert_handler.h"
#include "pstorage.h"
#include "ble_lbs.h"
#include "bsp.h"
#include "ble_gap.h"
#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define WAKEUP_BUTTON_PIN               BSP_BUTTON_0                                /**< Button used to wake up the application. */

#define ADVERTISING_LED_PIN_NO          BSP_LED_0                                   /**< Is on when device is advertising. */
#define CONNECTED_LED_PIN_NO            BSP_LED_1                                   /**< Is on when device has connected. */

#define LEDBUTTON_LED_PIN_NO            BSP_LED_0
#define LEDBUTTON_BUTTON_PIN_NO         BSP_BUTTON_1

#define DEVICE_NAME                     "LedButtonDemo"                             /**< Name of device. Will be included in the advertising data. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            2                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_GPIOTE_MAX_USERS            1                                           /**< Maximum number of users of the GPIOTE handler. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SEC_PARAM_TIMEOUT               30                                          /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static ble_lbs_t                        m_lbs;

#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. */

// Persistent storage system event handler
// 에러의 결과를 저장하는 듯 (확인필요)
void pstorage_sys_event_handler (uint32_t p_evt);

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
   ble_debug_assert_handler(error_code, line_num, p_file_name);

    /* On assert, the system can only recover with a reset.
    NVIC_SystemReset();
    */
}

// SoftDevice에서 assert할때 false 나면 실행되는 콜백함수 
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

// BLE 상태 표시하는 LED를 위한 GPIO 초기화 
static void leds_init(void)
{
    nrf_gpio_cfg_output(ADVERTISING_LED_PIN_NO);
    nrf_gpio_cfg_output(CONNECTED_LED_PIN_NO);
    nrf_gpio_cfg_output(LEDBUTTON_LED_PIN_NO);
}


// TIMER 초기화. 스케쥴러에서 이용
static void timers_init(void)
{
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, true);
}

// BLE GAP 초기화 (디바이스 이름, 외형?, 접속 설정... 설정값)
// DEVICE_NAME 을 수정하면 목록에 뜨는 이름이 수정되나 (확인필요)
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


// BLE Advertising 초기화
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;
    
    ble_uuid_t adv_uuids[] = {{LBS_UUID_SERVICE, m_lbs.uuid_type}};

    // BLE Advertising 값 만들기 advdata에 설정.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

  
    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = adv_uuids;
    
    err_code = ble_advdata_set(&advdata, &scanrsp);
    APP_ERROR_CHECK(err_code);
}

// 커스텀 서비스에서 led 를 켜고 끄기위한 핸들러.
// 일부러 어플래케이션 단에 두었다. 
static void led_write_handler(ble_lbs_t * p_lbs, uint8_t led_state)
{
    if (led_state)
    {
        nrf_gpio_pin_set(LEDBUTTON_LED_PIN_NO);
    }
    else
    {
        nrf_gpio_pin_clear(LEDBUTTON_LED_PIN_NO);
    }
}

// LED Button 서비스 초기화. 커스텀 서비스
static void services_init(void)
{
    uint32_t err_code;
    ble_lbs_init_t init;
    
    init.led_write_handler = led_write_handler;
    
    err_code = ble_lbs_init(&m_lbs, &init);
    APP_ERROR_CHECK(err_code);
}


// BLE 보안설정
static void sec_params_init(void)
{
    
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}



/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
// 커넥션 파라미터 모듈의 이벤트 핸들러. 
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

// 커넥션 파라미터 에러가 나면 실행되는 핸들러.. nrf_error를 보면 왜 
// 실행이 안되는지 정보가 있는 모양...
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


// BLE 커넷션 설정 초기화
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


// 타이머 시작인데... 아무일도 안하네
static void timers_start(void)
{
}


// BLE Advertising 시작
// Advertising 시작되면 ADVERTISING_LED_PIN_NO LED를 켜네.
static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;

    // Adavertising 설정값 지정
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
    nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
}


// # on_ble_evt
// 응용프로그램에서의 BLE 이벤트 핸들러.
// 응용프로그램단에 노출. 왜? 커넥션 정보를 이용하여 표시하거나 커넥션 정보를 이용하여 주변장치를 제어
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    static ble_gap_evt_auth_status_t m_auth_status;
		static ble_gap_master_id_t p_master_id;
		static ble_gap_sec_keyset_t keys_exchanged;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);
            nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            err_code = app_button_disable();
            APP_ERROR_CHECK(err_code);
            
            advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_SUCCESS,
                                                   &m_sec_params,&keys_exchanged);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0,BLE_GATTS_SYS_ATTR_FLAG_USR_SRVCS);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
            m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
            break;

        case BLE_GAP_EVT_SEC_INFO_REQUEST:
						
            if (p_master_id.ediv == p_ble_evt->evt.gap_evt.params.sec_info_request.master_id.ediv)
            {
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, &keys_exchanged.keys_central.p_enc_key->enc_info, &keys_exchanged.keys_central.p_id_key->id_info, NULL);
                APP_ERROR_CHECK(err_code);
								p_master_id.ediv = p_ble_evt->evt.gap_evt.params.sec_info_request.master_id.ediv;
            }
            else
            {
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, NULL, NULL,NULL);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING)
            {
                nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);

                nrf_gpio_cfg_sense_input(WAKEUP_BUTTON_PIN,
                                         BUTTON_PULL,
                                         NRF_GPIO_PIN_SENSE_LOW);
                
                err_code = sd_power_system_off();
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}


# ble_evt_dispatch
// ble_evt_dispatch BLE Stack 이벤트가 발생하면 실행되는 함수
// on_ble_evt, ble_conn_params_on_ble_evt, ble_lbs_on_ble_evt 를 콜해서
// 응용, 연결, 서비스에서 BLE Stack에 반응하게 만들어 준다.
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_lbs_on_ble_evt(&m_lbs, p_ble_evt);
}


// 시스템 이벤트가 발생하면 실행 (dispatch)
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
}


//BLE 스택 초기화
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    /* Initialize the SoftDevice handler module. */
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);

    /* Enable BLE stack  */
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    ble_gap_addr_t addr;
    
    err_code = sd_ble_gap_address_get(&addr);
    APP_ERROR_CHECK(err_code);
    sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE, &addr);
    APP_ERROR_CHECK(err_code);
    /* Subscribe for BLE events. */
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
    
    /* Register with the SoftDevice handler module for BLE events. */
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


// 이벤트 스케줄러 실행 (확인 필요) 이걸 해서 뭘 실행하는거냐?
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    uint32_t err_code;
    
    switch (pin_no)
    {
        case LEDBUTTON_BUTTON_PIN_NO:
            err_code = ble_lbs_on_button_change(&m_lbs, button_action);
            if (err_code != NRF_SUCCESS &&
                err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}

// GPIOTE 주변장치 초기화
static void gpiote_init(void)
{
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
}

// 버튼 초기화. 기본 풀업.
static void buttons_init(void)
{
    /* Note: Array must be static because a pointer to it will be saved in the Button handler
           module.
           */
    static app_button_cfg_t buttons[] =
    {
        {WAKEUP_BUTTON_PIN, false, BUTTON_PULL, NULL},
        {LEDBUTTON_BUTTON_PIN_NO, false, BUTTON_PULL, button_event_handler}
    };

    app_button_init(buttons, sizeof(buttons) / sizeof(buttons[0]), BUTTON_DETECTION_DELAY);
}


// power_manage: 슬립 슬립
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

// # main 함수
// * 주변장치초기화
// * BLE 스택 초기화
// * GAP 초기화
// * 서비스 초기화
// * BLE Advertising 초기화
// * BLE Connection 초기화
// * BLE 보안 초기화
// * 타이머 시작
// * BLE Advertising 시작
int main(void)
{
    leds_init();
    timers_init();
    gpiote_init();
    buttons_init();
    ble_stack_init();
    scheduler_init();    
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
    sec_params_init();

    timers_start();
    advertising_start();

    for (;;)
    {
      // 메인루프에서는 케쥴러에 일이 있으면 실행하고, 슬립모드에 들어간다. 
        app_sched_execute();
        power_manage();
    }
}
