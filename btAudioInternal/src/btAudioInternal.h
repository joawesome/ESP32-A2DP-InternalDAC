#ifndef BTAUDIO_H
#define BTAUDIO_H



#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "Arduino.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "driver/i2s.h"
#include "esp_avrc_api.h"
#include "filterInternal.h"
#include "DRCInternal.h"

//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"

#ifdef ARDUINO_ARCH_ESP32
#include "esp32-hal-log.h"
#include "esp32-hal-bt.h"
#endif

#define APP_CORE_TAG  "BT_APP_CORE"
#define APP_SIG_WORK_DISPATCH (0x01)

#define AUTOCONNECT_TRY_NUM (5)

typedef void (* app_callback_t) (uint16_t event, void *param);

typedef struct {
    uint16_t             sig;      /*!< signal to app_task */
    uint16_t             event;    /*!< message event id */
    app_callback_t          cb;       /*!< context switch callback */
    void                 *param;   /*!< parameter area needs to be last */
} app_msg_t;


// postprocessing 
enum {
    NOTHING = 0,
    FILTER,
	COMPRESS,
	FILTER_COMPRESS,
	BT_APP_EVT_STACK_UP = 0,
};
class btAudio {
  public:
	//Constructor
	btAudio(const char *devName);
	
	// Bluetooth functionality
	void begin();  
	void end();
	//virtual int16_t get_sound_data();
	void setSinkCallback(void (*sinkCallback)(const uint8_t *data, uint32_t len));
	
	// I2S Audio
	void InternalDAC();
	void volume(float vol);
	
	virtual esp_a2d_audio_state_t get_audio_state();
    virtual esp_a2d_mct_t get_audio_type();

	// Filtering
	void createFilter(int n, float hp,int type);
	void stopFilter();
	
	// Compression
	void compress(float T,float alphAtt,float alphRel, float R,float w,float mu );
	void decompress();
	
	// meta data
	void updateMeta();
	
	virtual void app_a2d_callback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
	virtual void app_task_handler();
	virtual void av_hdl_stack_evt(uint16_t event, void *p_param);
	static void i2sCallback(const uint8_t *data, uint32_t len);
	void connectToLastDevice();
	
	static void avrc_callback(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param);
	
	
	float _T=60.0;
	float _alphAtt=0.001;
	float _alphRel=0.1; 
	float _R=4.0;
	float _w=10.0;
	float _mu=0.0; 
	
	static String title;
    static String artist;
    static String album;
	static String genre;
	String connectionstate;
  protected:
  
    xQueueHandle app_task_queue;
    xTaskHandle app_task_handle;
  
    uint32_t m_pkt_cnt = 0;
	esp_a2d_audio_state_t m_audio_state = ESP_A2D_AUDIO_STATE_STOPPED;
    esp_a2d_audio_state_t audio_state;
	esp_a2d_mct_t audio_type;
	
	
  
	esp_bd_addr_t lastBda = {NULL};
	virtual void init_nvs();
	virtual void getLastBda();
    virtual void setLastBda(esp_bd_addr_t bda, size_t size);
	
	
	virtual void app_task_start_up(void);
	virtual bool app_send_msg(app_msg_t *msg);
	virtual bool app_work_dispatch(app_callback_t p_cback, uint16_t event, void *p_params, int param_len);
    virtual void app_work_dispatched(app_msg_t *msg);
	virtual void av_hdl_a2d_evt(uint16_t event, void *p_param);

  private:
    
    const char *_devName;
	const char * bt_name;
	bool _filtering=false;
	bool _compressing=false;
    static uint32_t  _sampleRate;
	static int _postprocess;
	
	
	
	
	// bluetooth address of connected device
	static uint8_t _address[6];
	static DRC _DRCR;
	static DRC _DRCL;
	static float _vol;
	static int16_t soundata1;
	static filter _filtLlp;
    static filter _filtRlp;
    static filter _filtLhp;
    static filter _filtRhp;


};	

#endif
