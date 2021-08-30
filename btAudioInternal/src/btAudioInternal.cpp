#include "btAudioInternal.h"

btAudio* actualbtAudio;
////////////////////////////////////////////////////////////////////
////////////// Nasty statics for i2sCallback ///////////////////////
////////////////////////////////////////////////////////////////////
 float btAudio::_vol=0.95;
 uint8_t btAudio::_address[6];
 uint32_t btAudio::_sampleRate=44100;
 
 int connectionTries = 0;
 extern "C" void app_task_handler_2(void *arg);
 extern "C" void app_a2d_callback_2(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
 extern "C" void audio_data_callback_2(const uint8_t *data, uint32_t len);
 extern "C" void app_rc_ct_callback_2(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param);
  
 String btAudio::title="";
 String btAudio::album="";
 String btAudio::genre="";
 String btAudio::artist="";
 
 int btAudio::_postprocess=0;
 filter btAudio::_filtLhp = filter(2,_sampleRate,3,highpass); 
 filter btAudio::_filtRhp = filter(2,_sampleRate,3,highpass);
 filter btAudio::_filtLlp = filter(20000,_sampleRate,3,lowpass); 
 filter btAudio::_filtRlp = filter(20000,_sampleRate,3,lowpass);  
 DRC btAudio::_DRCL = DRC(_sampleRate,60.0,0.001,0.2,4.0,10.0,0.0); 
 DRC btAudio::_DRCR = DRC(_sampleRate,60.0,0.001,0.2,4.0,10.0,0.0); 
 
 i2s_port_t i2s_port; 

////////////////////////////////////////////////////////////////////
////////////////////////// Constructor /////////////////////////////
////////////////////////////////////////////////////////////////////
btAudio::btAudio(const char *devName) {
  actualbtAudio = this;
  _devName=devName;  
}

////////////////////////////////////////////////////////////////////
////////////////// Bluetooth Functionality /////////////////////////
////////////////////////////////////////////////////////////////////
void btAudio::begin() {
 //Arduino bluetooth initialisation
  btStart();
  
     ESP_LOGD(BT_AV_TAG, "%s", __func__);
    //store parameters

    ESP_LOGI(BT_AV_TAG,"Device name will be set to '%s'",this->_devName);
    
	// Initialize NVS
	init_nvs();
	getLastBda();
	

  // bluedroid  allows for bluetooth classic
  esp_bluedroid_init();
  esp_bluedroid_enable();
  
   app_task_start_up();
  
  //Lambda for callback
    auto av_hdl_stack_evt_2 = [](uint16_t event, void *p_param) {
        ESP_LOGD(BT_AV_TAG, "%s", __func__);
        if (actualbtAudio) {
            actualbtAudio->av_hdl_stack_evt(event,p_param);
        }
    };
	app_work_dispatch(av_hdl_stack_evt_2, BT_APP_EVT_STACK_UP, NULL, 0);
}

void btAudio::end() {
  esp_a2d_sink_deinit();
  esp_bluedroid_disable();
  esp_bluedroid_deinit();
  btStop();  
}

esp_a2d_audio_state_t btAudio::get_audio_state() {
  return audio_state;
}

/*void btAudio::get_sound_data() {
  return (soundata1);
}*/

esp_a2d_mct_t btAudio::get_audio_type() {
  return audio_type;
}


void  btAudio::app_a2d_callback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    // lambda for callback
    auto av_hdl_a2d_evt_2 =[](uint16_t event, void *p_param){
        ESP_LOGD(BT_AV_TAG, "%s", __func__);
        if (actualbtAudio) {
            actualbtAudio->av_hdl_a2d_evt(event,p_param);  
        }
    };
   ESP_LOGD(BT_AV_TAG, "%s", __func__);
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT:
        ESP_LOGD(BT_AV_TAG, "%s ESP_A2D_CONNECTION_STATE_EVT", __func__);
        app_work_dispatch(av_hdl_a2d_evt_2, event, param, sizeof(esp_a2d_cb_param_t));
        break;
    case ESP_A2D_AUDIO_STATE_EVT:
        ESP_LOGD(BT_AV_TAG, "%s ESP_A2D_AUDIO_STATE_EVT", __func__);
        audio_state = param->audio_stat.state;
        app_work_dispatch(av_hdl_a2d_evt_2,event, param, sizeof(esp_a2d_cb_param_t));
        break;
    case ESP_A2D_AUDIO_CFG_EVT: {
        ESP_LOGD(BT_AV_TAG, "%s ESP_A2D_AUDIO_CFG_EVT", __func__);
        app_work_dispatch(av_hdl_a2d_evt_2, event, param, sizeof(esp_a2d_cb_param_t));
        break;
    }
    default:
        ESP_LOGE(BT_AV_TAG, "Invalid A2DP event: %d", event);
        break;
    } 

}



void btAudio::updateMeta() {
  uint8_t attr_mask = ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST | ESP_AVRC_MD_ATTR_ALBUM | ESP_AVRC_MD_ATTR_GENRE;
  esp_avrc_ct_send_metadata_cmd(1, attr_mask);
}
void btAudio::avrc_callback(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param) {
  esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(param);
  char *attr_text;
  String mystr;

  switch (event) {
    case ESP_AVRC_CT_METADATA_RSP_EVT: {
      attr_text = (char *) malloc (rc->meta_rsp.attr_length + 1);
      memcpy(attr_text, rc->meta_rsp.attr_text, rc->meta_rsp.attr_length);
      attr_text[rc->meta_rsp.attr_length] = 0;
      mystr = String(attr_text);

      switch (rc->meta_rsp.attr_id) {
        case ESP_AVRC_MD_ATTR_TITLE:
          //Serial.print("Title: ");
          //Serial.println(mystr);
		  title= mystr;
          break;
        case ESP_AVRC_MD_ATTR_ARTIST:
          //Serial.print("Artist: ");
          //Serial.println(mystr);
          artist= mystr;
		  break;
        case ESP_AVRC_MD_ATTR_ALBUM:
          //Serial.print("Album: ");----
          //Serial.println(mystr);
          album= mystr;
		  break;
        case ESP_AVRC_MD_ATTR_GENRE:
          //Serial.print("Genre: ");
          //Serial.println(mystr);
          genre= mystr;
		  break;
      }
      free(attr_text);
  }break;
    default:
      ESP_LOGE(BT_RC_CT_TAG, "unhandled AVRC event: %d", event);
      break;
  }
}
/*void btAudio::setSinkCallback(void (*sinkCallback)(const uint8_t *data, uint32_t len) ){
	 esp_a2d_sink_register_data_callback(sinkCallback);
}*/
////////////////////////////////////////-////////////////////////////
////////////////// I2S Audio Functionality /////////////////////////
////////////////////////////////////////////////////////////////////
void btAudio::InternalDAC() {
   // i2s configuration
   i2s_port = (i2s_port_t) 0;
static const i2s_config_t i2s_config = {
      .mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
      .sample_rate = 44100, // corrected by info from bluetooth
      .bits_per_sample = (i2s_bits_per_sample_t) 16, /* the DAC module will only take the 8bits from MSB */
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
      .communication_format = I2S_COMM_FORMAT_I2S_MSB,
      .intr_alloc_flags = 0, // default interrupt priority
      .dma_buf_count = 8,
      .dma_buf_len = 64,
      .use_apll = false
  };

  
  // i2s pinout
  static const i2s_pin_config_t pin_config = {
    .bck_io_num =26,//26-
    .ws_io_num = 27, //27
    .data_out_num = 25, //25
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  
  // now configure i2s with constructed pinout and config
  i2s_driver_install(i2s_port, &i2s_config, 0, NULL);
  if (i2s_config.mode & I2S_MODE_DAC_BUILT_IN) {
      ESP_LOGI(BT_AV_TAG, "Output will go to DAC pins");
      i2s_set_pin(i2s_port, NULL);      
    } else {
      i2s_set_pin(i2s_port, &pin_config);
    }
  
  // Sets the function that will handle data (i2sCallback)
   esp_a2d_sink_register_data_callback(audio_data_callback_2);
}
void btAudio::i2sCallback(const uint8_t *data, uint32_t len){
	
  size_t i2s_bytes_write;
  
  
  int16_t* data16=(int16_t*)data; //playData doesnt want const
  
  int16_t fy[2];
  float temp;
  
  int jump =4; //how many bytes at a time get sent to buffer
  int  n = len/jump; // number of byte chunks	
	switch (_postprocess) {
   case NOTHING:
        for(int i=0;i<n;i++){
		 //process left channel
		  
		 fy[0] = (int16_t)((*data16)*_vol) + 0x8000;
		 data16++;
		 
		 // process right channel
		 fy[1] = (int16_t)((*data16)*_vol) + 0x8000;
		 data16++;
		 int16_t soundata1 = fy[0];
		 i2s_write(i2s_port,(void*) fy, jump, &i2s_bytes_write,  100 ); 
		}
		break;
   case FILTER:
		for(int i=0;i<n;i++){
		 //process left channel
		 temp = _filtLlp.process(_filtLhp.process((*data16)*_vol));
		 
		 // overflow check
		 if(temp>32767){
		 temp=32767;
		 }
		 if(temp < -32767){
			temp= -32767;
		 }
		 fy[0] = (int16_t)(temp) + 0x8000;
	     data16++;
		 
		 // process right channel
		 temp = _filtRlp.process(_filtRhp.process((*data16)*_vol));
		 if(temp>32767){
		 temp=32767;
		 }
		 if(temp < -32767){
			temp= -32767;
		 }
		 fy[1] =(int16_t) (temp) + 0x8000;
		 data16++; 
		 i2s_write(i2s_port, fy, jump, &i2s_bytes_write,  100 );
		} 
		break;
   case COMPRESS:
	    for(int i=0;i<n;i++){
		 //process left channel
		 fy[0] = (*data16); 
		 fy[0]=_DRCL.softKnee(fy[0]*_vol) + 0x8000;
		 data16++;
		 
		 // process right channel
		 fy[1] = (*data16);
		 fy[1]=_DRCR.softKnee(fy[1]*_vol) + 0x8000;
		 data16++;
		 i2s_write(i2s_port, fy, jump, &i2s_bytes_write,  100 );
		}
		break;
   case FILTER_COMPRESS:
      for(int i=0;i<n;i++){
		 //process left channel(overflow check built into DRC)
		 fy[0] = _DRCL.softKnee(_filtLhp.process(_filtLlp.process((*data16)*_vol))) + 0x8000;
		 data16++;
		 
		 //process right channel(overflow check built into DRC)
		 fy[1] = _DRCR.softKnee(_filtRhp.process(_filtRlp.process((*data16)*_vol))) + 0x8000;
		 data16++;
		 i2s_write(i2s_port, fy, jump, &i2s_bytes_write,  100 );
		}
		break;	
  }

}



void btAudio::volume(float vol){
	_vol = constrain(vol,0,2);	
}
////////////////////////////////////////////////////////////////////
////////////////// Filtering Functionality /////////////////////////
////////////////////////////////////////////////////////////////////
void btAudio::createFilter(int n, float fc, int type){
   fc=constrain(fc,2,20000);
   switch (type) {
   case lowpass:
	_filtLlp= filter(fc,_sampleRate,n,type);
    _filtRlp= filter(fc,_sampleRate,n,type);
   break;
   case highpass:
	_filtLhp= filter(fc,_sampleRate,n,type);
    _filtRhp= filter(fc,_sampleRate,n,type);
   break;
   }
	_filtering=true;
	
	if(_filtering & _compressing){
	 _postprocess=3;	
	}else{
	 _postprocess=1;
	}
}
void btAudio::stopFilter(){
	_filtering=false;
	if(_compressing){
	  _postprocess = 2;
	}else{
	 _postprocess = 0;
	}
}

////////////////////////////////////////////////////////////////////
////////////////// Compression Functionality ///////////////////////
////////////////////////////////////////////////////////////////////
void btAudio::compress(float T,float alphAtt,float alphRel, float R,float w,float mu){
	_T=T;
	_alphAtt=alphAtt;
	_alphRel=alphRel;
	_R=R;
	_w=w;
	_mu=mu;
	
	_DRCL = DRC(_sampleRate,T,alphAtt,alphRel,R,w,mu);
	_DRCR = DRC(_sampleRate,T,alphAtt,alphRel,R,w,mu);
	_compressing=true;
	
	if(_filtering & _compressing){
	 _postprocess=3;	
	}else{
	 _postprocess=2;
	}	
}
void btAudio::decompress(){
	_compressing=false;
	
	if(_filtering){
	  _postprocess = 1;
	}else{
	 _postprocess = 0;
	}
}
////////////////////////////////////////////////////////////////////
////////////// Connect To Last Device Functionality/APP ////////////
////////////////////////////////////////////////////////////////////

bool btAudio::app_work_dispatch(app_callback_t p_cback, uint16_t event, void *p_params, int param_len)
{
    ESP_LOGD(BT_APP_CORE_TAG, "%s event 0x%x, param len %d", __func__, event, param_len);
    
    app_msg_t msg;
    memset(&msg, 0, sizeof(app_msg_t));

    msg.sig = APP_SIG_WORK_DISPATCH;
    msg.event = event;
    msg.cb = p_cback;

    if (param_len == 0) {
        return app_send_msg(&msg);
    } else if (p_params && param_len > 0) {
        if ((msg.param = malloc(param_len)) != NULL) {
            memcpy(msg.param, p_params, param_len);
            return app_send_msg(&msg);
        }
    }

    return false;
}

void btAudio::app_work_dispatched(app_msg_t *msg)
{
    ESP_LOGD(BT_AV_TAG, "%s", __func__);
    if (msg->cb) {
        msg->cb(msg->event, msg->param);
    }
}

bool btAudio::app_send_msg(app_msg_t *msg)
{
    ESP_LOGD(BT_AV_TAG, "%s", __func__);
    if (msg == NULL) {
        return false;
    }

    if (xQueueSend(app_task_queue, msg, 10 / portTICK_RATE_MS) != pdTRUE) {
        ESP_LOGE(BT_APP_CORE_TAG, "%s xQueue send failed", __func__);
        return false;
    }
    return true;
}

void btAudio::app_task_handler()
{
    ESP_LOGD(BT_AV_TAG, "%s", __func__);
    app_msg_t msg;
    for (;;) {
        if (pdTRUE == xQueueReceive(app_task_queue, &msg, (portTickType)portMAX_DELAY)) {
            ESP_LOGD(BT_APP_CORE_TAG, "%s, sig 0x%x, 0x%x", __func__, msg.sig, msg.event);
            switch (msg.sig) {
            case APP_SIG_WORK_DISPATCH:
                ESP_LOGW(BT_APP_CORE_TAG, "%s, APP_SIG_WORK_DISPATCH sig: %d", __func__, msg.sig);
                app_work_dispatched(&msg);
                break;
            default:
                ESP_LOGW(BT_APP_CORE_TAG, "%s, unhandled sig: %d", __func__, msg.sig);
                break;
            } // switch (msg.sig)

            if (msg.param) {
                free(msg.param);
            }
        }
    }
}

void  btAudio::av_hdl_stack_evt(uint16_t event, void *p_param)
{
    switch (event) {
    case BT_APP_EVT_STACK_UP: {
        ESP_LOGD(BT_AV_TAG, "%s av_hdl_stack_evt %s", __func__, "BT_APP_EVT_STACK_UP");
        /* set up device name */
        esp_bt_dev_set_device_name(_devName);

        /* initialize A2DP sink */
        esp_a2d_register_callback(app_a2d_callback_2);
        esp_a2d_sink_register_data_callback(audio_data_callback_2);
        esp_a2d_sink_init();

		if ( *lastBda != NULL ) connectToLastDevice();
		
        /* initialize AVRCP controller */
        esp_avrc_ct_init();
        esp_avrc_ct_register_callback(avrc_callback);

        /* set discoverable and connectable mode, wait to be connected */
        esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
        break;
    }
    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled evt %d", __func__, event);
        break;
    }
}

void btAudio::app_task_start_up(void)
{
    ESP_LOGD(BT_AV_TAG, "%s", __func__);
    app_task_queue = xQueueCreate(10, sizeof(app_msg_t));
    xTaskCreate(app_task_handler_2, "BtAppT", 2048, NULL, configMAX_PRIORITIES - 3, &app_task_handle);
    return;
}

void  btAudio::av_hdl_a2d_evt(uint16_t event, void *p_param)
{
    ESP_LOGD(BT_AV_TAG, "%s evt %d", __func__, event);
    esp_a2d_cb_param_t *a2d = NULL;
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
        ESP_LOGD(BT_AV_TAG, "%s ESP_A2D_CONNECTION_STATE_EVT", __func__);
        a2d = (esp_a2d_cb_param_t *)(p_param);
        uint8_t *bda = a2d->conn_stat.remote_bda;
        ESP_LOGI(BT_AV_TAG, "A2DP connection state: %s, [%02x:%02x:%02x:%02x:%02x:%02x]",
             m_a2d_conn_state_str[a2d->conn_stat.state], bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
		if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
			connectionstate = "disconnected"
            ESP_LOGI(BT_AV_TAG, "ESP_A2D_CONNECTION_STATE_DISCONNECTED");
			i2s_stop(i2s_port);
			i2s_zero_dma_buffer(i2s_port);
			if ( ( *lastBda != NULL ) && connectionTries < AUTOCONNECT_TRY_NUM ){
				ESP_LOGI(BT_AV_TAG,"Connection try number: %d", connectionTries);
				connectToLastDevice();
			}
			else esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
        } else if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED){
			connectionstate = "connected";
			ESP_LOGI(BT_AV_TAG, "ESP_A2D_CONNECTION_STATE_CONNECTED");
            esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_NONE);
			connectionTries = 0;
			i2s_start(i2s_port);
			setLastBda(a2d->conn_stat.remote_bda, sizeof(a2d->conn_stat.remote_bda));
        } else if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTING){
			connectionstate = "connecting";
			ESP_LOGI(BT_AV_TAG, "ESP_A2D_CONNECTION_STATE_CONNECTING");
			connectionTries++;
        }
        break;
    }
    case ESP_A2D_AUDIO_STATE_EVT: {
        ESP_LOGD(BT_AV_TAG, "%s ESP_A2D_AUDIO_STATE_EVT", __func__);
        a2d = (esp_a2d_cb_param_t *)(p_param);
        ESP_LOGI(BT_AV_TAG, "A2DP audio state: %s", m_a2d_audio_state_str[a2d->audio_stat.state]);
        m_audio_state = a2d->audio_stat.state;
        if (ESP_A2D_AUDIO_STATE_STARTED == a2d->audio_stat.state) {
            m_pkt_cnt = 0;
        }
        break;
    }
    case ESP_A2D_AUDIO_CFG_EVT: {
        ESP_LOGD(BT_AV_TAG, "%s ESP_A2D_AUDIO_CFG_EVT", __func__);
        esp_a2d_cb_param_t *esp_a2d_callback_param = (esp_a2d_cb_param_t *)(p_param);
        audio_type = esp_a2d_callback_param->audio_cfg.mcc.type;
        a2d = (esp_a2d_cb_param_t *)(p_param);
        ESP_LOGI(BT_AV_TAG, "a2dp audio_cfg_cb , codec type %d", a2d->audio_cfg.mcc.type);
        // for now only SBC stream is supported
      /*  if (a2d->audio_cfg.mcc.type == ESP_A2D_MCT_SBC) {
            i2s_config.sample_rate = 16000;
            char oct0 = a2d->audio_cfg.mcc.cie.sbc[0];
            if (oct0 & (0x01 << 6)) {
                i2s_config.sample_rate = 32000;
            } else if (oct0 & (0x01 << 5)) {
                i2s_config.sample_rate = 44100;
            } else if (oct0 & (0x01 << 4)) {
                i2s_config.sample_rate = 48000;
            }
            
            i2s_set_clk(i2s_port, i2s_config.sample_rate, i2s_config.bits_per_sample, (i2s_channel_t)2);

            ESP_LOGI(BT_AV_TAG, "configure audio player %x-%x-%x-%x\n",
                     a2d->audio_cfg.mcc.cie.sbc[0],
                     a2d->audio_cfg.mcc.cie.sbc[1],
                     a2d->audio_cfg.mcc.cie.sbc[2],
                     a2d->audio_cfg.mcc.cie.sbc[3]);
            ESP_LOGI(BT_AV_TAG, "audio player configured, samplerate=%d", i2s_config.sample_rate);
        } */
        break;
    }
    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled evt %d", __func__, event);
        break;
    }
}

void btAudio::init_nvs(){
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND){
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
	}
    ESP_ERROR_CHECK( err );
}

void btAudio::getLastBda(){
    nvs_handle my_handle;
    esp_err_t err;
    
    err = nvs_open("connected_bda", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) ESP_LOGE("NVS OPEN ERROR");

    esp_bd_addr_t bda;
    size_t size = sizeof(bda);
    err = nvs_get_blob(my_handle, "last_bda", bda, &size);
    if ( err != ESP_OK) ESP_LOGE(BT_AV_TAG, "ERROR GETTING NVS BLOB");
    if ( err == ESP_ERR_NVS_NOT_FOUND ) ESP_LOGE(BT_AV_TAG, "NVS NOT FOUND");
    nvs_close(my_handle);
    if (err == ESP_OK) memcpy(lastBda,bda,size);
}

void btAudio::setLastBda(esp_bd_addr_t bda, size_t size){
	if ( memcmp(bda, lastBda, size) == 0 ) return; //same value, nothing to store
	nvs_handle my_handle;
	esp_err_t err;
	
	err = nvs_open("connected_bda", NVS_READWRITE, &my_handle);
	if (err != ESP_OK) ESP_LOGE("NVS OPEN ERROR");
	err = nvs_set_blob(my_handle, "last_bda", bda, size);
	if (err == ESP_OK) err = nvs_commit(my_handle);
	else ESP_LOGE(BT_AV_TAG, "NVS WRITE ERROR");
	if (err != ESP_OK) ESP_LOGE(BT_AV_TAG, "NVS COMMIT ERROR");
	nvs_close(my_handle);
	memcpy(lastBda,bda,size);
}
	
void btAudio::connectToLastDevice(){
	esp_err_t status = esp_a2d_sink_connect(lastBda);
	if ( status == ESP_FAIL ) ESP_LOGE(BT_AV_TAG,"Failed connecting to device!");
}

extern "C" void audio_data_callback_2(const uint8_t *data, uint32_t len) {
  //ESP_LOGD(BT_AV_TAG, "%s", __func__);
  if (actualbtAudio)
    actualbtAudio->i2sCallback(data,len);
}

extern "C" void app_task_handler_2(void *arg) {
  ESP_LOGD(BT_AV_TAG, "%s", __func__);
  if (actualbtAudio)
    actualbtAudio->app_task_handler();
}

extern "C" void app_a2d_callback_2(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param){
  ESP_LOGD(BT_AV_TAG, "%s", __func__);
  if (actualbtAudio)
    actualbtAudio->app_a2d_callback(event, param);
}

extern "C" void app_rc_ct_callback_2(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param){
  ESP_LOGD(BT_AV_TAG, "%s", __func__);
  if (actualbtAudio)
    actualbtAudio->avrc_callback(event, param);
}