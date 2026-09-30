//#######################################################version demi finale
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_adc/adc_continuous.h"
#include "esp_timer.h"
#include "driver/usb_serial_jtag.h"
#include "driver/gpio.h"
#include <math.h>
#include "dsps_biquad.h"
#include "dsps_biquad_gen.h"
// #include "ringbuffer_64.h"
// #include "ringbuffer_32.h"
// #include "ringbuffer_4096.h"
// #include "ringbuffer_64_int16.h"

extern "C" {
    #include "ringbuffer_64.h"
    #include "ringbuffer_32.h"
    #include "ringbuffer_4096.h"
    #include "ringbuffer_64_int16.h"
    #include "ringbuffer_128.h"
}


#include <mpu6050.h>
#include <atomic>
#include <cstring>


#include "dl_model_base.hpp"

#define SEUIL_RELATIF_INTEGRAL 0.2 // algorithme détecttion de côté



// Déclaration exacte basée sur le nom de votre fichier modele_cnn.espdl
// extern "C"{
// extern const uint8_t sin_model_espdl_start[]  asm("_binary_sin_model_2_espdl_start");
// extern const uint8_t sin_model_espdl_end[]   asm("_binary_sin_model_2_espdl_end");
// }

// dl::Model *model = nullptr;


// ===================== CONFIGURATION =====================
/////piezo
const float FREQ_ECHANTILLONNAGE = 40000.0f;
const float FREQ_COUPURE_HP      = 1500.0f;
const float FREQ_COUPURE_LP      = 15000.0f;
const float Q_FILTRE             = 0.5f; //0.7071f;
 
const float F_NORM_HP = FREQ_COUPURE_HP / FREQ_ECHANTILLONNAGE;
const float F_NORM_LP = FREQ_COUPURE_LP / FREQ_ECHANTILLONNAGE;

 
// Coefficients {b0, b1, b2, a1, a2} (a0 = 1, déjà normalisé) — calculés une fois au démarrage
float coeffs_hp[5];
float coeffs_lp[5];
 
// Lignes à retard : 2 valeurs par filtre, une par voie (donc 4 filtres = 4 lignes à retard)
float w_hp_1[2] = {0, 0};
float w_lp_1[2] = {0, 0};
float w_hp_2[2] = {0, 0};
float w_lp_2[2] = {0, 0};
 
void init_filtres() {
    dsps_biquad_gen_hpf_f32(coeffs_hp, F_NORM_HP, Q_FILTRE);
    dsps_biquad_gen_lpf_f32(coeffs_lp, F_NORM_LP, Q_FILTRE);
}

void filtrer_voie(float *buffer, int len, float *w_hp, float *w_lp) {
    // HP puis LP, en cascade, exactement comme filtre_hp_x.process() puis filtre_lp_x.process()
    dsps_biquad_f32(buffer, buffer, len, coeffs_hp, w_hp);
    dsps_biquad_f32(buffer, buffer, len, coeffs_lp, w_lp);
}
 


////CONFIGURATION APRES LES FILTRES


//// POUR LES MESURES ADC
static adc_continuous_handle_t adc_handle = NULL;
static TaskHandle_t s_task_handle = NULL;

#define PIEZO_ADC_CHANNEL           ADC_CHANNEL_0  // Modification : Utilisation de la macro officielle ADC_CHANNEL_1 (GPIO1)
#define PIEZO_SAMPLE_FREQ_HZ        80000
#define PIEZO_BUFFER_SIZE_SAMPLES   64
#define PIEZO_BUFFER_BYTE_SIZE      (PIEZO_BUFFER_SIZE_SAMPLES * SOC_ADC_DIGI_RESULT_BYTES) // Modification v6.x : SOC_ADC_DIGI_RESULT_BYTES
#define NB_MESURES_ENVOI 64
#define SEUIL_DETECTION 2
RingBuffer_4096 fond;
RingBuffer_32 soudain;
RingBuffer_128 piezo_1;
RingBuffer_128 piezo_2;
RingBuffer_64_int16 accel_x;
RingBuffer_64_int16 accel_y;
RingBuffer_64_int16 accel_z;
RingBuffer_64_int16 rot_x;
RingBuffer_64_int16 rot_y;
RingBuffer_64_int16 rot_z;
RingBuffer_64 swift;
RingBuffer_64 norme_acc;




static int liste_index_1[128];
static int liste_index_2[128];
static float liste_integral_1[128];
static float liste_integral_2[128];
static float buffer_test_cote[2][128];
/// POUR LES MESURES ADC

// ===================== CONFIGURATION GENERALES=====================
#define WIFI_SSID        "tp-link-nico"
#define WIFI_PASS        "MOTDEPASSE"
#define WIFI_MAX_RETRY   15
#define PC_IP            "192.168.0.9999999" //. 
#define PC_PORT          8000

#define DEVICE_ID 1 // on définira un device 2 plus tard
// Paquet : [device_id(1)] + 5×[ax(2)ay(2)az(2)gx(2)gy(2)gz(2)](60) + [impact(1)] = 62 octets
#define PAQUET_SIZE      (2*64*6+1+ 4 + 4+4+1) // les int_16 valeurs daccelero, + classe unit8_t + float énergie + float rotation + float force+device
static uint8_t        paquet[PAQUET_SIZE];
static int             sock;
static struct sockaddr_in dest_addr;


#define I2C_SDA        GPIO_NUM_5  // D4 XIAO ESP32-S3
#define I2C_SCL        GPIO_NUM_6   // D5 XIAO ESP32-S3

#define MPU_ADDR 0x68  
static mpu6050_dev_t dev = { };


static const char *TAG = "main";

//static int16_t acc_buf[2][6]; //6 valeurs c'est 3 gyro et 3 accel 
//static float donnee_enregistrement[2][128];



//static TaskHandle_t    acc_task_handle = NULL;
static SemaphoreHandle_t udp_sem       = NULL;




static std::atomic<uint32_t> paquets_par_sec  = 0;

static float energie=1.0f; 
static uint8_t class_predicted=0;

// ===================== CONFIGURATION ADC PIÉZO =====================
static void configuration_adc_continuous(void) {
    adc_continuous_handle_cfg_t adc_config = {};
        adc_config.max_store_buf_size = PIEZO_BUFFER_BYTE_SIZE * 4;
        adc_config.conv_frame_size    = PIEZO_BUFFER_BYTE_SIZE;
    
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    adc_digi_pattern_config_t adc_pattern [2] = {};
    
        
        
        adc_pattern [0].atten     = ADC_ATTEN_DB_12;
        adc_pattern [0].channel   = PIEZO_ADC_CHANNEL; 
        adc_pattern [0].unit      = ADC_UNIT_1;
        adc_pattern [0].bit_width = SOC_ADC_DIGI_MIN_BITWIDTH;
        
        
        adc_pattern [1].atten     = ADC_ATTEN_DB_12;
         adc_pattern [1].channel   = ADC_CHANNEL_1;
         adc_pattern [1].unit      = ADC_UNIT_1;
         adc_pattern [1].bit_width = SOC_ADC_DIGI_MIN_BITWIDTH;
        
    
    

    adc_continuous_config_t dig_cfg = {};

        dig_cfg.sample_freq_hz = PIEZO_SAMPLE_FREQ_HZ;
        dig_cfg.conv_mode      = ADC_CONV_SINGLE_UNIT_1;
        dig_cfg.format         = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
        dig_cfg.pattern_num    = 2;
        dig_cfg.adc_pattern    = adc_pattern;
    
    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));
}

// ===================== INTERRUPTION (CALLBACK) ADC DECLENCHEUR =====================
static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle,const adc_continuous_evt_data_t *edata,void *user_data) 
{

   // compteur_callback_isr++;
    BaseType_t high_task_wakeup = pdFALSE;
    
    if (s_task_handle != NULL) {
        // Réveille la tâche principale dès qu'une frame est pleine en mémoire
        vTaskNotifyGiveFromISR(s_task_handle, &high_task_wakeup);
    }    
    return (high_task_wakeup == pdTRUE);
}
// ===================== WIFI =====================
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_num = 0;

static void event_handler(void *arg, esp_event_base_t base,
                           int32_t event_id, void *event_data) {
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "WiFi déconnecté, raison = %d", disc->reason);
        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "IP : " IPSTR, IP2STR(&e->ip_info.ip));
        s_retry_num = 0;
        esp_wifi_set_ps(WIFI_PS_NONE);
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}


static void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_event_handler_instance_t inst_id, inst_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                         event_handler, NULL, &inst_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                         event_handler, NULL, &inst_ip));
    // wifi_config_t wifi_cfg = {
    //     .sta = {
    //         .ssid = WIFI_SSID,
    //         .password = WIFI_PASS,
    //         .threshold.authmode = WIFI_AUTH_WPA2_PSK,
    //     },
    // };

wifi_config_t wifi_cfg = {};
std::strncpy((char*)wifi_cfg.sta.ssid, WIFI_SSID, sizeof(wifi_cfg.sta.ssid));
std::strncpy((char*)wifi_cfg.sta.password, WIFI_PASS, sizeof(wifi_cfg.sta.password));

// 3. Configuration du mode d'authentification
wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                        pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) ESP_LOGI(TAG, "WiFi connecté");
    else                           ESP_LOGE(TAG, "WiFi échoué");
}


static void udp_socket_init(void) {
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Impossible de créer le socket : errno %d", errno);
        return;
    }

    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family      = AF_INET;
    dest_addr.sin_port        = htons(PC_PORT);
    dest_addr.sin_addr.s_addr = inet_addr(PC_IP);  // ou inet_pton(AF_INET, PC_IP, &dest_addr.sin_addr)
}

/// FIN WIFI///////////////


#define ADDR 0x68



static void MPU6050_init(void) {
    // 1. Définition de la description du composant avec I2C à 400 kHz
    ESP_ERROR_CHECK(mpu6050_init_desc(&dev, ADDR, I2C_NUM_0, GPIO_NUM_5, GPIO_NUM_6));
    dev.i2c_dev.cfg.master.clk_speed = 400000; // Force 400 kHz (Fast Mode)

    // 2. Détection du composant MPU6050
    while (1) {
        esp_err_t res = i2c_dev_probe(&dev.i2c_dev, I2C_DEV_WRITE);
        if (res == ESP_OK) {
            ESP_LOGI(TAG, "Composant MPU60x0 détecté !");
            break;
        }
        ESP_LOGE(TAG, "MPU60x0 non trouvé, nouvelle tentative...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // 3. Initialisation du composant
    ESP_ERROR_CHECK(mpu6050_init(&dev));

    // (Optionnel) Force les plages de mesure au maximum si souhaité
    ESP_ERROR_CHECK(mpu6050_set_full_scale_accel_range(&dev, MPU6050_ACCEL_RANGE_16));
    ESP_ERROR_CHECK(mpu6050_set_full_scale_gyro_range(&dev, MPU6050_GYRO_RANGE_2000));
    uint8_t rate;
    
    esp_err_t err= mpu6050_get_rate(&dev, &rate);
    if((err=ESP_OK)){
ESP_LOGI(TAG,"acelerometer sample rate %d", rate);

    }
    else  
    {ESP_LOGI(TAG,"c'est la merde");}


    ESP_LOGI(TAG, "Accel range enum: %d", (int)dev.ranges.accel);
    ESP_LOGI(TAG, "Gyro range enum: %d", (int)dev.ranges.gyro);
}

static void mpu_read_direct() {
    
    mpu6050_raw_acceleration_t accel = {  };
    mpu6050_raw_rotation_t rotation = {  };
    //ESP_ERROR_CHECK(mpu6050_get_temperature(&dev, &temp));
    ESP_ERROR_CHECK(mpu6050_get_raw_acceleration(&dev, &accel));
    ESP_ERROR_CHECK(mpu6050_get_raw_rotation(&dev, &rotation));

    //ESP_ERROR_CHECK(mpu6050_get_raw_data(&dev, &accel, &rotation));
    

    ring_buffer_push_64_acc(&accel_x,  accel.x);
    ring_buffer_push_64_acc(&accel_y,  accel.y);
    ring_buffer_push_64_acc(&accel_z,  accel.z);
    ring_buffer_push_64_acc(&rot_x,  rotation.x);
    ring_buffer_push_64_acc(&rot_y,  rotation.y);
    ring_buffer_push_64_acc(&rot_z,  rotation.z);

    float a_x=(accel.x/2048)*9.81;
    float a_y=(accel.y/2048)*9.81;
    float a_z=(accel.z/2048)*9.81;
    
    float plan= (a_x*a_x+a_y*a_y);
    float norme= (a_y*a_y+a_x*a_x+a_z*a_z);

    float rot=plan/norme;

    ring_buffer_push_64(&swift,  rot);
    ring_buffer_push_64(&norme_acc, norme);
    // int64_t temps_us = esp_timer_get_time(); 
    // //r= // microsecondes depuis le boot
    // printf("delta t%lld\n", temps_us-ti);
    // ti=temps_us;
   
}

/////////////////////FIN CAPTEUR ACCELERO////////////////
// ===================== TÂCHE REVEILLÉE PAR L'INTERRUPTION =====================


void function_tache_adc(void *pvParameters) 
{
   printf("on est rentré dans la tache adc");
    uint32_t ret_num = 0;
    float voie_1[NB_MESURES_ENVOI / 2];
    float voie_2[NB_MESURES_ENVOI / 2];
    uint8_t piezo_buf[PIEZO_BUFFER_BYTE_SIZE];
    int roll_off=30;
    uint8_t I=0;
    //printf("on rentre dans la tache adc");
   
    // 1. Enregistrement du handle de la tâche AVANT d'activer les interruptions
    s_task_handle = xTaskGetCurrentTaskHandle(); 
    
    adc_continuous_evt_cbs_t cbs = { };
        cbs.on_conv_done = s_conv_done_cb; 
    
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL));
    
    // 2. On ne démarre l'ADC qu'ICI, une seule et unique fois
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));




///////model cnn UTILISATION
// printf("juste avant\n");

// ESP_LOGI("MEM", "SRAM libre avant copie : %d octets", 
//          heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

// ESP_LOGI("MEM", "PSRAM libre avant copie : %d octets", 
//          heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
// size_t model_size = sin_model_espdl_end - sin_model_espdl_start;
// // Allocation en RAM d'un buffer aligné sur 16 octets (16-byte aligned)
//     uint8_t *model_buf_aligned = (uint8_t *)heap_caps_aligned_alloc(16, model_size, MALLOC_CAP_8BIT);

//     if (model_buf_aligned != NULL) {
//         // Copie du modèle depuis la Flash vers la RAM alignée
//         memcpy(model_buf_aligned, sin_model_espdl_start, model_size);

//         // Instanciation de votre modèle ESP-DL à partir du buffer RAM aligné
//         // (Remplacez MonModeleCNN par le nom de votre classe générée)
//         model = new dl::Model((const char *)model_buf_aligned); 

//         ESP_LOGI("CNN", "Modèle instancié avec succès sur mémoire RAM alignée (16 bytes).");
//     } else {
//         ESP_LOGE("CNN", "Échec d'allocation mémoire pour le modèle aligné !");
//     }

//     dl::TensorBase *input_model = model->get_inputs().begin()->second;
   
   
// dl::TensorBase input_tensor= dl::TensorBase(
//         {1, 2, 1, 128},
//         nullptr,
//         0,
//         dl::DATA_TYPE_FLOAT
//     );
// float *in_ptr = (float *)(input_tensor).get_element_ptr();

// if (in_ptr==nullptr){
// ESP_LOGE(TAG, "input nullptr");

// }
 

// dl::TensorBase *model_output = model->get_outputs().begin()->second;
// //float *out_ptr = (float *)output_tensor->get_element_ptr(); 
// //int m =-15000;



// ESP_LOGI("MEM", "SRAM libre après copie : %d octets", 
//          heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

// ESP_LOGI("MEM", "PSRAM libre après copie : %d octets", 
//          heap_caps_get_free_size(MALLOC_CAP_SPIRAM));




         //////fin modele cnn
    while (1) {
        //printf("on rentre dans le while de l'adc");
   


    int index_1=0;
    int index_2=0;
        // Met la tâche à 0% CPU en attendant que le buffer matériel soit plein

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    //////LECTURE ADC
        esp_err_t err = adc_continuous_read(adc_handle, piezo_buf, PIEZO_BUFFER_BYTE_SIZE, &ret_num, 0);
        
        if (err == ESP_OK && ret_num > 0) {
            //index=0;
            for( int i=0; i<ret_num ; i+=SOC_ADC_DIGI_RESULT_BYTES)
            {   

            adc_digi_output_data_t *p_data =(adc_digi_output_data_t*)&piezo_buf[i];
            //uint32_t *value= (uint32_t*)&piezo_buf[i];
            float value= p_data->type2.data;
            uint8_t canal = p_data->type2.channel;
            //printf("%d",canal);
            //envoi_python_buf[index]=value;
            if ((canal==ADC_CHANNEL_0) && (index_1<NB_MESURES_ENVOI/2)) 
            {
            voie_1[index_1]=value;
                index_1+=1;
            }
            if ((canal==ADC_CHANNEL_1) && (index_2<NB_MESURES_ENVOI/2)) 
            {
            voie_2[index_2]=value;
                index_2+=1;// on suppose que cet index va toujours jusqu'à NB_MESURES_ENVOI
               
            }
            //index++;     
            }

            //printf("ca filtre");
            filtrer_voie(voie_1, NB_MESURES_ENVOI / 2, w_hp_1, w_lp_1);
            filtrer_voie(voie_2, NB_MESURES_ENVOI / 2, w_hp_2, w_lp_2);
            
            for (int k=0; k<NB_MESURES_ENVOI/2; k++){

            float amplitude = fabsf(voie_1[k]) + fabsf(voie_2[k]);
            ring_buffer_push_4096(&fond,amplitude);
            ring_buffer_push_32(&soudain, amplitude);
            ring_buffer_push_128(&piezo_1, voie_1[k]);
            ring_buffer_push_128(&piezo_2, voie_2[k]);

            }
            
            float s=ring_buffer_get_average_32(&soudain) ;
            float f=ring_buffer_get_average_4096(&fond) ;
            float m=ring_buffer_get_max_32(&soudain);
            

            
            if((I<=4)&(I>1)){I-=1;}
            
            if (I==1){

            adc_continuous_stop(adc_handle);
            float max_1= ring_buffer_get_average_128(&piezo_1);
            float max_2= ring_buffer_get_average_128(&piezo_2);

            I-=1;
            //compteur++;
           //printf("il y a un impact %d I= %d\n", compteur,I);
            energie=max_1+max_2;

            

    

    //buf_envoyer=buf_actif;
    for (int i = 0; i < 128; i++) {
        float out_1 = 0.0f;
        float out_2 = 0.0f;
       // float m1=ring_buffer_get_max_64(&piezo_1);
       // float m2=ring_buffer_get_max_64(&piezo_2);
        ring_buffer_pop_128(&piezo_1, &out_1);
        ring_buffer_pop_128(&piezo_2, &out_2);

  
        // donnee_enregistrement[0][i] = out_1;
        // donnee_enregistrement[1][i] = out_2;
        buffer_test_cote[0][i] = out_1;
        buffer_test_cote[1][i] = out_2;
    }


    /////normalisation pour le modele
//moyenne
// float sum_1=0.0f;
// float sum_2=0.0f;
//         for (int i=0; i<128; i++){
//         sum_1+=donnee_enregistrement[0][i]; 
       
//         sum_2+=donnee_enregistrement[1][i]; 
//         }
//         sum_1=sum_1/128.0f;
//         sum_2=sum_2/128.0f;
// //variance
// float v_1=0.0f;
// float v_2=0.0f;

//         for (int i=0; i<128; i++){
//         v_1+= (donnee_enregistrement[0][i]-sum_1)*(donnee_enregistrement[0][i]-sum_1);
   
//         v_2+= (donnee_enregistrement[1][i]-sum_2)*(donnee_enregistrement[1][i]-sum_2);
//     }
//     v_1=sqrt(v_1/127.0f)+1e-6f;
//     v_2=sqrt(v_2/127.0f)+1e-6f;
//     for (int i=0; i<128; i++){
//        in_ptr[i]= (donnee_enregistrement[0][i]-sum_1)/ v_1;
       
//        in_ptr[128+i]= (donnee_enregistrement[1][i]-sum_2)/v_2;        
//     }


    ////fin normalisatoin
   
 //debut application modele  
//     input_model->assign(&input_tensor);

// model->run();
//fin application modele
//


///LEctuer de la sortie du modele


// dl::TensorBase *output_tensor = new dl::TensorBase({1, 2}, nullptr, 0, dl::DATA_TYPE_FLOAT);
// output_tensor->assign(model_output);
// float *out_ptr= (float*) output_tensor->get_element_ptr();

// float score_classe_0 = out_ptr[0];
// float score_classe_1 = out_ptr[1];
// printf("class_0 %f, class_1 %f\n", score_classe_0, score_classe_1);
// class_predicted= (score_classe_1>score_classe_0)? 1:0;
//// fin lecture resuslat modele
/////
 int index_piezo_1=0;
        int index_piezo_2=0;
       
        float integral_1=0;
        float integral_2=0;
        int8_t signe_1=0;
        int8_t signe_2=0;
        int8_t index_integral_1=0;
        int8_t index_integral_2=0;
        for (int k= 0; k<127; k++){   // 128-1
            

            integral_1+=buffer_test_cote[0][k];
            if (buffer_test_cote[0][k]*buffer_test_cote[0][k+1]<=0){
                    liste_index_1[index_piezo_1]= k;
                    liste_integral_1[index_piezo_1++]=integral_1;
                    integral_1=0;
                } 

             integral_2+=buffer_test_cote[1][k];
            if (buffer_test_cote[1][k]*buffer_test_cote[1][k+1]<=0){
                    liste_index_2[index_piezo_2]= k;
                    liste_integral_2[index_piezo_2++]=integral_2;
                    integral_2=0;
                }     
        }
        float max=0;
        //on récuupère signe de la première integral du piezo 1
        for(int i =0; i<index_piezo_1; i++){           
            max=std::max(max, fabsf(liste_integral_1[i]));      // ainsi on a un bloc maximum        
        }
        for(int i =0; i<index_piezo_1; i++){           
               if (fabsf(liste_integral_1[i])>((SEUIL_RELATIF_INTEGRAL)*max))
               { 
                signe_1 = liste_integral_1[i]>0.0f ?  1.0f : (liste_integral_1[i]<0.0f ? -1.0f: 0.0f) ;
                index_integral_1=liste_index_1[i];
                
                break;
            }
        }
        //on récuupère signe de la première integral du piezo 2
        max=0;
        for(int i =0; i<index_piezo_2; i++){           
            max=std::max(max, fabsf(liste_integral_2[i]));      // ainsi on a un bloc maximum        
        }
        for(int i =0; i<index_piezo_2; i++){           
               if (fabsf(liste_integral_2[i])>((SEUIL_RELATIF_INTEGRAL)*max))
               { 
                signe_2 = liste_integral_2[i]>0.0f ?  1.0f : (liste_integral_2[i]<0.0f ? -1.0f : 0.0f) ;
                index_integral_2=liste_index_2[i];
                
                break;
            }
        }


        if (index_integral_1<index_integral_2){class_predicted= signe_1;}
        else { class_predicted =signe_2; }

        ///////
        xSemaphoreGive(udp_sem);

        //puisqu'il y a eu l'impact

        adc_continuous_start(adc_handle);
    
}
    

    //printf("%f", r);
            if (f>10e-6f)
            {
             //on capte l'impact enfonction du ration signal/ fond et on trie encore le signal avec fcritere et on attend prochiane frame pour enregistrer la forme de l'onde (I=1)
            float r = s/f;
            
            if ((r>SEUIL_DETECTION)&& (m>100.0f)&&(roll_off==0)){
            //printf("impact ici\n");
            I=4;
            roll_off= 200;   
             }
            if (roll_off>0){roll_off--;}
            }
//            
        }

}

}

// inline void normalisation(ringbuffer_64_int16 &ringo)
// {
// float m= ring_buffer_get_average_64_acc(&ringo);



// }
/// TACHE UPD ENVOI
static void tache_udp(void *arg) 

{
    
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "tache_udp prête");
    
    uint8_t classe;
   

    while (1) {
        
              
        mpu_read_direct();

            if (xSemaphoreTake(udp_sem, 0) == pdTRUE) {
            //printf("on envoit");

            classe=class_predicted;


            //printf("%d \n", mesure_acc_x);
            int offset = 0;
            //paquet[offset++] = DEVICE_ID;
    
            // for (int j = 0; j < PAQUET_SIZE; j++) {
            int16_t  p = 0;
               

                for (int k=0; k<64; k++){
                    ring_buffer_pop_64_acc(&accel_x, &p);
                    paquet[offset++]= (uint8_t)((p>>8) & 0b0000000011111111);
                    paquet[offset++]= (uint8_t)(p & 0b0000000011111111);
                }
                 for (int k=0; k<64; k++){
                    ring_buffer_pop_64_acc(&accel_y, &p);
                    paquet[offset++]= (uint8_t)((p>>8) & 0b0000000011111111);
                    paquet[offset++]= (uint8_t)(p & 0b0000000011111111);
                }
                 for (int k=0; k<64; k++){
                    ring_buffer_pop_64_acc(&accel_z, &p);
                    paquet[offset++]= (uint8_t)((p>>8) & 0b0000000011111111);
                    paquet[offset++]= (uint8_t)(p & 0b0000000011111111);
                }
                 for (int k=0; k<64; k++){
                    ring_buffer_pop_64_acc(&rot_x, &p);
                    paquet[offset++]= (uint8_t)((p>>8) & 0b0000000011111111);
                    paquet[offset++]= (uint8_t)(p & 0b0000000011111111);
                }
                 for (int k=0; k<64; k++){
                    ring_buffer_pop_64_acc(&rot_y, &p);
                    paquet[offset++]= (uint8_t)((p>>8) & 0b0000000011111111);
                    paquet[offset++]= (uint8_t)(p & 0b0000000011111111);
                }
                 for (int k=0; k<64; k++){
                    ring_buffer_pop_64_acc(&rot_z, &p);
                    paquet[offset++]= (uint8_t)((p>>8) & 0b0000000011111111);
                    paquet[offset++]= (uint8_t)(p & 0b0000000011111111);
                }
//: le côté            
                paquet[offset++]= classe;


//l'énergie (volume)
                uint8_t *e = (uint8_t*)&energie;
                    paquet[offset++]=e[3];
                    paquet[offset++]=e[2];
                    paquet[offset++]=e[1];
                    paquet[offset++]=e[0];
                         
float acceleration =ring_buffer_get_average_64(&norme_acc);
//force (le pitch)
                uint8_t *a = (uint8_t*)&acceleration;
                    paquet[offset++]=a[3];
                    paquet[offset++]=a[2];
                    paquet[offset++]=a[1];
                    paquet[offset++]=a[0];
                
// la rotation delay
float rotation =ring_buffer_get_average_64(&swift);
//force 
                uint8_t *r = (uint8_t*)&rotation;
                    paquet[offset++]=r[3];

                    paquet[offset++]=r[2];
                    paquet[offset++]=r[1];
                    paquet[offset++]=r[0];
  //L'id du device

  paquet[offset++]=DEVICE_ID;

//envoi des données inférieur à 1000 octets
            int err = sendto(sock, paquet,PAQUET_SIZE, 0,
                             (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err >= 0) paquets_par_sec++;
            else ESP_LOGE(TAG, "Erreur UDP : %d (%s)", errno, strerror(errno));


            if (err >= 0) {
             paquets_par_sec++;
                } else {
             ESP_LOGE(TAG, "Erreur UDP : %d (%s)", errno, strerror(errno));
            }
    }         





    vTaskDelay(pdMS_TO_TICKS(3));



    }

}




// ===================== Main =====================


extern "C" void app_main(void)
{

    //initialisation des ringbuffer et des filtres
    init_filtres(); 
    ring_buffer_init_4096(&fond);
    ring_buffer_init_32(&soudain);
    ring_buffer_init_128(&piezo_1);
    ring_buffer_init_128(&piezo_2);

     
    ring_buffer_init_64_acc(&accel_x);
    ring_buffer_init_64_acc(&accel_y);
    ring_buffer_init_64_acc(&accel_z);
    ring_buffer_init_64_acc(&rot_x);
    ring_buffer_init_64_acc(&rot_y);
    ring_buffer_init_64_acc(&rot_z);
    ring_buffer_init_64(&swift);
    ring_buffer_init_64(&norme_acc);

 //// Initilisation du modele cnn
    // //     ESP_LOGI("ESP-DL", "Modèle chargé avec succès !");
    // size_t model_size = modele_cnn_espdl_end - modele_cnn_espdl_start;

    // // Allocation en RAM d'un buffer aligné sur 16 octets 
    // uint8_t *model_buf_aligned = (uint8_t *)heap_caps_aligned_alloc(16, model_size, MALLOC_CAP_8BIT);

    // if (model_buf_aligned != NULL) {
    //     // Copie du modèle depuis la Flash vers la RAM alignée
    //     memcpy(model_buf_aligned, modele_cnn_espdl_start, model_size);

    //     // Instanciation de votre modèle ESP-DL à partir du buffer RAM aligné
    //     // (Remplacez MonModeleCNN par le nom de votre classe générée)
    //     model = new dl::Model((const char *)model_buf_aligned); 

    //     ESP_LOGI("CNN", "Modèle instancié avec succès sur mémoire RAM alignée (16 bytes).");
    // } else {
    //     ESP_LOGE("CNN", "Échec d'allocation mémoire pour le modèle aligné !");
    // }

   
// 1. Installer le driver USB en premier. SI MESURE TORCH
// usb_serial_jtag_driver_config_t usb_cfg = {
//     .tx_buffer_size = 2048,   // >= NB_MESURES_ENVOI*2*sizeof(float), avec marge
//     .rx_buffer_size = 256,
// };
// ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));
    // Initialisation matérielle globale
    configuration_adc_continuous();
    
  ESP_ERROR_CHECK(i2cdev_init());
MPU6050_init();
   
//DESACTIVE
esp_err_t ret = nvs_flash_init();
     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
     ESP_ERROR_CHECK(nvs_flash_erase());
     ret = nvs_flash_init();
 }
 //DESACTIVE
// ESP_ERROR_CHECK(ret); // requis par le WiFi
   //DESACTIVE
wifi_init_sta();
   
    //le socket wifi pour écrire sur le port 8000
    udp_socket_init();   // <-- ici
    if (sock < 0) { vTaskDelete(NULL); }  //
//DESACTIVE
    udp_sem = xSemaphoreCreateBinary(); //le semaphore qui passe de tache calculs à udp
       
    
    
    // Création UNIQUE de la tâche en tâche de fond (Pas de while ici !)
    xTaskCreatePinnedToCore(
        function_tache_adc,   // Fonction à exécuter
        "Tache_Lect_ADC",     // Nom pour le débogage
        8192,                 // Taille mémoire allouée (8 Ko)
        NULL,                 // Pas de paramètre
        5,                    // Priorité élevée pour éviter les surcharges DMA
        NULL,                  // Aucun handle externe requis
        1
    );
   // printf("on a créé latache adc");
  
    // usb_serial_jtag_driver_config_t usb_cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    // ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));
    //ESP_LOGI(TAG, "Application configurée. La tâche ADC tourne désormais de manière autonome.");
    //xTaskCreate(tache_accelero,  "acc_piezo", 4096, NULL, 10, &acc_task_handle);
   //desactive 
   xTaskCreatePinnedToCore(tache_udp,       "udp",       4096, NULL,  1, NULL,0);
    while (1) vTaskDelay(portMAX_DELAY);

}
