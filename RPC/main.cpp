#include "mbed.h"
#include "math.h"
#include "accelerometer_handler.h"
#include "config.h"
#include "magic_wand_model_data.h"
#include "mbed_rpc.h"
#include "uLCD_4DGL.h"
#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"
#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"
#include "stm32l475e_iot01_accelero.h"

constexpr int kTensorArenaSize = 60 * 1024;
uint8_t tensor_arena[kTensorArenaSize];  

WiFiInterface *wifi;
InterruptIn btn(USER_BUTTON);
BufferedSerial pc(USBTX, USBRX);
uLCD_4DGL uLCD(D1, D0, D2);
DigitalOut myled1(LED1);
DigitalOut myled2(LED2);
DigitalOut myled3(LED3);
void UI_gesture(Arguments *in, Reply *out);
void angle_detection(Arguments *in, Reply *out);
void mode_change(Arguments *in, Reply *out);
int gesture(void);
void angle_initialize(void);
RPCFunction rpcUI(&UI_gesture, "UI");
RPCFunction rpcangle(&angle_detection, "AD");
RPCFunction rpcmode(&mode_change, "mode_change");
int angle[3] = {30,45,60};
int angle_select = angle[0];
int mode = 0;
int initial_x, initial_y, initial_z;
int16_t value[3] = {0};
float angle_value;
float cosine[3] = {0.866, 0.707, 0.5};
int array_index = 0;
int countt = 0;
bool tilt_success = 0;
bool over = 0;
volatile int message_num = 0;
volatile int arrivedcount = 0;
volatile bool closed = false;
MQTT::Client<MQTTNetwork, Countdown> *client2;
const char* topic = "Mbed";

Thread mqtt_thread(osPriorityHigh);
Thread mqtt_thread2(osPriorityHigh);
EventQueue mqtt_queue;
EventQueue mqtt_queue2;
EventQueue queue;
EventQueue queue2;
Thread th;
Thread th2;

void messageArrived(MQTT::MessageData& md) {
    MQTT::Message &message = md.message;
    char msg[300];
    sprintf(msg, "Message arrived: QoS%d, retained %d, dup %d, packetID %d\r\n", message.qos, message.retained, message.dup, message.id);
    printf(msg);
    ThisThread::sleep_for(1000ms);
    char payload[300];
    sprintf(payload, "Payload %.*s\r\n", message.payloadlen, (char*)message.payload);
    printf(payload);
    ++arrivedcount;
}

void publish_message(MQTT::Client<MQTTNetwork, Countdown>* client) {
    MQTT::Message message;
    char buff[100];
    if (mode == 1) {
      sprintf(buff, " 1 ");
      printf("Puslish message: select angle %d\r\n", angle_select);
    }
    if (mode == 2) {
      printf("Puslish message: over angle %d times\r\n", countt);
      sprintf(buff, " 2  %d ", countt);
    }
    message.qos = MQTT::QOS0;
    message.retained = false;
    message.dup = false;
    message.payload = (void*) buff;
    message.payloadlen = strlen(buff) + 1;
    int rc = client->publish(topic, message);
    printf("rc: %d\r\n", rc);
}

// Return the result of the last prediction
int PredictGesture(float* output) {
  // How many times the most recent gesture has been matched in a row
  static int continuous_count = 0;
  // The result of the last prediction
  static int last_predict = -1;

  // Find whichever output has a probability > 0.8 (they sum to 1)
  int this_predict = -1;
  for (int i = 0; i < label_num; i++) {
    if (output[i] > 0.8) this_predict = i;
  }

  // No gesture was detected above the threshold
  if (this_predict == -1) {
    continuous_count = 0;
    last_predict = label_num;
    return label_num;
  }

  if (last_predict == this_predict) {
    continuous_count += 1;
  } else {
    continuous_count = 0;
  }
  last_predict = this_predict;

  // If we haven't yet had enough consecutive matches for this gesture,
  // report a negative result
  if (continuous_count < config.consecutiveInferenceThresholds[this_predict]) {
    return label_num;
  }
  // Otherwise, we've seen a positive result, so clear all our variables
  // and report it
  continuous_count = 0;
  last_predict = -1;

  return this_predict;
}

int main(int argc, char* argv[])
{
  printf("Hello\n");
  uLCD.printf("\nHello\n");
  wifi = WiFiInterface::get_default_instance();
    if (!wifi) {
      printf("ERROR: No WiFiInterface found.\r\n");
      return -1;
    }


  printf("\nConnecting to %s...\r\n", MBED_CONF_APP_WIFI_SSID);
  int ret = wifi->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
  if (ret != 0) {
    printf("\nConnection error: %d\r\n", ret);
    return -1;
  }


  NetworkInterface* net = wifi;
  MQTTNetwork mqttNetwork(net);
  MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);
  client2 = &client;
  //TODO: revise host to your IP
  const char* host = "172.20.10.2";
  printf("Connecting to TCP network...\r\n");

  SocketAddress sockAddr;
  sockAddr.set_ip_address(host);
  sockAddr.set_port(1883);

  printf("address is %s/%d\r\n", (sockAddr.get_ip_address() ? sockAddr.get_ip_address() : "None"),  (sockAddr.get_port() ? sockAddr.get_port() : 0) ); //check setting
  int rc = mqttNetwork.connect(sockAddr);//(host, 1883);
  if (rc != 0) {
    printf("Connection error.");
    return -1;
  }
  printf("Successfully connected!\r\n");

  MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
  data.MQTTVersion = 3;
  data.clientID.cstring = "Mbed";

  if ((rc = client.connect(data)) != 0){
    printf("Fail to connect MQTT\r\n");
  }
  if (client.subscribe(topic, MQTT::QOS0, messageArrived) != 0){
    printf("Fail to subscribe\r\n");
  }
  mqtt_thread.start(callback(&mqtt_queue, &EventQueue::dispatch_forever));
  mqtt_thread2.start(callback(&mqtt_queue2, &EventQueue::dispatch_forever));
  btn.rise(mqtt_queue.event(&publish_message, &client));
  th.start(callback(&queue, &EventQueue::dispatch_forever));
  th2.start(callback(&queue2, &EventQueue::dispatch_forever));
  
  char buf[256], outbuf[256];

  FILE *devin = fdopen(&pc, "r");
  FILE *devout = fdopen(&pc, "w");
  BSP_ACCELERO_Init();
  mode = 0;
  while (true) {
    memset(buf, 0, 256);
    for (int i = 0; ; i++) {
      char recv = fgetc(devin);
      if (recv == '\n') {
          printf("\r\n");
          break;
      }
      buf[i] = fputc(recv, devout);
    }
    //Call the static call method on the RPC class
    RPC::call(buf, outbuf);
    printf("%s\r\n", outbuf);
    
  }
}

void UI_gesture (Arguments *in, Reply *out)
{
    printf("Enter UI_gesture mode\n");
    queue.call(&gesture);
    bool success = true;
    char buffer[200];
    char strings[30];
    sprintf(strings, "Start UI_gesture mode");
    strcpy(buffer, strings);
    if (success) {
        out->putData(buffer);
    } else {
        out->putData("Failed");
    }
}

int gesture()
{
  mode = 1;
  myled1 = 1;
  // Whether we should clear the buffer next time we fetch data
  bool should_clear_buffer = false;
  bool got_data = false;

  // The gesture index of the prediction
  int gesture_index;

  // Set up logging.
  static tflite::MicroErrorReporter micro_error_reporter;
  tflite::ErrorReporter* error_reporter = &micro_error_reporter;

  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  const tflite::Model* model = tflite::GetModel(g_magic_wand_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    error_reporter->Report(
        "Model provided is schema version %d not equal "
        "to supported version %d.",
        model->version(), TFLITE_SCHEMA_VERSION);
    return -1;
  }

  // Pull in only the operation implementations we need.
  // This relies on a complete list of all the ops needed by this graph.
  // An easier approach is to just use the AllOpsResolver, but this will
  // incur some penalty in code space for op implementations that are not
  // needed by this graph.
  static tflite::MicroOpResolver<6> micro_op_resolver;
  micro_op_resolver.AddBuiltin(
      tflite::BuiltinOperator_DEPTHWISE_CONV_2D,
      tflite::ops::micro::Register_DEPTHWISE_CONV_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_MAX_POOL_2D,
                               tflite::ops::micro::Register_MAX_POOL_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_CONV_2D,
                               tflite::ops::micro::Register_CONV_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_FULLY_CONNECTED,
                               tflite::ops::micro::Register_FULLY_CONNECTED());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_SOFTMAX,
                               tflite::ops::micro::Register_SOFTMAX());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_RESHAPE,
                               tflite::ops::micro::Register_RESHAPE(), 1);

  // Build an interpreter to run the model with
  static tflite::MicroInterpreter static_interpreter(
      model, micro_op_resolver, tensor_arena, kTensorArenaSize, error_reporter);
  tflite::MicroInterpreter* interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors
  interpreter->AllocateTensors();

  // Obtain pointer to the model's input tensor
  TfLiteTensor* model_input = interpreter->input(0);
  if ((model_input->dims->size != 4) || (model_input->dims->data[0] != 1) ||
      (model_input->dims->data[1] != config.seq_length) ||
      (model_input->dims->data[2] != kChannelNumber) ||
      (model_input->type != kTfLiteFloat32)) {
    error_reporter->Report("Bad input tensor parameters in model");
    return -1;
  }

  int input_length = model_input->bytes / sizeof(float);

  TfLiteStatus setup_status = SetupAccelerometer(error_reporter);
  if (setup_status != kTfLiteOk) {
    error_reporter->Report("Set up failed\n");
    return -1;
  }

  error_reporter->Report("Set up successful...\n");

    while(mode == 1) {
        // Attempt to read new data from the accelerometer
        got_data = ReadAccelerometer(error_reporter, model_input->data.f,
                                    input_length, should_clear_buffer);

        // If there was no new data,
        // don't try to clear the buffer again and wait until next time
        if (!got_data) {
          should_clear_buffer = false;
          continue;
        }

        // Run inference, and report any error
        TfLiteStatus invoke_status = interpreter->Invoke();
        if (invoke_status != kTfLiteOk) {
          error_reporter->Report("Invoke failed on index: %d\n", begin_index);
          continue;
        }

        // Analyze the results to obtain a prediction
        gesture_index = PredictGesture(interpreter->output(0)->data.f);

        // Clear the buffer next time we read data
        should_clear_buffer = gesture_index < label_num;

        // Produce an output
        if (gesture_index < label_num) {
            error_reporter->Report(config.output_message[gesture_index]);
        }
        uLCD.locate(1,2);
        if (gesture_index == 0) {
            angle_select = angle[0];
            array_index = 0;
        }
        else if (gesture_index == 1) {
            angle_select = angle[1];
            array_index = 1;
        }
        else if (gesture_index == 2) {
            angle_select = angle[2];
            array_index = 2;
        }
        uLCD.printf("\nSelect angle: %d\n", angle[array_index]);
    }
    return 0;
}

void angle_det(void)
{
    mode = 2;
    myled2 = 1;
    printf("Enter angle_dection mode\n");
    int16_t pDataXYZ[3] = {0};
    int k = 0;
    int16_t x, y, z;
    int16_t xtemp = 0;
    int16_t ytemp = 0;
    int16_t ztemp = 0;
    while (k < 10) {
        BSP_ACCELERO_AccGetXYZ(pDataXYZ);
        x = pDataXYZ[0];
        y = pDataXYZ[1];
        z = pDataXYZ[2];
        if (x == xtemp && y == ytemp && z == ztemp) k++;
        xtemp = x;
        ytemp = y;
        ztemp = z;
        ThisThread::sleep_for(10ms);
    }
    initial_x = x;
    initial_y = y;
    initial_z = z;
    printf("Success initialize\n");
    countt = 0;
    float result = 0;
    while(mode == 2) {
        BSP_ACCELERO_AccGetXYZ(value);
        float temp1, temp2;
        temp1 = (float)((value[0]*initial_x + value[1]*initial_y + value[2]*initial_z));
        temp2 = (sqrt((double)(value[0]*value[0]+value[1]*value[1]+value[2]*value[2]))*sqrt((double)(initial_x*initial_x+initial_y*initial_y+initial_z*initial_z)));
        angle_value = temp1 / temp2;
        result = acos (angle_value) * 180.0 / 3.14159;
        //printf("tilt angle = %f\n", result);
        uLCD.locate(1,2);
        uLCD.printf("\ntilt angle: %ff\n", result);
        if (angle_value < cosine[array_index]) {
            myled3 = 1;
            countt++;
            mqtt_queue2.call(&publish_message, client2);
        }
        else { 
          countt = 0;
          myled3 = 0;
        }
        ThisThread::sleep_for(250ms);
    }
}

void angle_detection (Arguments *in, Reply *out)
{
  queue2.call(&angle_det);
  bool success = true;
  char buffer[200];
  char strings[30];
  sprintf(strings, "Start angle_detection mode");
  strcpy(buffer, strings);
  if (success) {
    out->putData(buffer);
  } else {
    out->putData("Failed");
  }
}

void mode_change (Arguments *in, Reply *out)
{
  if (mode == 1) {
    printf("Exit UI_gesture mode\n");
    mode = 0;
    myled1 = 0;
  }
  if (mode == 2) {
    countt = 0;
    mode = 0;
    myled2 = 0;
    myled3 = 0;
    printf("Exit angle_detection mode\n");
  }
}