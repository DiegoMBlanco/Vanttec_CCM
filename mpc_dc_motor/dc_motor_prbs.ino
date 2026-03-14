#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <math.h>

// =====================
// AJUSTA AQUÍ
// =====================
#define MOTOR_VERSION_140RPM   1
#if MOTOR_VERSION_140RPM
  //#define PPR        494.0f
  #define PPR 44.5f 
  #define GEAR_RATIO  45.0f
#else
  #define PPR        341.0f
  #define GEAR_RATIO  26.0f
#endif

#define TS 0.2f   // 200ms — suficiente para que el motor responda

// =====================
// ROS OBJECTS
// =====================
rcl_node_t node;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_publisher_t pwm_pub;
rcl_publisher_t speed_pub;
rcl_timer_t timer;
std_msgs__msg__Float32 pwm_msg;
std_msgs__msg__Float32 speed_msg;

// =====================
// PWM CONFIG
// =====================
#define PWM_PIN  22
#define IN1_PIN  19
#define IN2_PIN  18
#define PWM_CHNL  0
#define PWM_FRQ  1000
#define PWM_RES   8

// =====================
// ENCODER
// =====================
#define ENC_A 25
#define ENC_B 26
volatile long encoder_count = 0;

// =====================
// PRBS
// =====================
uint16_t lfsr = 0xACE1u;
float prbs_amplitude = 0.6f;  // rango: offset ~ offset+amplitude
float prbs_offset    = 0.3f;  // mínimo PWM para evitar zona muerta

// =====================
// ENCODER ISR
// =====================
void IRAM_ATTR encoder_isr() {
  encoder_count++;  // siempre positivo (motor gira en una sola dirección)
}

// =====================
// PRBS GENERATOR
// =====================
float generate_prbs() {
  unsigned lsb = lfsr & 1;
  lfsr >>= 1;
  if (lsb) lfsr ^= 0xB400u;
  float bit = (float)(lfsr & 1);
  return prbs_offset + prbs_amplitude * bit;  // retorna 0.4 o 0.9
}

// =====================
// TIMER CALLBACK
// =====================
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  if (timer == NULL) return;

  // 1) Generar PRBS y aplicar PWM
  float pwm_value = generate_prbs();
  //float pwm_value = 0.9f;
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  uint32_t duty = (uint32_t)((pow(2, PWM_RES) - 1) * pwm_value);
  ledcWrite(PWM_CHNL, duty);

  // 2) Calcular velocidad del eje de SALIDA (después de reductora)
  static long prev_count = 0;
  long current_count = encoder_count;
  long delta = current_count - prev_count;
  prev_count = current_count;

  float speed_rps_motor  = (float)delta / (PPR * TS);         // eje del motor
  float speed_rps_output = speed_rps_motor / GEAR_RATIO;       // eje de salida
  float speed_rad_s      = speed_rps_output * 2.0f * M_PI;    // rad/s

  // 3) Publicar
  pwm_msg.data   = pwm_value;
  speed_msg.data = speed_rad_s;
  rcl_publish(&pwm_pub, &pwm_msg, NULL);
  rcl_publish(&speed_pub, &speed_msg, NULL);
}

// =====================
// SETUP
// =====================
void setup() {
  set_microros_transports();

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(ENC_A,   INPUT);
  pinMode(ENC_B,   INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_A), encoder_isr, RISING);
  ledcSetup(PWM_CHNL, PWM_FRQ, PWM_RES);
  ledcAttachPin(PWM_PIN, PWM_CHNL);

  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "motor_prbs_node", "", &support);

  rclc_publisher_init_default(
    &pwm_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/motor_pwm");

  rclc_publisher_init_default(
    &speed_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/motor_speed");

  rclc_timer_init_default(
    &timer, &support,
    RCL_MS_TO_NS(200),   // 200ms
    timer_callback);

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);
}

// =====================
// LOOP
// =====================
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
}
