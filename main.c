/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* USER CODE BEGIN PD */
typedef struct {
    float x; // roll
    float y; // pitch
    float z; // yaw
} mpu;

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define BETA 0.1f

typedef struct {
    float throttle; // monte
    float roll;     // ->,<-
    float pitch;    // avant,arriere
    float yaw;      // rotation
} pidout;

typedef struct {
    float kp, ki, kd;
    float errure;
    float integral;
} pid;

#define DWT_CTRL   (*(volatile uint32_t*)0xE0001000)
#define DWT_CYCCNT (*(volatile uint32_t*)0xE0001004)
#define DEM_CR     (*(volatile uint32_t*)0xE000EDFC)
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

/* FreeRTOS objects */
osThreadId_t TaskMPUHandle;
const osThreadAttr_t TaskMPU_attributes = {
    .name = "TaskMPU",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityHigh,
};

osThreadId_t TaskPIDHandle;
const osThreadAttr_t TaskPID_attributes = {
    .name = "TaskPID",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityHigh,
};

osThreadId_t TaskMotourHandle;
const osThreadAttr_t TaskMotour_attributes = {
    .name = "TaskMotour",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityHigh,
};

osThreadId_t TaskUARTHandle;
const osThreadAttr_t TaskUART_attributes = {
    .name = "TaskUART",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityHigh,  //  même priorité que les autres
};

osMessageQueueId_t QueueMPUHandle;
const osMessageQueueAttr_t QueueMPU_attributes = { .name = "QueueMPU" };

osMessageQueueId_t QueuepidHandle;
const osMessageQueueAttr_t Queuepid_attributes = { .name = "Queuepid" };

osMutexId_t MutexconsigneHandle;
const osMutexAttr_t Mutexconsigne_attributes = { .name = "Mutexconsigne" };

osMutexId_t MutexuartHandle;
const osMutexAttr_t Mutexuart_attributes = { .name = "Mutexuart" };

/* USER CODE BEGIN PV */
// Consignes pilote (partagées entre TaskUART et TaskPID)
volatile float g_thro  = 0.0f;
volatile float g_pitch = 0.0f;
volatile float g_yaw   = 0.0f;

// Offsets gyroscope calibration
float offset_x = 0.0f;
float offset_y = 0.0f;
float offset_z = 0.0f;

// Offsets angles (position repos)
float angle_offset_x = 0.0f;
float angle_offset_y = 0.0f;

// dt et DWT
#define dt    0.01f
#define ALPHA 0.95f
uint32_t last_time = 0;
float dt_reel = 0.01f;

// Quaternion Madgwick
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// PID controllers
pid pid_roll  = {260.0f, 0.0f, 180.0f, 0.0f, 0.0f};
pid pid_pitch = {260.0f, 0.0f, 180.0f, 0.0f, 0.0f};
pid pid_yaw   = {100.0f, 0.0f,  50.0f, 0.0f, 0.0f};
/* USER CODE END PV */

/* Function prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
void Taskmpu(void *argument);
void Taskpid(void *argument);
void Taskmotour(void *argument);
void Taskuart(void *argument);

/* USER CODE BEGIN PFP */
void affiche_boot(char *msg);   //  sans mutex — pour main() avant kernel
void affiche(char *msg);        // avec mutex — pour les tâches FreeRTOS
void lire_cap(void);
void init_cap(void);
void calibration(void);
void recup_val(mpu *cap);
void Madgwick_Update(float gx, float gy, float gz, float ax, float ay, float az);
void Madgwick_GetAngles(float *roll, float *pitch, float *yaw);
float pid_calcul(pid *p, float consigne, float mesure);
uint32_t throttle_to_pwm(float throttle);
/* USER CODE END PFP */

/* ==========================================================================
 * main
 * ========================================================================== */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();

    /* USER CODE BEGIN 2 */
    // Utilise affiche_boot (pas de mutex) avant osKernelStart
    affiche_boot(" <== demarrage ==>\r\n");
    lire_cap();

    // Active le compteur de cycles DWT
    DEM_CR    |= 0x01000000;
    DWT_CTRL  |= 0x00000001;
    DWT_CYCCNT = 0;

    // Arme les ESC : signal 500us (min) pendant 3s

    affiche_boot("PWM demarre\r\n");
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    affiche("Armement ESC attend \r\n");

    // Étape 1 — signal MAX
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1000);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1000);
    HAL_Delay(2000);

    // Étape 2 — signal MIN
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 500);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 500);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 500);
    HAL_Delay(2000);

    affiche("ESC armes \r\n");
    /* USER CODE END 2 */

    /* Init scheduler */
    osKernelInitialize();

    /* Mutex */
    MutexconsigneHandle = osMutexNew(&Mutexconsigne_attributes);
    MutexuartHandle     = osMutexNew(&Mutexuart_attributes);

    /* Queues */
    QueueMPUHandle = osMessageQueueNew(12, sizeof(mpu),    &QueueMPU_attributes);
    QueuepidHandle = osMessageQueueNew(4,  sizeof(pidout), &Queuepid_attributes);

    /* Threads */
    TaskMPUHandle    = osThreadNew(Taskmpu,    NULL, &TaskMPU_attributes);
    TaskPIDHandle    = osThreadNew(Taskpid,    NULL, &TaskPID_attributes);
    TaskMotourHandle = osThreadNew(Taskmotour, NULL, &TaskMotour_attributes);
    TaskUARTHandle   = osThreadNew(Taskuart,   NULL, &TaskUART_attributes);

    /* Start scheduler */
    osKernelStart();

    while (1) {}
}

/* ==========================================================================
 * Fonctions utilitaires
 * ========================================================================== */

/**
 *  Affichage SANS mutex — à utiliser AVANT osKernelStart (dans main)
 */
void affiche_boot(char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 200);
}

/**
 *  Affichage AVEC mutex — à utiliser dans les tâches FreeRTOS
 */
void affiche(char *msg)
{
    osMutexAcquire(MutexuartHandle, osWaitForever);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 200);
    osMutexRelease(MutexuartHandle);
}

/**
 * Scan I2C — appelé au boot, utilise affiche_boot
 */
void lire_cap(void)
{
    char tab[64];
    uint8_t etat = 0;
    affiche_boot("<== test i2c ==> \r\n");
    for (uint8_t i = 0x08; i < 0x78; i++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c1, i << 1, 2, 10) == HAL_OK)
        {
            sprintf(tab, " trouve 0x%02X\r\n", i);
            affiche_boot(tab);
            etat = 1;
        }
    }
    if (!etat) affiche_boot("aucun peripherique I2C !\r\n");
    else       affiche_boot("peripherique I2C OK\r\n");
}

/**
 *  Init MPU6050 — appelée dans TaskMPU (après kernel)
 */
void init_cap(void)
{
    uint8_t val = 0x00;
    // Wake up MPU6050
    HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0x6B, I2C_MEMADD_SIZE_8BIT, &val, 1, 1000);
    HAL_Delay(100);
    // Gyro full scale ±250°/s
    HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0x1B, I2C_MEMADD_SIZE_8BIT, &val, 1, 1000);
    affiche("MPU6050 initialise\r\n");
}

/**
 *  Calibration gyro + offset angles
 */
void calibration(void)
{
    float sx = 0.0f, sy = 0.0f, sz = 0.0f;
    uint8_t tab[14];
    char buf[64];

    affiche("Calibration... ne pas bouger !\r\n");

    for (int i = 0; i < 200; i++) {
        HAL_I2C_Mem_Read(&hi2c1, 0xD0, 0x3B,
            I2C_MEMADD_SIZE_8BIT, tab, 14, 1000);
        int16_t gx = (int16_t)(tab[8]  << 8 | tab[9]);
        int16_t gy = (int16_t)(tab[10] << 8 | tab[11]);
        int16_t gz = (int16_t)(tab[12] << 8 | tab[13]);
        sx += gx / 131.0f;
        sy += gy / 131.0f;
        sz += gz / 131.0f;
        HAL_Delay(10);
    }

    offset_x = sx / 200.0f;
    offset_y = sy / 200.0f;
    offset_z = sz / 200.0f;

    sprintf(buf, "Offset gyro X:%.2f Y:%.2f Z:%.2f\r\n",
        offset_x, offset_y, offset_z);
    affiche(buf);

    // Stabilisation filtre Madgwick
    affiche("Stabilisation Madgwick...\r\n");
    mpu cap_init;
    for (int i = 0; i < 100; i++) {
        recup_val(&cap_init);
        HAL_Delay(10);
    }

    // Offset position repos
    angle_offset_x = cap_init.x;
    angle_offset_y = cap_init.y;

    sprintf(buf, "Angle offset X:%.2f Y:%.2f\r\n",
        angle_offset_x, angle_offset_y);
    affiche(buf);

    // Convergence finale
    affiche("Convergence finale...\r\n");
    mpu cap_temp;
    for (int i = 0; i < 500; i++) {
        recup_val(&cap_temp);
        HAL_Delay(10);
    }
    affiche("Calibration OK ! Pret.\r\n");
}

/**
 *  Lit le MPU6050, calcule dt réel, appelle Madgwick, retourne angles corrigés
 */
void recup_val(mpu *cap)
{
    uint8_t tab[14];

    uint32_t now = DWT_CYCCNT;
    dt_reel = (float)(now - last_time) / 180000000.0f;
    last_time = now;
    if (dt_reel < 0.001f) dt_reel = 0.001f;
    if (dt_reel > 0.1f)   dt_reel = 0.01f;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        &hi2c1, 0xD0, 0x3B,
        I2C_MEMADD_SIZE_8BIT, tab, 14, 1000);

    if (status != HAL_OK) {
        affiche("I2C erreur !\r\n");
        osMutexAcquire(MutexconsigneHandle, osWaitForever);
            g_thro  = 0.0f;
            g_pitch = 0.0f;
            g_yaw   = 0.0f;
            osMutexRelease(MutexconsigneHandle);

            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 500);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 500);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 500);
            return;
    }

    // Accelerometre (g)
    int16_t ax_raw = (int16_t)(tab[0] << 8 | tab[1]);
    int16_t ay_raw = (int16_t)(tab[2] << 8 | tab[3]);
    int16_t az_raw = (int16_t)(tab[4] << 8 | tab[5]);
    float Ax = ax_raw / 16384.0f;
    float Ay = ay_raw / 16384.0f;
    float Az = az_raw / 16384.0f;

    // Gyroscope (deg/s) - offset appliqué
    int16_t gx_raw = (int16_t)(tab[8]  << 8 | tab[9]);
    int16_t gy_raw = (int16_t)(tab[10] << 8 | tab[11]);
    int16_t gz_raw = (int16_t)(tab[12] << 8 | tab[13]);
    float Gx = (gx_raw / 131.0f) - offset_x;
    float Gy = (gy_raw / 131.0f) - offset_y;
    float Gz = (gz_raw / 131.0f) - offset_z;

    // Zone morte yaw
    if (fabsf(Gz) < 1.0f) Gz = 0.0f;

    // Filtre Madgwick
    Madgwick_Update(Gx, Gy, Gz, Ax, Ay, Az);

    // Angles Euler
    float roll, pitch, yaw;
    Madgwick_GetAngles(&roll, &pitch, &yaw);
    cap->x = pitch - angle_offset_x;  // ← échangé
    cap->y = roll  - angle_offset_y;  // ← échangé
    cap->z = yaw;
}

/**
 *  Calcul PID
 */
float pid_calcul(pid *p, float consigne, float mesure)
{
    float e = consigne - mesure;
    float P = p->kp * e;

    p->integral += e * 0.01f;
    if (p->integral >  10.0f) p->integral =  10.0f;
    if (p->integral < -10.0f) p->integral = -10.0f;
    float I = p->ki * p->integral;

    float D = p->kd * (e - p->errure) / 0.01f;
    p->errure = e;

    float correction = P + I + D;
    if (correction >  100.0f) correction =  100.0f;
    if (correction < -100.0f) correction = -100.0f;
    return correction;
}

/**
 *  Convertit throttle 0-100% en valeur CCR (500-1000 pour ESC standard)
 */
uint32_t throttle_to_pwm(float throttle)
{
    if (throttle < 0.0f)   throttle = 0.0f;
    if (throttle > 100.0f) throttle = 100.0f;
    return (uint32_t)(500.0f + (throttle * 4.5f));
}

/**
 *  Madgwick AHRS update
 */
void Madgwick_Update(float gx, float gy, float gz,
                     float ax, float ay, float az)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3;
    float _4q0, _4q1, _4q2;
    float _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;

    // Gyro deg/s -> rad/s
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    // Derivee quaternion depuis gyro
    qDot1 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    qDot2 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    qDot3 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    qDot4 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    // Normalise accel
    recipNorm = 1.0f / sqrtf(ax*ax + ay*ay + az*az);
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    _2q0 = 2.0f*q0; _2q1 = 2.0f*q1; _2q2 = 2.0f*q2; _2q3 = 2.0f*q3;
    _4q0 = 4.0f*q0; _4q1 = 4.0f*q1; _4q2 = 4.0f*q2;
    _8q1 = 8.0f*q1; _8q2 = 8.0f*q2;
    q0q0 = q0*q0; q1q1 = q1*q1; q2q2 = q2*q2; q3q3 = q3*q3;

    s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
    s1 = _4q1*q3q3 - _2q3*ax + 4.0f*(q0q0*q1)
       - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
    s2 = 4.0f*(q0q0*q2) + _2q0*ax + _4q2*q3q3
       - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
    s3 = 4.0f*(q1q1*q3) - _2q1*ax + 4.0f*(q2q2*q3) - _2q2*ay;

    recipNorm = 1.0f / sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

    qDot1 -= BETA * s0;
    qDot2 -= BETA * s1;
    qDot3 -= BETA * s2;
    qDot4 -= BETA * s3;

    q0 += qDot1 * dt_reel;
    q1 += qDot2 * dt_reel;
    q2 += qDot3 * dt_reel;
    q3 += qDot4 * dt_reel;

    recipNorm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

/**
 *  Convertit quaternion en angles Euler (degres)
 */
void Madgwick_GetAngles(float *roll, float *pitch, float *yaw)
{
    *roll  = atan2f(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * 57.2958f;
    *pitch = asinf (2*(q0*q2 - q3*q1))                         * 57.2958f;
    *yaw   = atan2f(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3)) * 57.2958f;
}

/* 
 * TÂCHES FreeRTOS
 *  */

/**
 *  TaskMPU — init, calibration, lecture MPU6050 @ 100ms
 */
void Taskmpu(void *argument)
{
    mpu cap;
    char buf[64];
    static uint8_t cpt = 0;

    init_cap();
    calibration();
    affiche("Systeme pret !\r\n");

    for (;;)
    {
        recup_val(&cap);

        // Affiche 1 fois sur 10 (toutes les secondes) pour ne pas saturer l'UART
        if (++cpt >= 10) {
            cpt = 0;
            sprintf(buf, "X:%.2f Y:%.2f Z:%.2f\r\n", cap.x, cap.y, cap.z);
            affiche(buf);
        }

        osMessageQueuePut(QueueMPUHandle, &cap, 0, 0);
        osDelay(100);
    }
}

/**
 *  TaskPID — lit angles, calcule PID, envoie vers moteurs
 */
void Taskpid(void *argument)
{
    mpu cap;
    pidout p;
    char buf[128];

    for (;;)
    {
        if (osMessageQueueGet(QueueMPUHandle, &cap, NULL, 100) == osOK)
        {
            // Lecture consignes (protégée)
            osMutexAcquire(MutexconsigneHandle, osWaitForever);
            float cons_throttle = g_thro;
            float cons_pitch    = g_pitch;
            float cons_yaw      = g_yaw;
            osMutexRelease(MutexconsigneHandle);

            // Calcul PID
            p.throttle = cons_throttle;
            p.roll     = pid_calcul(&pid_roll,  0.0f,       cap.x);
            p.pitch    = pid_calcul(&pid_pitch, cons_pitch,  cap.y);
            p.yaw      = pid_calcul(&pid_yaw,   cons_yaw,    cap.z);

            sprintf(buf, "THR:%.1f ROL:%.2f PIT:%.2f YAW:%.2f\r\n",
                p.throttle, p.roll, p.pitch, p.yaw);
            affiche(buf);

            osMessageQueuePut(QueuepidHandle, &p, 0, 0);
        }
        osDelay(10);
    }
}

/**
 *  TaskMoteur — applique les PWM sur les 4 moteurs
 *
 *  Disposition quadcopter + (vue dessus) :
 *        M1 (avant-gauche)   M2 (avant-droit)
 *        M3 (arriere-gauche) M4 (arriere-droit)
 *
 *  Sens de rotation (anti-couple) :
 *        M1 CCW (+yaw)   M2 CW  (-yaw)
 *        M3 CW  (-yaw)   M4 CCW (+yaw)
 */
void Taskmotour(void *argument)
{
    uint32_t m1, m2, m3, m4;
    pidout p;
    char buf[128];

    // Demarre les 4 canaux PWM


    for (;;)
    {
        if (osMessageQueueGet(QueuepidHandle, &p, NULL, osWaitForever) == osOK)
        {
            float base = p.throttle;

            m1 = throttle_to_pwm(base + p.roll + p.pitch - p.yaw);
            m2 = throttle_to_pwm(base - p.roll + p.pitch + p.yaw);
            m3 = throttle_to_pwm(base + p.roll - p.pitch + p.yaw);
            m4 = throttle_to_pwm(base - p.roll - p.pitch - p.yaw);

            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, m1);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, m2);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, m3);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, m4);

            //  Ce sprintf est maintenant DANS la boucle
            sprintf(buf, "M1:%lu M2:%lu M3:%lu M4:%lu\r\n", m1, m2, m3, m4);
            affiche(buf);
        }
    }
}

/**
 *  TaskUART — reception commandes clavier depuis Putty
 *
 *  z/Z : throttle +2%     s/S : throttle -2%
 *  a/A : pitch +1°        e/E : pitch -1°
 *  q/Q : yaw -1°          d/D : yaw +1°
 *  ESPACE : arret urgence (tout a zero)
 */
void Taskuart(void *argument)
{
    uint8_t c;
    char buf[64];

    for (;;)
    {
        //  HAL_UART_Receive avec timeout court — non bloquant pour FreeRTOS
        if (HAL_UART_Receive(&huart2, &c, 1, 10) == HAL_OK)
        {
            osMutexAcquire(MutexconsigneHandle, osWaitForever);

            switch (c)
            {
                case 'z': case 'Z':
                    g_thro += 2.0f;
                    if (g_thro > 500.0f) g_thro = 500.0f;
                    break;
                case 's': case 'S':
                    g_thro -= 4.0f;
                    if (g_thro < 0.0f) g_thro = 0.0f;
                    break;
                case 'q': case 'Q':
                    g_yaw -= 4.0f;
                    break;
                case 'd': case 'D':
                    g_yaw += 4.0f;
                    break;
                case 'a': case 'A':
                    g_pitch += 4.0f;
                    break;
                case 'e': case 'E':
                    g_pitch -= 4.0f;
                    break;
                case ' ':
                    g_thro  = 0.0f;
                    g_pitch = 0.0f;
                    g_yaw   = 0.0f;
                    break;
                default:
                    break;
            }

            // Copie locale avant release mutex
            float t = g_thro;
            float pi = g_pitch;
            float ya = g_yaw;
            osMutexRelease(MutexconsigneHandle);

            //   confirme que la touche a ete recue
            sprintf(buf, ">> THR:%.1f PIT:%.1f YAW:%.1f\r\n", t, pi, ya);
            affiche(buf);
        }

        osDelay(5); //  libere le CPU entre chaque poll
    }
}

/* ==========================================================================
 * Configurations peripheriques (generees par CubeMX)
 * ========================================================================== */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 8;
    RCC_OscInitStruct.PLL.PLLN            = 90;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = 2;
    RCC_OscInitStruct.PLL.PLLR            = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    if (HAL_PWREx_EnableOverDrive() != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 100000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}

static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig     = {0};
    TIM_OC_InitTypeDef sConfigOC              = {0};

    // Fclk TIM2 = APB1 * 2 = 45MHz * 2 = 90MHz
    // Prescaler 89 -> Ftick = 90MHz / 90 = 1MHz -> periode 1us
    // Period 19999 -> periode PWM = 20000us = 20ms = 50Hz   ESC standard
    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 179;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 19999;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim2);
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin  = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = LD2_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

/* ==========================================================================
 * Error Handler
 * ========================================================================== */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    // printf("Assert: file %s line %d\r\n", file, line);
}
#endif
