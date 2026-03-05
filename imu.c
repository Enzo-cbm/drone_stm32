/*========================CODE IMU=======================================*/

/*
 *
 * a l'aide de la fonction init_mpu le mpu est initialiser :
 *     -reset
 *     -selection calibre accel et gyro
 *     -selection filtre aux vibration
 *
 *le mpu renvoi les donne sous forme d'octet, 16,bit deux octets par donnees  un octet poids fort
 * et un faible lsb, la fonction be16 sert a faire l addition et la conversio float de ces 16bits
 *
 * 2 fonction mpu read et right servent a lire et ecrire sur le mpu
 *
 *
 * la fonction rtosinit sert a initialiser le rtos, mettre les registre imu a 0
 * indiquer qu aucune donne n a ete recu et creer une tache concernant la lecture du mpu
 *
 * la fonction startreaddma ima lance un dma pour lire ce dernier. la lecture est non bloquante et le programme principale continue.
 * une interruption sera appeler lorsque le dma sera effectue pour mettre les registre a jour.
 *
 * quand le dma recoit le 14iem octet l interruption est automatiquement declacher et appelle le callback
 *
 * le callback notifie que les donne sont prete et reveille la tache rtos, il demande eventuellement un changement de tache courante
 *
 * a la prochaine boucle la foction process_latest copiera les donnes de imu_buf dans un registre du cpu pour que ces donnees soient utiliser plus tard pour le pid
 *
 * enfin la fonction calibrer mpu sert a supprimer les offset du gyro.
 *
 *
 *
 */






#include "imu.h"
#include"stm32f4xx_hal.h"
#include "i2c.h"
#include <string.h>
#include "task.h"
#include "FreeRTOS.h"


/*============= VARIABLES PRIVEES ================*/

//facteurs de conversion des donnes
static const float  accel_scale = 1.0f/4096.0f;   //static limite la porte au fichier .c, visible uniquement dans ce fichier, inaccessible aux autres, unique, persistante (garde sa valeur entre 2 appel)
//static const float gyro_scale = 1.0f/65.5f;


#define MPU_ADDR      (0x68 << 1)
#define MPU_REG_START 0x3B
#define MPU_LEN       14


//buffer ou le dma ecrit les 14octets
static uint8_t dma_buf[MPU_LEN];


static imu_sample_t imu;    //donnees imu raw, filtre et angle


//flag mis a 1 une fois le dma termine
static volatile uint8_t rx_done = 0;  //voltile indique que la variable peut changer en dehors du flux normal du programme ici une interruption change la valeur de la variable

//tache imu reveille a la fin
static TaskHandle_t task_to_notify = NULL;
//taskHandle_t est un type freertos permet de notifier/suspendre/reveiller/changer la priorite de la tache


static volatile uint8_t in_flight = 0;
//prevention d une nouvelle dma si la precedente n est pas finie


static volatile calibration_t calib = calib_not_done;

static uint16_t calib_target = 2000;
static uint16_t calib_count = 0;




/*==================FONCTIONS DU IMU===================*/


/*------reconstruction d un entier signer a partir de 2 octet--------*/
static inline int16_t be16(const uint8_t msb, const uint8_t lsb)
{
	return (int16_t)((msb<<8) | lsb);
}




/*---Ecriture avec un bus i2c----*/
static HAL_StatusTypeDef MPU_WriteReg(uint8_t reg, uint8_t val)
{
	return HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}


/*---lecture avec un bus i2c----*/
static HAL_StatusTypeDef MPU_ReadReg(uint8_t start_reg, uint8_t *buf, uint8_t len)
{
	return HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, start_reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}


static void MPU_CalibrationNewSamples(void);





/*---initialisation mpu----*/
void init_MPU(void)
{

	//reset
	MPU_WriteReg(0x6B, 0x80);
	HAL_Delay(100);

	//wake up
	MPU_WriteReg(0x6B, 0x00);

	//selection registre 27 vitesse angulaire
	MPU_WriteReg(0x1B, 0x08);

	//selection registre 28 vitesse angulaire
	MPU_WriteReg(0x1C, 0x10);

	//filtre
	MPU_WriteReg(0x1A, 0x03);

	HAL_Delay(250); //laisse le temps au mpu de demarrer

	//ajouter un flag en cas d erreur d init led qui clignote

}



/*fonction d initialisation rtos de l imu, indique quelle tache reveiller, remet l etat interne a 0*/
void MPU_RtosInit(TaskHandle_t taskToNotify)
{
	task_to_notify  = taskToNotify ;    //mise en memoire de la tache a reveille
	memset(&imu, 0, sizeof(imu));
	//met toute la structure imu a 0 : accel_raw, gyro_raw

	rx_done = 0;
	//indique qu aucune donne n a ete recu pour l instant
}


/*
 * demarre une lecture dma non bloquante
 * retour immediat
 * la fin arrivera dans le callback
 */
BaseType_t MPU_StartReadDMA(void)
//baseType_t est un type freertos generalement un int utiliser pour : succes/echec, booleen rtos valeur : pdTRUE/pdFALSE
{
	if (in_flight) return pdFALSE;  //pas de lecture si une lecture est deja en cours


	//reinitialisation du flag de fin de reception il sera mis a 1 plus tard lors du callback
	rx_done = 0;
	in_flight = 1;   //mise en route d un nouvelle lecture


	//lecturde mpu_len = 14 octets pour le dma depuis le registre mpu_reg_start, copie dans dma_buf sans bloquer le cpu
	HAL_StatusTypeDef st = HAL_I2C_Mem_Read_DMA(&hi2c1, MPU_ADDR, MPU_REG_START, I2C_MEMADD_SIZE_8BIT, dma_buf, MPU_LEN);

	//a la fin de la dma HAL_I2C_Mem_Read_DMA lance l isr qui appelle le callback qui signale que les donne sont accessible et prete a etre copie du dma_buf au cpu

	if (st != HAL_OK) in_flight = 0;
	return (st == HAL_OK) ? pdTRUE : pdFALSE;



}



/*
 * Callback hal appele automatiquement par le hall quand une lecture i2c est termine
 */

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
/*+++++cette fonction n est jamais appele volontairement, c est la fonxction HAL au travers de l isr qui appelle le callback une fois les 14 octets recu++++++++++++*/
	//le callback est global et appele pour tour les i2c ici on verifie que cela concerne hi2c1
	if (hi2c != &hi2c1) return;


	//le flag indique que les 14 octets ont bien ete recu
	rx_done = 1;

	in_flight = 0;   //aucune lecture dma n est en cours

	if (task_to_notify) //task_to_notify = NULL si mpu_rtosinit n est pas appele on evite de notifier une tache inexistance
	{

		BaseType_t hpw = pdFALSE;
		//hpw = higher priority woken sert a savoir si la tache reveille a une priorite plus elevee que la tache actuellement interrompu

		vTaskNotifyGiveFromISR(task_to_notify, &hpw);
		//reveille une tache

		portYIELD_FROM_ISR(hpw);
		//si la tache imu a une priorite plus haute alors le freertos bascule dessus
	}
}




/*
 * appele dans la tache imu apres notification
 * convertit dma_buf -> accel, gyro dans imu
 */

void MPU_ProcessLatest(void)
{
	if (!rx_done) return;
	rx_done = 0;


	//accel
	imu.accel_raw[axis_x] = be16(dma_buf[0], dma_buf[1]);
	imu.accel_raw[axis_y] = be16(dma_buf[2], dma_buf[3]);
	imu.accel_raw[axis_z] = be16(dma_buf[4], dma_buf[5]);

	//temperature.
	imu.temperature = be16(dma_buf[6], dma_buf[7]);


	//gyro
	imu.gyro_raw[axis_x] = be16(dma_buf[8], dma_buf[9]);
	imu.gyro_raw[axis_y] = be16(dma_buf[10], dma_buf[11]);
	imu.gyro_raw[axis_z] = be16(dma_buf[12], dma_buf[13]);

	imu.accel_raw[axis_x] *= -1;
	imu.gyro_raw[axis_y]  *= -1;
	imu.gyro_raw[axis_z]  *= -1;

	//la calibration est automatiquement faite, et s arrete une fois les 2000 mesures effectuees
	MPU_CalibrationNewSamples();

}

//les donnes imu sont static ici on permet la future creation d une variable glZ
const imu_sample_t *MPU_GetSample(void)
{
    return &imu;
}

void MPU_CalibStart()
{


	for(int i = axis_x; i <= axis_z; ++i)
	{
		imu.accel_offset[i] = 0.0f;
		imu.gyro_offset[i] = 0.0f;
	}

	//initialisation de la calibration
	calib_count = 0;
	calib = calib_running;


}


static void MPU_CalibrationNewSamples(void)
{

	//securites
	if (calib != calib_running) { return; }


	if(calib_count >= calib_target)
	{

		calib = calib_done;
		return;
	}




	imu.accel_offset[axis_x] += (float)imu.accel_raw[axis_x]/(float)calib_target;
	imu.accel_offset[axis_y] += (float)imu.accel_raw[axis_y]/(float)calib_target;
	imu.accel_offset[axis_z] += (float)imu.accel_raw[axis_z]/(float)calib_target;
	imu.gyro_offset[axis_x] += (float)imu.gyro_raw[axis_x]/(float)calib_target;
	imu.gyro_offset[axis_y] += (float)imu.gyro_raw[axis_y]/(float)calib_target;
	imu.gyro_offset[axis_z] += (float)imu.gyro_raw[axis_z]/(float)calib_target;


	calib_count++;

	if(calib_count == calib_target)
	{
		imu.accel_offset[axis_z] -= 1.0f/accel_scale;
		calib = calib_done;
	}



}


/*permeT de rendre l etat de la calibration global*/
uint8_t MPU_IsCalibrated(void)
{
    return (calib == calib_done);
}


/*====================CODAGE DE LA TASK======================*/


/*
 *
 * 1 : les init mpu et rtos
 * 2 : calib start
 *
 * 3 : lecture dma
 *
 * ............attente recption des donner grace a ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
 *
 * 4 : procees latest (qui fait la caliration auto)
 *
 *
 */

























