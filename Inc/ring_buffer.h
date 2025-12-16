#ifndef RING_BUFFER_H
#define RING_BUFFER_H

/* Include */
#include "stm32l4xx_hal.h"
/* End of Includes*/

/* Definitions */
#define UART_BUFFER_SIZE_BYTES 256
/* End of Definitions */

/* Custom Variables */
typedef struct{
    unsigned char buffer[UART_BUFFER_SIZE_BYTES];
    volatile uint8_t head;
    volatile uint8_t tail;
}ring_buffer_t;
/* End of Custom Variables */

/* Function Declarations */

/** @brief
 * 
 */
void uart_start_ring_buffer(void);

/** @brief
 * 
 */
int Uart_read(void);

/** @brief
 * 
 */
void Uart_write(int c);

/** @brief
 * 
 */
void uart_send_string(const char *s);

/** @brief 
 * 
 */
void Uart_printbase (long n, uint8_t base);

/** @brief
 * 
 */
int IsDataAvailable(void);

/** @brief
 * 
 */
int check_for (char *str, char *buffertolookinto);

/** @brief
 * 
 */
void GetDataFromBuffer (char *startString, char *endString, char *buffertocopyfrom, char *buffertocopyinto);

/** @brief
 * 
 */
void Uart_flush (void);

/** @brief 
 * 
*/
int Uart_peek();

/** @brief
 * 
 */
int Copy_upto (char *string, char *buffertocopyinto);

/** @brief
 * 
 */
int Get_after (char *string, uint8_t numberofchars, char *buffertosave);

/** @brief
 * 
 */
int uart_wait_for_line(char *string, uint16_t commandTimeout);

/** @brief
 * 
 */
void uart_ring_buffer_isr(UART_HandleTypeDef *huart);


#endif /*RING_BUFFER_H*/
