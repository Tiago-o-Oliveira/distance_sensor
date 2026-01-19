#include "ring_buffer.h"
#include <string.h>
#include <stdbool.h>

extern UART_HandleTypeDef huart2;

#define uart &huart2

#define TIMEOUT_DEF 500  // 500ms timeout

uint16_t timeout;
ring_buffer_t rx_buffer = { { 0 }, 0, 0};
ring_buffer_t tx_buffer = { { 0 }, 0, 0};
bool isBufferOverfloded = false;

ring_buffer_t *_rx_buffer;
ring_buffer_t *_tx_buffer;

void store_char(unsigned char c, ring_buffer_t *buffer);

void uart_start_ring_buffer(void){
  _rx_buffer = &rx_buffer;
  _tx_buffer = &tx_buffer;

  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  //__HAL_UART_ENABLE_IT(uart, UART_IT_ERR);

  /* Enable the UART Data Register not empty Interrupt */
  __HAL_UART_ENABLE_IT(uart, UART_IT_RXNE);
}

void store_char(unsigned char c, ring_buffer_t *buffer){
  
	int nextHeadPos = (unsigned int)(buffer->head + 1) % UART_BUFFER_SIZE_BYTES;

	// if we should be storing the received character into the location
	// just before the tail (meaning that the head would advance to the
	// current location of the tail), we're about to overflow the buffer
	// and so we don't write the character or advance the head.
	if(nextHeadPos != buffer->tail) {
		buffer->buffer[buffer->head] = c;
		buffer->head = nextHeadPos;
	}
}

/* checks, if the entered string is present in the giver buffer ?
 */
 int check_for (char *str, char *buffertolookinto)
{
	int stringlength = strlen (str);
	int bufferlength = strlen (buffertolookinto);
	int so_far = 0;
	int indx = 0;
repeat:
	while (str[so_far] != buffertolookinto[indx])
		{
			indx++;
			if (indx>stringlength) return 0;
		}
	if (str[so_far] == buffertolookinto[indx])
	{
		while (str[so_far] == buffertolookinto[indx])
		{
			so_far++;
			indx++;
		}
	}

	if (so_far == stringlength);
	else
	{
		so_far =0;
		if (indx >= bufferlength) return -1;
		goto repeat;
	}

	if (so_far == stringlength) return 1;
	else return -1;
}

int Uart_read(void){
  // if the head isn't ahead of the tail, we don't have any characters
  if(_rx_buffer->head == _rx_buffer->tail){
    return -1;
  }
  else{
    unsigned char c = _rx_buffer->buffer[_rx_buffer->tail];
    _rx_buffer->tail = (unsigned int)(_rx_buffer->tail + 1) % UART_BUFFER_SIZE_BYTES;
    return c;
  }
}

/* writes a single character to the uart and increments head
 */
void Uart_write(int c){
	if (c>=0){
		int i = (_tx_buffer->head + 1) % UART_BUFFER_SIZE_BYTES;
		
		while (i == _tx_buffer->tail);

		_tx_buffer->buffer[_tx_buffer->head] = (uint8_t)c;
		_tx_buffer->head = i;

		__HAL_UART_ENABLE_IT(uart, UART_IT_TXE); // Enable UART transmission interrupt
	}
}

/* checks if the new data is available in the incoming buffer
 */
int IsDataAvailable(void){
	return (uint16_t)((_rx_buffer->head - _rx_buffer->tail) & (UART_BUFFER_SIZE_BYTES - 1));
}

/* sends the string to the uart
 */
void uart_send_string(const char *s){
	//while(*s) Uart_write(*s++);

}

void GetDataFromBuffer (char *startString, char *endString, char *buffertocopyfrom, char *buffertocopyinto)
{
	int startStringLength = strlen (startString);
	int endStringLength   = strlen (endString);
	int so_far = 0;
	int indx = 0;
	int startposition = 0;
	int endposition = 0;

repeat1:
	while (startString[so_far] != buffertocopyfrom[indx]) indx++;
	if (startString[so_far] == buffertocopyfrom[indx])
	{
		while (startString[so_far] == buffertocopyfrom[indx])
		{
			so_far++;
			indx++;
		}
	}

	if (so_far == startStringLength) startposition = indx;
	else
	{
		so_far =0;
		goto repeat1;
	}

	so_far = 0;

repeat2:
	while (endString[so_far] != buffertocopyfrom[indx]) indx++;
	if (endString[so_far] == buffertocopyfrom[indx])
	{
		while (endString[so_far] == buffertocopyfrom[indx])
		{
			so_far++;
			indx++;
		}
	}

	if (so_far == endStringLength) endposition = indx-endStringLength;
	else
	{
		so_far =0;
		goto repeat2;
	}

	so_far = 0;
	indx=0;

	for (int i=startposition; i<endposition; i++)
	{
		buffertocopyinto[indx] = buffertocopyfrom[i];
		indx++;
	}
}

void Uart_flush (void)
{
	_rx_buffer->head = 0;
	_rx_buffer->tail = 0;
}

int Uart_peek(){
  if(_rx_buffer->head == _rx_buffer->tail){
    return -1;
  }else{
    return _rx_buffer->buffer[_rx_buffer->tail];
  }
}

/* copies the data from the incoming buffer into our buffer
 * Must be used if you are sure that the data is being received
 * it will copy irrespective of, if the end string is there or not
 * if the end string gets copied, it returns 1 or else 0
 * Use it either after (IsDataAvailable) or after (Wait_for) functions
 */
int Copy_upto (char *string, char *buffertocopyinto){
	int so_far =0 ;
	int len = strlen (string);
	int indx = 0;

again:
	while (Uart_peek() != string[so_far]){
			buffertocopyinto[indx] = _rx_buffer->buffer[_rx_buffer->tail];
			_rx_buffer->tail = (unsigned int)(_rx_buffer->tail + 1) % UART_BUFFER_SIZE_BYTES;
			indx++;
			while (!IsDataAvailable());

		}
	while (Uart_peek() == string [so_far]){
		so_far++;
		buffertocopyinto[indx++] = Uart_read();
		if (so_far == len) return 1;
		timeout = TIMEOUT_DEF;
		while ((!IsDataAvailable())&&timeout);
		if (timeout == 0) return 0;
	}

	if (so_far != len){
		so_far = 0;
		goto again;
	}

	if (so_far == len) return 1;
	else return 0;
}

/* must be used after wait_for function
 * get the entered number of characters after the entered string
 */
int Get_after (char *string, uint8_t numberofchars, char *buffertosave)
{
	for (int indx=0; indx<numberofchars; indx++)
	{
		timeout = TIMEOUT_DEF;
		while ((!IsDataAvailable())&&timeout);  // wait until some data is available
		if (timeout == 0) return 0;  // if data isn't available within time, then return 0
		buffertosave[indx] = Uart_read();  // save the data into the buffer... increments the tail
	}
	return 1;
}

/* Waits for a particular string to arrive in the incoming buffer... It also increments the tail
 * returns 1, if the string is detected
 */
// added timeout feature so the function won't block the processing of the other functions
int uart_wait_for_line(char *string, uint16_t commandTimeout){
	int so_far = 0;
	int len = strlen (string);

again:
	timeout = commandTimeout;

	while ((!IsDataAvailable()) && timeout);  // let's wait for the data to show up

	if (timeout == 0){ 
        return 0;
    } 

	while (Uart_peek() != string[so_far]){
		if (_rx_buffer->tail != _rx_buffer->head){
			_rx_buffer->tail = (unsigned int)(_rx_buffer->tail + 1) % UART_BUFFER_SIZE_BYTES;  // increment the tail
		}
		else{
			return 0;
		}
	}


	while (Uart_peek() == string [so_far]){
		// now we will peek for the other letters too
		so_far++;
		_rx_buffer->tail = (unsigned int)(_rx_buffer->tail + 1) % UART_BUFFER_SIZE_BYTES;  // increment the tail
		
        if (so_far == len){
            return 1;
        }

		timeout = commandTimeout;
		while ((!IsDataAvailable())&&timeout);
		
        if (timeout == 0) {
            return 0;
        }
	
    }

	if (so_far != len){
		so_far = 0;
		goto again;/* 0_o    o_0    0_0 */
	}

	if (so_far == len) return 1;
	else return 0;
}

void uart_ring_buffer_isr(UART_HandleTypeDef *huart){
	   uint32_t isrflags   = READ_REG(huart->Instance->ISR);
	  uint32_t cr1its     = READ_REG(huart->Instance->CR1);

    /* if DR is not empty and the Rx Int is enabled */
    if (((isrflags & USART_ISR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
    {
    	 /******************
    	    	      *  @note   PE (Parity error), FE (Framing error), NE (Noise error), ORE (Overrun
    	    	      *          error) and IDLE (Idle line detected) flags are cleared by software
    	    	      *          sequence: a read operation to USART_SR register followed by a read
    	    	      *          operation to USART_DR register.
    	    	      * @note   RXNE flag can be also cleared by a read to the USART_DR register.
    	    	      * @note   TC flag can be also cleared by software sequence: a read operation to
    	    	      *          USART_SR register followed by a write operation to USART_DR register.
    	    	      * @note   TXE flag is cleared only by a write to the USART_DR register.

    	 *********************/
    	huart->Instance->ISR;                       /* Read status register */
        unsigned char c = huart->Instance->RDR;     /* Read data register */
        store_char (c, _rx_buffer);  // store data in buffer
        return;
    }

    /*If interrupt is caused due to Transmit Data Register Empty */
    if (((isrflags & USART_ISR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET))
    {
    	if(tx_buffer.head == tx_buffer.tail)
    	    {
    	      // Buffer empty, so disable interrupts
    	      __HAL_UART_DISABLE_IT(huart, UART_IT_TXE);

    	    }

    	 else
    	    {
    	      // There is more data in the output buffer. Send the next byte
    	      unsigned char c = tx_buffer.buffer[tx_buffer.tail];
    	      tx_buffer.tail = (tx_buffer.tail + 1) % UART_BUFFER_SIZE_BYTES;

    	      /******************
    	      *  @note   PE (Parity error), FE (Framing error), NE (Noise error), ORE (Overrun
    	      *          error) and IDLE (Idle line detected) flags are cleared by software
    	      *          sequence: a read operation to USART_SR register followed by a read
    	      *          operation to USART_DR register.
    	      * @note   RXNE flag can be also cleared by a read to the USART_DR register.
    	      * @note   TC flag can be also cleared by software sequence: a read operation to
    	      *          USART_SR register followed by a write operation to USART_DR register.
    	      * @note   TXE flag is cleared only by a write to the USART_DR register.

    	      *********************/

    	      huart->Instance->ISR;
    	      huart->Instance->TDR = c;

    	    }
    	return;
    }
}


/** Rebuild **/




bool ring_buffer_is_not_empty(void){
	return (uint16_t)((_rx_buffer->head - _rx_buffer->tail) & (UART_BUFFER_SIZE_BYTES - 1));
}

static inline bool ring_buffer_get_byte(uint8_t *byteData){
	uint16_t actualTail = _rx_buffer->tail;


	if (actualTail == _rx_buffer->head){
		return false;
	}

	*byteData = _rx_buffer->buffer[_rx_buffer->tail];;

	_rx_buffer->tail = (uint16_t)(_rx_buffer->tail + 1u) & (UART_BUFFER_SIZE_BYTES-1);
	return true;
}

void ring_buffer_put_byte(uint8_t byteToBuffer){
	uint16_t nextPositiontoWrite = (uint16_t)(_rx_buffer->head + 1u) & (UART_BUFFER_SIZE_BYTES-1);

	if(nextPositiontoWrite != _rx_buffer->tail){
		_rx_buffer->buffer[_rx_buffer->head] = byteToBuffer;
		_rx_buffer->head = nextPositiontoWrite;

	}else{
		_rx_buffer->tail = (uint16_t)(_rx_buffer->tail + 1u) & (UART_BUFFER_SIZE_BYTES-1);
		_rx_buffer->buffer[_rx_buffer->head] = byteToBuffer;
		_rx_buffer->head = nextPositiontoWrite;
		isBufferOverfloded = true;// Now it's just for control purpose
	}
}

static inline void ring_buffer_stream_matcher(uint8_t byteToTest, const uint8_t *stringToWait, uint16_t stringLength, uint16_t *bytesMatched){
    if (stringLength == 0){
    	*bytesMatched = 0;
    	return;
    }

    if (byteToTest == stringToWait[*bytesMatched]) {
        (*bytesMatched)++;
        return;
    }

    if (byteToTest == stringToWait[0]) {
        *bytesMatched = 1u;
    } else {
        *bytesMatched = 0u;
    }

}

uint8_t uart_wait_for_string(const uint8_t *stringToWait, uint16_t timeout_ms){

    const uint32_t start = HAL_GetTick();

    uint16_t stringLength = strlen((const char *)stringToWait);

    uint16_t bytesMatched = 0;

    isBufferOverfloded = false;

    for (;;) {
        uint8_t byteToTest;

        bool gotAnyBytes = false;

        while (ring_buffer_get_byte(&byteToTest)) {
        	gotAnyBytes = true;

        	ring_buffer_stream_matcher(byteToTest, stringToWait, stringLength, &bytesMatched);

            if (bytesMatched == stringLength) {
                return true;
            }
        }
        /*
        if (isBufferOverfloded) {
            return false;
        }
*/
        if ((HAL_GetTick() - start) >= timeout_ms) {
            return false;
        }

        if (!gotAnyBytes) {
            __WFI();
        }
    }
}

void ring_buffer_uart_isr(UART_HandleTypeDef *huart){
	uint32_t uartIsrFlags = READ_REG(huart->Instance->ISR);
	uint32_t control1Configs = READ_REG(huart->Instance->CR1);

	if (uartIsrFlags & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) {
		huart->Instance->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
	}


	if (((uartIsrFlags & USART_ISR_RXNE) != RESET) && ((control1Configs & USART_CR1_RXNEIE) != RESET)){
		huart->Instance->ISR;

		uint8_t receivedByte = huart->Instance->RDR;
		ring_buffer_put_byte(receivedByte);
		return;
	}
}

void ring_buffer_flush_buffer(void){
	memset(_rx_buffer->buffer,'\0', UART_BUFFER_SIZE_BYTES);
	_rx_buffer->head = 0;
	_rx_buffer->tail = 0;
}


