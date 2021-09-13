#include "bsp_usart.h"

#if UART1_FIFO_EN == 1
static UART_T g_tUart1;
static uint8_t g_TxBuf1[UART1_TX_BUF_SIZE]; /* ���ͻ����� */
static uint8_t g_RxBuf1[UART1_RX_BUF_SIZE]; /* ���ջ����� */
#endif
#if UART2_FIFO_EN == 1
static UART_T g_tUart2;
static uint8_t g_TxBuf2[UART2_TX_BUF_SIZE]; /* ���ͻ����� */
static uint8_t g_RxBuf2[UART2_RX_BUF_SIZE]; /* ���ջ����� */
#endif

#if UART3_FIFO_EN == 1
static UART_T g_tUart3;
static uint8_t g_TxBuf3[UART3_TX_BUF_SIZE]; /* ���ͻ����� */
static uint8_t g_RxBuf3[UART3_RX_BUF_SIZE]; /* ���ջ����� */
#endif

static void UartVarInit(void)
{
#if UART1_FIFO_EN == 1
	g_tUart1.uart = USART1;					  /* STM32 �����豸 */
	g_tUart1.pTxBuf = g_TxBuf1;				  /* ���ͻ�����ָ�� */
	g_tUart1.pRxBuf = g_RxBuf1;				  /* ���ջ�����ָ�� */
	g_tUart1.usTxBufSize = UART1_TX_BUF_SIZE; /* ���ͻ�������С */
	g_tUart1.usRxBufSize = UART1_RX_BUF_SIZE; /* ���ջ�������С */
	g_tUart1.usTxWrite = 0;					  /* ����FIFOд���� */
	g_tUart1.usTxRead = 0;					  /* ����FIFO������ */
	g_tUart1.usRxWrite = 0;					  /* ����FIFOд���� */
	g_tUart1.usRxRead = 0;					  /* ����FIFO������ */
	g_tUart1.usRxCount = 0;					  /* ���յ��������ݸ��� */
	g_tUart1.usTxCount = 0;					  /* �����͵����ݸ��� */
	g_tUart1.SendBefor = 0;					  /* ��������ǰ�Ļص����� */
	g_tUart1.SendOver = 0;					  /* ������Ϻ�Ļص����� */
	g_tUart1.ReciveNew = 0;					  /* ���յ������ݺ�Ļص����� */
	g_tUart1.Sending = 0;					  /* ���ڷ����б�־ */
#endif

#if UART2_FIFO_EN == 1
	g_tUart2.uart = USART2;					  /* STM32 �����豸 */
	g_tUart2.pTxBuf = g_TxBuf2;				  /* ���ͻ�����ָ�� */
	g_tUart2.pRxBuf = g_RxBuf2;				  /* ���ջ�����ָ�� */
	g_tUart2.usTxBufSize = UART2_TX_BUF_SIZE; /* ���ͻ�������С */
	g_tUart2.usRxBufSize = UART2_RX_BUF_SIZE; /* ���ջ�������С */
	g_tUart2.usTxWrite = 0;					  /* ����FIFOд���� */
	g_tUart2.usTxRead = 0;					  /* ����FIFO������ */
	g_tUart2.usRxWrite = 0;					  /* ����FIFOд���� */
	g_tUart2.usRxRead = 0;					  /* ����FIFO������ */
	g_tUart2.usRxCount = 0;					  /* ���յ��������ݸ��� */
	g_tUart2.usTxCount = 0;					  /* �����͵����ݸ��� */
	g_tUart2.SendBefor = 0;					  /* ��������ǰ�Ļص����� */
	g_tUart2.SendOver = 0;					  /* ������Ϻ�Ļص����� */
	g_tUart2.ReciveNew = 0;					  /* ���յ������ݺ�Ļص����� */
	g_tUart2.Sending = 0;					  /* ���ڷ����б�־ */
#endif

#if UART3_FIFO_EN == 1
	g_tUart3.uart = USART3;					  /* STM32 �����豸 */
	g_tUart3.pTxBuf = g_TxBuf3;				  /* ���ͻ�����ָ�� */
	g_tUart3.pRxBuf = g_RxBuf3;				  /* ���ջ�����ָ�� */
	g_tUart3.usTxBufSize = UART3_TX_BUF_SIZE; /* ���ͻ�������С */
	g_tUart3.usRxBufSize = UART3_RX_BUF_SIZE; /* ���ջ�������С */
	g_tUart3.usTxWrite = 0;					  /* ����FIFOд���� */
	g_tUart3.usTxRead = 0;					  /* ����FIFO������ */
	g_tUart3.usRxWrite = 0;					  /* ����FIFOд���� */
	g_tUart3.usRxRead = 0;					  /* ����FIFO������ */
	g_tUart3.usRxCount = 0;					  /* ���յ��������ݸ��� */
	g_tUart3.usTxCount = 0;					  /* �����͵����ݸ��� */
	g_tUart3.SendBefor = RS485_SendBefor;	  /* ��������ǰ�Ļص����� */
	g_tUart3.SendOver = RS485_SendOver;		  /* ������Ϻ�Ļص����� */
	g_tUart3.ReciveNew = RS485_ReciveNew;	  /* ���յ������ݺ�Ļص����� */
	g_tUart3.Sending = 0;					  /* ���ڷ����б�־ */
#endif
}

void bsp_SetUartParam(USART_TypeDef *Instance, uint32_t BaudRate, uint32_t Parity, uint32_t Mode)
{
	UART_HandleTypeDef UartHandle;

	/*##-1- ���ô���Ӳ������ ######################################*/
	/* �첽����ģʽ (UART Mode) */
	/* ��������:
	  - �ֳ�    = 8 λ
	  - ֹͣλ  = 1 ��ֹͣλ
	  - У��    = ����Parity
	  - ������  = ����BaudRate
	  - Ӳ�������ƹر� (RTS and CTS signals) */

	UartHandle.Instance = Instance;
	UartHandle.Init.BaudRate = BaudRate;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits = UART_STOPBITS_1;
	UartHandle.Init.Parity = Parity;
	UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode = Mode;
	UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
	UartHandle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&UartHandle) != HAL_OK)
	{
		Error_Handler();
	}
}

static void InitHardUart(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
#if UART1_FIFO_EN == 1	   /* ����1 */
	/* ʹ�� USARTx ʱ�� */ /* ʹ�� GPIO TX/RX ʱ�� */
	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	/* ����TX RX���� */
	GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	/* ���ò����ʡ���żУ�� */
	bsp_SetUartParam(USART1, UART1_BAUD, UART_PARITY_NONE, UART_MODE_TX_RX);
	/* ����NVIC the NVIC for UART */
	HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
#endif

#if UART2_FIFO_EN == 1	   /* ����1 */
	/* ʹ�� USARTx ʱ�� */ /* ʹ�� GPIO TX/RX ʱ�� */
	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	/* ����TX RX���� */
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	/* ���ò����ʡ���żУ�� */
	bsp_SetUartParam(USART2, UART2_BAUD, UART_PARITY_NONE, UART_MODE_TX_RX);
	/* ����NVIC the NVIC for UART */
	HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
#endif
}

static void UartIRQ(UART_T *_pUart)
{
	uint32_t isrflags = READ_REG(_pUart->uart->ISR);
	uint32_t cr1its = READ_REG(_pUart->uart->CR1);
	uint32_t cr3its = READ_REG(_pUart->uart->CR3);

	/* ��������ж�  */
	if ((isrflags & USART_ISR_RXNE) != RESET) //���ռĴ�����Ϊ��
	{
		/* �Ӵ��ڽ������ݼĴ�����ȡ���ݴ�ŵ�����FIFO */
		uint8_t ch;

		ch = READ_REG(_pUart->uart->RDR); //��ȡ���� ����ֵ��ch
		_pUart->pRxBuf[_pUart->usRxWrite] = ch;
		if (++_pUart->usRxWrite >= _pUart->usRxBufSize)
		{
			_pUart->usRxWrite = 0;
		}
		if (_pUart->usRxCount < _pUart->usRxBufSize)
		{
			_pUart->usRxCount++;
		}

		/* �ص�����,֪ͨӦ�ó����յ�������,һ���Ƿ���1����Ϣ��������һ����� */
		//if (_pUart->usRxWrite == _pUart->usRxRead)
		//if (_pUart->usRxCount == 1)
		{
			if (_pUart->ReciveNew)
			{
				_pUart->ReciveNew(ch); /* ���磬����MODBUS����������ֽ��� */
			}
		}
	}

	/* �����ͻ��������ж� */
	//�������ݼĴ���Ϊ�� �����ж� ���� ���ɴ����ж�
	if (((isrflags & USART_ISR_TXE) != RESET) && (cr1its & USART_CR1_TXEIE) != RESET)
	{
		//		//if (_pUart->usTxRead == _pUart->usTxWrite)
		if (_pUart->usTxCount == 0)
		{
			/* ���ͻ�������������ȡ��ʱ�� ��ֹ���ͻ��������ж� ��ע�⣺��ʱ���1�����ݻ�δ����������ϣ�*/
			//USART_ITConfig(_pUart->uart, USART_IT_TXE, DISABLE);
			//��ֹ�����ж�
			CLEAR_BIT(_pUart->uart->CR1, USART_CR1_TXEIE);

			/* ʹ�����ݷ�������ж� */
			//USART_ITConfig(_pUart->uart, USART_IT_TC, ENABLE);
			SET_BIT(_pUart->uart->CR1, USART_CR1_TCIE);
		}
		else
		{
			//���� ����
			_pUart->Sending = 1;

			/* �ӷ���FIFOȡ1���ֽ�д�봮�ڷ������ݼĴ��� */
			//USART_SendData(_pUart->uart, _pUart->pTxBuf[_pUart->usTxRead]);
			_pUart->uart->TDR = _pUart->pTxBuf[_pUart->usTxRead];
			// fifo Խ��
			if (++_pUart->usTxRead >= _pUart->usTxBufSize)
			{
				_pUart->usTxRead = 0;
			}
			_pUart->usTxCount--;
		}
	}
	/* ����bitλȫ��������ϵ��ж� */
	if (((isrflags & USART_ISR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET))
	{
		//		//if (_pUart->usTxRead == _pUart->usTxWrite)
		if (_pUart->usTxCount == 0)
		{
			/* �������FIFO������ȫ��������ϣ���ֹ���ݷ�������ж� */
			//USART_ITConfig(_pUart->uart, USART_IT_TC, DISABLE);
			CLEAR_BIT(_pUart->uart->CR1, USART_CR1_TCIE);

			//			/* �ص�����, һ����������RS485ͨ�ţ���RS485оƬ����Ϊ����ģʽ��������ռ���� */
			//			if (_pUart->SendOver)
			//			{
			//				_pUart->SendOver();
			//			}
			//
			//			_pUart->Sending = 0;
		}
		else
		{ /* ��������£��������˷�֧ */

			/* �������FIFO�����ݻ�δ��ϣ���ӷ���FIFOȡ1������д�뷢�����ݼĴ��� */
			//USART_SendData(_pUart->uart, _pUart->pTxBuf[_pUart->usTxRead]);
			_pUart->uart->TDR = _pUart->pTxBuf[_pUart->usTxRead];
			if (++_pUart->usTxRead >= _pUart->usTxBufSize)
			{
				_pUart->usTxRead = 0;
			}
			_pUart->usTxCount--;
		}
	}
}
