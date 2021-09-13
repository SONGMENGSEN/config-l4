#include "bsp_uart_fifo.h"
//usart 1
#define USART1_CLK_ENABLE() __HAL_RCC_USART1_CLK_ENABLE()
#define USART1_TX_GPIO_PORT GPIOA
#define USART1_TX_PIN GPIO_PIN_9
#define USART1_TX_AF GPIO_AF7_USART1
#define USART1_RX_GPIO_PORT GPIOA
#define USART1_RX_PIN GPIO_PIN_10
#define USART1_RX_AF GPIO_AF7_USART1
//usart 2
#define USART2_CLK_ENABLE() __HAL_RCC_USART2_CLK_ENABLE()
#define USART2_TX_GPIO_PORT GPIOA
#define USART2_TX_PIN GPIO_PIN_2
#define USART2_TX_AF GPIO_AF7_USART2
#define USART2_RX_GPIO_PORT GPIOA
#define USART2_RX_PIN GPIO_PIN_3
#define USART2_RX_AF GPIO_AF7_USART2
//usart 3
#define USART3_CLK_ENABLE() __HAL_RCC_USART3_CLK_ENABLE()
#define USART3_TX_GPIO_PORT GPIOB
#define USART3_TX_PIN GPIO_PIN_10
#define USART3_TX_AF GPIO_AF7_USART3
#define USART3_RX_GPIO_PORT GPIOB
#define USART3_RX_PIN GPIO_PIN_11
#define USART3_RX_AF GPIO_AF7_USART3

#if UART1_FIFO_EN == 1
UART_T g_tUart1;
static uint8_t g_TxBuf1[UART1_TX_BUF_SIZE];
static uint8_t g_RxBuf1[UART1_RX_BUF_SIZE];
#endif
#if UART2_FIFO_EN == 1
static UART_T g_tUart2;
static uint8_t g_TxBuf2[UART2_TX_BUF_SIZE];
static uint8_t g_RxBuf2[UART2_RX_BUF_SIZE];
#endif
#if UART3_FIFO_EN == 1
static UART_T g_tUart3;
static uint8_t g_TxBuf3[UART3_TX_BUF_SIZE];
static uint8_t g_RxBuf3[UART3_RX_BUF_SIZE];
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

/*
*********************************************************************************************************
*	�� �� ��: bsp_SetUartParam
*	����˵��: ���ô��ڵ�Ӳ�������������ʣ�����λ��ֹͣλ����ʼλ��У��λ���ж�ʹ�ܣ��ʺ���STM32- H7������
*	��    ��: Instance   USART_TypeDef���ͽṹ��
*             BaudRate   ������
*             Parity     У�����ͣ���У�����żУ��
*             Mode       ���ͺͽ���ģʽʹ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
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

	if (HAL_UART_Init(&UartHandle) != HAL_OK)
	{
		Error_Handler();
	}
}

static void InitHardUart(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

#if UART1_FIFO_EN == 1 /* ����1 */

	/* ʹ�� USARTx ʱ�� */
	USART1_CLK_ENABLE();

	/* ����TX���� */
	GPIO_InitStruct.Pin = USART1_TX_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = USART1_TX_AF;
	HAL_GPIO_Init(USART1_TX_GPIO_PORT, &GPIO_InitStruct);

	/* ����RX���� */
	GPIO_InitStruct.Pin = USART1_RX_PIN;
	GPIO_InitStruct.Alternate = USART1_RX_AF;
	HAL_GPIO_Init(USART1_RX_GPIO_PORT, &GPIO_InitStruct);

	/* ����NVIC the NVIC for UART */
	HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(USART1_IRQn);

	/* ���ò����ʡ���żУ�� */
	bsp_SetUartParam(USART1, UART1_BAUD, UART_PARITY_NONE, UART_MODE_TX_RX);

	CLEAR_BIT(USART1->ISR, USART_ICR_TCCF);	 /* ���TC������ɱ�־ */
	CLEAR_BIT(USART1->RQR, USART_RQR_RXFRQ); /* ���RXNE���ձ�־ */
	// USART_CR1_PEIE | USART_CR1_RXNEIE
	SET_BIT(USART1->CR1, USART_CR1_RXNEIE); /* ʹ��PE. RX�����ж� */
#endif

#if UART2_FIFO_EN == 1 /* ����2 */
	/* ʹ�� GPIO TX/RX ʱ�� */
	/* ʹ�� USARTx ʱ�� */
	USART2_CLK_ENABLE();

	/* ����TX���� */
	GPIO_InitStruct.Pin = USART2_TX_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = USART2_TX_AF;
	HAL_GPIO_Init(USART2_TX_GPIO_PORT, &GPIO_InitStruct);

	/* ����RX���� */
	GPIO_InitStruct.Pin = USART2_RX_PIN;
	GPIO_InitStruct.Alternate = USART2_RX_AF;
	HAL_GPIO_Init(USART2_RX_GPIO_PORT, &GPIO_InitStruct);

	/* ����NVIC the NVIC for UART */
	HAL_NVIC_SetPriority(USART2_IRQn, 0, 2);
	HAL_NVIC_EnableIRQ(USART2_IRQn);

	/* ���ò����ʡ���żУ�� */
	bsp_SetUartParam(USART2, UART2_BAUD, UART_PARITY_NONE, UART_MODE_TX_RX); // UART_MODE_TX_RX

	CLEAR_BIT(USART2->ISR, USART_ICR_TCCF);	 /* ���TC������ɱ�־ */
	CLEAR_BIT(USART2->RQR, USART_RQR_RXFRQ); /* ���RXNE���ձ�־ */
	SET_BIT(USART2->CR1, USART_CR1_RXNEIE);	 /* ʹ��PE. RX�����ж� */
#endif

#if UART3_FIFO_EN == 1 /* ����3 */
	/* ʹ�� GPIO TX/RX ʱ�� */
	/* ʹ�� USARTx ʱ�� */
	USART3_CLK_ENABLE();

	/* ����TX���� */
	GPIO_InitStruct.Pin = USART3_TX_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = USART3_TX_AF;
	HAL_GPIO_Init(USART3_TX_GPIO_PORT, &GPIO_InitStruct);

	/* ����RX���� */
	GPIO_InitStruct.Pin = USART3_RX_PIN;
	GPIO_InitStruct.Alternate = USART3_RX_AF;
	HAL_GPIO_Init(USART3_RX_GPIO_PORT, &GPIO_InitStruct);

	/* ����NVIC the NVIC for UART */
	HAL_NVIC_SetPriority(USART3_IRQn, 0, 3);
	HAL_NVIC_EnableIRQ(USART3_IRQn);

	/* ���ò����ʡ���żУ�� */
	bsp_SetUartParam(USART3, UART3_BAUD, UART_PARITY_NONE, UART_MODE_TX_RX);

	CLEAR_BIT(USART3->ISR, USART_ISR_TC);	/* ���TC������ɱ�־ */
	CLEAR_BIT(USART3->ISR, USART_ISR_RXNE); /* ���RXNE���ձ�־ */
	SET_BIT(USART3->CR1, USART_CR1_RXNEIE); /* ʹ��PE. RX�����ж� */
#endif
}

void bsp_InitUart(void)
{

	UartVarInit(); /* �����ȳ�ʼ��ȫ�ֱ���,������Ӳ�� */

	InitHardUart(); /* ���ô��ڵ�Ӳ������(�����ʵ�) */

	//	RS485_InitTXE();/* ����RS485оƬ�ķ���ʹ��Ӳ��������Ϊ������� */
}

/*
*********************************************************************************************************
*	�� �� ��: ComToUart
*	����˵��: ��COM�˿ں�ת��ΪUARTָ��
*	��    ��: _ucPort: �˿ں�(COM1 - COM8)
*	�� �� ֵ: uartָ��
*********************************************************************************************************
*/
UART_T *ComToUart(COM_PORT_E _ucPort)
{
	if (_ucPort == COM1)
	{
#if UART1_FIFO_EN == 1
		return &g_tUart1;
#else
		return 0;
#endif
	}
	else if (_ucPort == COM2)
	{
#if UART2_FIFO_EN == 1
		return &g_tUart2;
#else
		return 0;
#endif
	}
	else if (_ucPort == COM3)
	{
#if UART3_FIFO_EN == 1
		return &g_tUart3;
#else
		return 0;
#endif
	}
	else
	{
		Error_Handler();
		return 0;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: ComToUart
*	����˵��: ��COM�˿ں�ת��Ϊ USART_TypeDef* USARTx
*	��    ��: _ucPort: �˿ں�(COM1 - COM8)
*	�� �� ֵ: USART_TypeDef*,  USART1, USART2, USART3, UART4, UART5��USART6��UART7��UART8��
*********************************************************************************************************
*/
USART_TypeDef *ComToUSARTx(COM_PORT_E _ucPort)
{
	if (_ucPort == COM1)
	{
#if UART1_FIFO_EN == 1
		return USART1;
#else
		return 0;
#endif
	}
	else if (_ucPort == COM2)
	{
#if UART2_FIFO_EN == 1
		return USART2;
#else
		return 0;
#endif
	}
	else if (_ucPort == COM3)
	{
#if UART3_FIFO_EN == 1
		return USART3;
#else
		return 0;
#endif
	}
	else
	{
		/* �����κδ��� */
		return 0;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: UartSend
*	����˵��: ��д���ݵ�UART���ͻ�����,�����������жϡ��жϴ�������������Ϻ��Զ��رշ����ж�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void UartSend(UART_T *_pUart, uint8_t *_ucaBuf, uint16_t _usLen)
{
	uint16_t i;

	for (i = 0; i < _usLen; i++)
	{
		/* ������ͻ������Ѿ����ˣ���ȴ��������� */
		while (1)
		{
			__IO uint16_t usCount;

			usCount = _pUart->usTxCount;

			if (usCount < _pUart->usTxBufSize)
			{
				break;
			}
			else if (usCount == _pUart->usTxBufSize) /* ���������������� */
			{
				if ((_pUart->uart->CR1 & USART_CR1_TXEIE) == 0)
				{
					SET_BIT(_pUart->uart->CR1, USART_CR1_TXEIE);
				}
			}
		}

		/* �����������뷢�ͻ����� */
		_pUart->pTxBuf[_pUart->usTxWrite] = _ucaBuf[i];

		if (++_pUart->usTxWrite >= _pUart->usTxBufSize)
		{
			_pUart->usTxWrite = 0;
		}
		_pUart->usTxCount++;
	}

	SET_BIT(_pUart->uart->CR1, USART_CR1_TXEIE); /* ʹ�ܷ����жϣ��������գ� */
}

/*
*********************************************************************************************************
*	�� �� ��: comSendBuf
*	����˵��: �򴮿ڷ���һ�����ݡ����ݷŵ����ͻ��������������أ����жϷ�������ں�̨��ɷ���
*	��    ��: _ucPort: �˿ں�(COM1 - COM8)
*			  _ucaBuf: �����͵����ݻ�����
*			  _usLen : ���ݳ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void comSendBuf(COM_PORT_E _ucPort, uint8_t *_ucaBuf, uint16_t _usLen)
{
	UART_T *pUart;

	pUart = ComToUart(_ucPort);
	if (pUart == 0)
	{
		return;
	}

	if (pUart->SendBefor != 0)
	{
		pUart->SendBefor(); /* �����RS485ͨ�ţ���������������н�RS485����Ϊ����ģʽ */
	}

	UartSend(pUart, _ucaBuf, _usLen);
}

/*
*********************************************************************************************************
*	�� �� ��: comSendChar
*	����˵��: �򴮿ڷ���1���ֽڡ����ݷŵ����ͻ��������������أ����жϷ�������ں�̨��ɷ���
*	��    ��: _ucPort: �˿ں�(COM1 - COM8)
*			  _ucByte: �����͵�����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void comSendChar(COM_PORT_E _ucPort, uint8_t _ucByte)
{
	comSendBuf(_ucPort, &_ucByte, 1);
}

/*
*********************************************************************************************************
*	�� �� ��: UartGetChar
*	����˵��: �Ӵ��ڽ��ջ�������ȡ1�ֽ����� ��������������ã�
*	��    ��: _pUart : �����豸
*			  _pByte : ��Ŷ�ȡ���ݵ�ָ��
*	�� �� ֵ: 0 ��ʾ������  1��ʾ��ȡ������
*********************************************************************************************************
*/
static uint8_t UartGetChar(UART_T *_pUart, uint8_t *_pByte)
{
	uint16_t usCount;

	/* usRxWrite �������жϺ����б���д���������ȡ�ñ���ʱ����������ٽ������� */

	usCount = _pUart->usRxCount;

	/* �������д������ͬ���򷵻�0 */
	//if (_pUart->usRxRead == usRxWrite)
	if (usCount == 0) /* �Ѿ�û������ */
	{
		return 0;
	}
	else
	{
		*_pByte = _pUart->pRxBuf[_pUart->usRxRead]; /* �Ӵ��ڽ���FIFOȡ1������ */

		/* ��дFIFO������ */
		//		DISABLE_INT();
		if (++_pUart->usRxRead >= _pUart->usRxBufSize)
		{
			_pUart->usRxRead = 0;
		}
		_pUart->usRxCount--;
		//		ENABLE_INT();
		return 1;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: comGetChar
*	����˵��: �ӽ��ջ�������ȡ1�ֽڣ��������������������ݾ��������ء�
*	��    ��: _ucPort: �˿ں�(COM1 - COM8)
*			  _pByte: ���յ������ݴ���������ַ
*	�� �� ֵ: 0 ��ʾ������, 1 ��ʾ��ȡ����Ч�ֽ�
*********************************************************************************************************
*/
uint8_t comGetChar(COM_PORT_E _ucPort, uint8_t *_pByte)
{
	UART_T *pUart;

	pUart = ComToUart(_ucPort);
	if (pUart == 0)
	{
		return 0;
	}

	return UartGetChar(pUart, _pByte);
}

/*
*********************************************************************************************************
*	�� �� ��: comClearTxFifo
*	����˵��: ���㴮�ڷ��ͻ�����
*	��    ��: _ucPort: �˿ں�(COM1 - COM8)
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void comClearTxFifo(COM_PORT_E _ucPort)
{
	UART_T *pUart;

	pUart = ComToUart(_ucPort);
	if (pUart == 0)
	{
		return;
	}

	pUart->usTxWrite = 0;
	pUart->usTxRead = 0;
	pUart->usTxCount = 0;
}
/*
*********************************************************************************************************
*	�� �� ��: comClearRxFifo
*	����˵��: ���㴮�ڽ��ջ�����
*	��    ��: _ucPort: �˿ں�(COM1 - COM8)
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void comClearRxFifo(COM_PORT_E _ucPort)
{
	UART_T *pUart;

	pUart = ComToUart(_ucPort);
	if (pUart == 0)
	{
		return;
	}

	pUart->usRxWrite = 0;
	pUart->usRxRead = 0;
	pUart->usRxCount = 0;
}

/*
*********************************************************************************************************
*	�� �� ��: UartIRQ
*	����˵��: ���жϷ��������ã�ͨ�ô����жϴ�������
*	��    ��: _pUart : �����豸
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void UartIRQ(UART_T *_pUart)
{
	uint32_t isrflags = READ_REG(_pUart->uart->ISR);
	uint32_t cr1its = READ_REG(_pUart->uart->CR1);
	uint32_t cr3its = READ_REG(_pUart->uart->CR3);

	/* ���������ж�  */
	if ((isrflags & USART_ISR_RXNE) != RESET)
	{
		/* �Ӵ��ڽ������ݼĴ�����ȡ���ݴ�ŵ�����FIFO */
		uint8_t ch;
		//CLEAR_BIT(_pUart->uart->ICR,UART_CLEAR_IDLEF);

		ch = READ_REG(_pUart->uart->RDR);
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

	/* �������ͻ��������ж� */
	if (((isrflags & USART_ISR_TXE) != RESET) && (cr1its & USART_CR1_TXEIE) != RESET)
	{
		//if (_pUart->usTxRead == _pUart->usTxWrite)
		if (_pUart->usTxCount == 0)
		{
			/* ���ͻ�������������ȡ��ʱ�� ��ֹ���ͻ��������ж� ��ע�⣺��ʱ���1�����ݻ�δ����������ϣ�*/
			//USART_ITConfig(_pUart->uart, USART_IT_TXE, DISABLE);
			CLEAR_BIT(_pUart->uart->CR1, USART_CR1_TXEIE);

			/* ʹ�����ݷ�������ж� */
			//USART_ITConfig(_pUart->uart, USART_IT_TC, ENABLE);
			SET_BIT(_pUart->uart->CR1, USART_CR1_TCIE);
		}
		else
		{
			_pUart->Sending = 1;

			/* �ӷ���FIFOȡ1���ֽ�д�봮�ڷ������ݼĴ��� */
			//USART_SendData(_pUart->uart, _pUart->pTxBuf[_pUart->usTxRead]);
			_pUart->uart->TDR = _pUart->pTxBuf[_pUart->usTxRead];
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
		//if (_pUart->usTxRead == _pUart->usTxWrite)
		if (_pUart->usTxCount == 0)
		{
			/* �������FIFO������ȫ��������ϣ���ֹ���ݷ�������ж� */
			//USART_ITConfig(_pUart->uart, USART_IT_TC, DISABLE);
			CLEAR_BIT(_pUart->uart->CR1, USART_CR1_TCIE);

			/* �ص�����, һ����������RS485ͨ�ţ���RS485оƬ����Ϊ����ģʽ��������ռ���� */
			if (_pUart->SendOver)
			{
				_pUart->SendOver();
			}

			_pUart->Sending = 0;
		}
		else
		{
			/* ��������£��������˷�֧ */

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

	SET_BIT(_pUart->uart->ICR, UART_CLEAR_PEF);
	SET_BIT(_pUart->uart->ICR, UART_CLEAR_FEF);
	SET_BIT(_pUart->uart->ICR, UART_CLEAR_NEF);
	SET_BIT(_pUart->uart->ICR, UART_CLEAR_OREF);
	SET_BIT(_pUart->uart->ICR, UART_CLEAR_IDLEF);
	SET_BIT(_pUart->uart->ICR, UART_CLEAR_TCF);
	SET_BIT(_pUart->uart->ICR, UART_CLEAR_LBDF);
	SET_BIT(_pUart->uart->ICR, UART_CLEAR_CTSF);
	SET_BIT(_pUart->uart->ICR, UART_CLEAR_CMF);
	SET_BIT(_pUart->uart->ICR, UART_CLEAR_WUF);
	SET_BIT(_pUart->uart->ICR, UART_CLEAR_RTOF);
}

/*
*********************************************************************************************************
*   �� �� ��: UartTxEmpty
*   ����˵��: �жϷ��ͻ������Ƿ�Ϊ�ա�
*   ��    ��:  _pUart : �����豸
*   �� �� ֵ: 1Ϊ�ա�0Ϊ���ա�
*********************************************************************************************************
*/
uint8_t UartTxEmpty(COM_PORT_E _ucPort)
{
	UART_T *pUart;
	uint8_t Sending;

	pUart = ComToUart(_ucPort);
	if (pUart == 0)
	{
		return 0;
	}

	Sending = pUart->Sending;

	if (Sending != 0)
	{
		return 0;
	}
	return 1;
}

/*
*********************************************************************************************************
*	�� �� ��: USART1_IRQHandler  USART2_IRQHandler USART3_IRQHandler UART4_IRQHandler UART5_IRQHandler��
*	����˵��: USART�жϷ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
#if UART1_FIFO_EN == 1
void USART1_IRQHandler(void)
{
	UartIRQ(&g_tUart1);
}
#endif

#if UART2_FIFO_EN == 1
void USART2_IRQHandler(void)
{
	UartIRQ(&g_tUart2);
}
#endif

#if UART3_FIFO_EN == 1
void USART3_IRQHandler(void)
{
	UartIRQ(&g_tUart3);
}
#endif

int fputc(int ch, FILE *f)
{
#if 0 /* ����Ҫprintf���ַ�ͨ�������ж�FIFO���ͳ�ȥ��printf�������������� */
	comSendChar(COM1, ch);
	
	return ch;
#else /* ����������ʽ����ÿ���ַ�,�ȴ����ݷ������ */
	/* дһ���ֽڵ�USART1 */
	USART1->TDR = ch;

	/* �ȴ����ͽ��� */
	while ((USART1->ISR & USART_ISR_TC) == 0)
	{
	}

	return ch;
#endif
}

int fgetc(FILE *f)
{

#if 1 /* �Ӵ��ڽ���FIFO��ȡ1������, ֻ��ȡ�����ݲŷ��� */
	uint8_t ucData;

	while (comGetChar(COM1, &ucData) == 0)
		;

	return ucData;
#else
	/* �ȴ����յ����� */
	while ((USART1->ISR & USART_ISR_RXNE) == 0)
	{
	}

	return (int)USART1->RDR;
#endif
}