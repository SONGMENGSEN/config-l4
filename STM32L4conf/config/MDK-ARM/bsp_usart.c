#include "bsp_usart.h"

#if UART1_FIFO_EN == 1
static UART_T g_tUart1;
static uint8_t g_TxBuf1[UART1_TX_BUF_SIZE]; /* 发送缓冲区 */
static uint8_t g_RxBuf1[UART1_RX_BUF_SIZE]; /* 接收缓冲区 */
#endif
#if UART2_FIFO_EN == 1
static UART_T g_tUart2;
static uint8_t g_TxBuf2[UART2_TX_BUF_SIZE]; /* 发送缓冲区 */
static uint8_t g_RxBuf2[UART2_RX_BUF_SIZE]; /* 接收缓冲区 */
#endif

#if UART3_FIFO_EN == 1
static UART_T g_tUart3;
static uint8_t g_TxBuf3[UART3_TX_BUF_SIZE]; /* 发送缓冲区 */
static uint8_t g_RxBuf3[UART3_RX_BUF_SIZE]; /* 接收缓冲区 */
#endif

static void UartVarInit(void)
{
#if UART1_FIFO_EN == 1
	g_tUart1.uart = USART1;					  /* STM32 串口设备 */
	g_tUart1.pTxBuf = g_TxBuf1;				  /* 发送缓冲区指针 */
	g_tUart1.pRxBuf = g_RxBuf1;				  /* 接收缓冲区指针 */
	g_tUart1.usTxBufSize = UART1_TX_BUF_SIZE; /* 发送缓冲区大小 */
	g_tUart1.usRxBufSize = UART1_RX_BUF_SIZE; /* 接收缓冲区大小 */
	g_tUart1.usTxWrite = 0;					  /* 发送FIFO写索引 */
	g_tUart1.usTxRead = 0;					  /* 发送FIFO读索引 */
	g_tUart1.usRxWrite = 0;					  /* 接收FIFO写索引 */
	g_tUart1.usRxRead = 0;					  /* 接收FIFO读索引 */
	g_tUart1.usRxCount = 0;					  /* 接收到的新数据个数 */
	g_tUart1.usTxCount = 0;					  /* 待发送的数据个数 */
	g_tUart1.SendBefor = 0;					  /* 发送数据前的回调函数 */
	g_tUart1.SendOver = 0;					  /* 发送完毕后的回调函数 */
	g_tUart1.ReciveNew = 0;					  /* 接收到新数据后的回调函数 */
	g_tUart1.Sending = 0;					  /* 正在发送中标志 */
#endif

#if UART2_FIFO_EN == 1
	g_tUart2.uart = USART2;					  /* STM32 串口设备 */
	g_tUart2.pTxBuf = g_TxBuf2;				  /* 发送缓冲区指针 */
	g_tUart2.pRxBuf = g_RxBuf2;				  /* 接收缓冲区指针 */
	g_tUart2.usTxBufSize = UART2_TX_BUF_SIZE; /* 发送缓冲区大小 */
	g_tUart2.usRxBufSize = UART2_RX_BUF_SIZE; /* 接收缓冲区大小 */
	g_tUart2.usTxWrite = 0;					  /* 发送FIFO写索引 */
	g_tUart2.usTxRead = 0;					  /* 发送FIFO读索引 */
	g_tUart2.usRxWrite = 0;					  /* 接收FIFO写索引 */
	g_tUart2.usRxRead = 0;					  /* 接收FIFO读索引 */
	g_tUart2.usRxCount = 0;					  /* 接收到的新数据个数 */
	g_tUart2.usTxCount = 0;					  /* 待发送的数据个数 */
	g_tUart2.SendBefor = 0;					  /* 发送数据前的回调函数 */
	g_tUart2.SendOver = 0;					  /* 发送完毕后的回调函数 */
	g_tUart2.ReciveNew = 0;					  /* 接收到新数据后的回调函数 */
	g_tUart2.Sending = 0;					  /* 正在发送中标志 */
#endif

#if UART3_FIFO_EN == 1
	g_tUart3.uart = USART3;					  /* STM32 串口设备 */
	g_tUart3.pTxBuf = g_TxBuf3;				  /* 发送缓冲区指针 */
	g_tUart3.pRxBuf = g_RxBuf3;				  /* 接收缓冲区指针 */
	g_tUart3.usTxBufSize = UART3_TX_BUF_SIZE; /* 发送缓冲区大小 */
	g_tUart3.usRxBufSize = UART3_RX_BUF_SIZE; /* 接收缓冲区大小 */
	g_tUart3.usTxWrite = 0;					  /* 发送FIFO写索引 */
	g_tUart3.usTxRead = 0;					  /* 发送FIFO读索引 */
	g_tUart3.usRxWrite = 0;					  /* 接收FIFO写索引 */
	g_tUart3.usRxRead = 0;					  /* 接收FIFO读索引 */
	g_tUart3.usRxCount = 0;					  /* 接收到的新数据个数 */
	g_tUart3.usTxCount = 0;					  /* 待发送的数据个数 */
	g_tUart3.SendBefor = RS485_SendBefor;	  /* 发送数据前的回调函数 */
	g_tUart3.SendOver = RS485_SendOver;		  /* 发送完毕后的回调函数 */
	g_tUart3.ReciveNew = RS485_ReciveNew;	  /* 接收到新数据后的回调函数 */
	g_tUart3.Sending = 0;					  /* 正在发送中标志 */
#endif
}

void bsp_SetUartParam(USART_TypeDef *Instance, uint32_t BaudRate, uint32_t Parity, uint32_t Mode)
{
	UART_HandleTypeDef UartHandle;

	/*##-1- 配置串口硬件参数 ######################################*/
	/* 异步串口模式 (UART Mode) */
	/* 配置如下:
	  - 字长    = 8 位
	  - 停止位  = 1 个停止位
	  - 校验    = 参数Parity
	  - 波特率  = 参数BaudRate
	  - 硬件流控制关闭 (RTS and CTS signals) */

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
#if UART1_FIFO_EN == 1	   /* 串口1 */
	/* 使能 USARTx 时钟 */ /* 使能 GPIO TX/RX 时钟 */
	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	/* 配置TX RX引脚 */
	GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	/* 配置波特率、奇偶校验 */
	bsp_SetUartParam(USART1, UART1_BAUD, UART_PARITY_NONE, UART_MODE_TX_RX);
	/* 配置NVIC the NVIC for UART */
	HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
#endif

#if UART2_FIFO_EN == 1	   /* 串口1 */
	/* 使能 USARTx 时钟 */ /* 使能 GPIO TX/RX 时钟 */
	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	/* 配置TX RX引脚 */
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	/* 配置波特率、奇偶校验 */
	bsp_SetUartParam(USART2, UART2_BAUD, UART_PARITY_NONE, UART_MODE_TX_RX);
	/* 配置NVIC the NVIC for UART */
	HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
#endif
}

static void UartIRQ(UART_T *_pUart)
{
	uint32_t isrflags = READ_REG(_pUart->uart->ISR);
	uint32_t cr1its = READ_REG(_pUart->uart->CR1);
	uint32_t cr3its = READ_REG(_pUart->uart->CR3);

	/* 处理接收中断  */
	if ((isrflags & USART_ISR_RXNE) != RESET) //接收寄存器不为空
	{
		/* 从串口接收数据寄存器读取数据存放到接收FIFO */
		uint8_t ch;

		ch = READ_REG(_pUart->uart->RDR); //读取数据 并赋值给ch
		_pUart->pRxBuf[_pUart->usRxWrite] = ch;
		if (++_pUart->usRxWrite >= _pUart->usRxBufSize)
		{
			_pUart->usRxWrite = 0;
		}
		if (_pUart->usRxCount < _pUart->usRxBufSize)
		{
			_pUart->usRxCount++;
		}

		/* 回调函数,通知应用程序收到新数据,一般是发送1个消息或者设置一个标记 */
		//if (_pUart->usRxWrite == _pUart->usRxRead)
		//if (_pUart->usRxCount == 1)
		{
			if (_pUart->ReciveNew)
			{
				_pUart->ReciveNew(ch); /* 比如，交给MODBUS解码程序处理字节流 */
			}
		}
	}

	/* 处理发送缓冲区空中断 */
	//发送数据寄存器为空 引发中断 并且 生成串口中断
	if (((isrflags & USART_ISR_TXE) != RESET) && (cr1its & USART_CR1_TXEIE) != RESET)
	{
		//		//if (_pUart->usTxRead == _pUart->usTxWrite)
		if (_pUart->usTxCount == 0)
		{
			/* 发送缓冲区的数据已取完时， 禁止发送缓冲区空中断 （注意：此时最后1个数据还未真正发送完毕）*/
			//USART_ITConfig(_pUart->uart, USART_IT_TXE, DISABLE);
			//禁止发送中断
			CLEAR_BIT(_pUart->uart->CR1, USART_CR1_TXEIE);

			/* 使能数据发送完毕中断 */
			//USART_ITConfig(_pUart->uart, USART_IT_TC, ENABLE);
			SET_BIT(_pUart->uart->CR1, USART_CR1_TCIE);
		}
		else
		{
			//正在 发送
			_pUart->Sending = 1;

			/* 从发送FIFO取1个字节写入串口发送数据寄存器 */
			//USART_SendData(_pUart->uart, _pUart->pTxBuf[_pUart->usTxRead]);
			_pUart->uart->TDR = _pUart->pTxBuf[_pUart->usTxRead];
			// fifo 越界
			if (++_pUart->usTxRead >= _pUart->usTxBufSize)
			{
				_pUart->usTxRead = 0;
			}
			_pUart->usTxCount--;
		}
	}
	/* 数据bit位全部发送完毕的中断 */
	if (((isrflags & USART_ISR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET))
	{
		//		//if (_pUart->usTxRead == _pUart->usTxWrite)
		if (_pUart->usTxCount == 0)
		{
			/* 如果发送FIFO的数据全部发送完毕，禁止数据发送完毕中断 */
			//USART_ITConfig(_pUart->uart, USART_IT_TC, DISABLE);
			CLEAR_BIT(_pUart->uart->CR1, USART_CR1_TCIE);

			//			/* 回调函数, 一般用来处理RS485通信，将RS485芯片设置为接收模式，避免抢占总线 */
			//			if (_pUart->SendOver)
			//			{
			//				_pUart->SendOver();
			//			}
			//
			//			_pUart->Sending = 0;
		}
		else
		{ /* 正常情况下，不会进入此分支 */

			/* 如果发送FIFO的数据还未完毕，则从发送FIFO取1个数据写入发送数据寄存器 */
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
