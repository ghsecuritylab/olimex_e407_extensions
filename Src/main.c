/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "api.h"

#include <uxr/client/client.h>
#include <ucdr/microcdr.h>

#include <sys/socket.h>
#include <ip_addr.h>

#include "FreeRTOS.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal1,
  .stack_size = 3000
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
#define BUFSIZE 4096
char buffer[BUFSIZE];
extern struct netif gnetif;
extern appMain;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
 uint8_t c[1];
 c[0] = ch & 0x00FF;
 HAL_UART_Transmit(&huart3, &c[0], 1, 10);
 return ch;
}

int _write(int file,char *ptr, int len)
{
 int DataIdx;
 for(DataIdx= 0; DataIdx< len; DataIdx++)
 {
 __io_putchar(*ptr++);
 }
return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_LWIP_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PG6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  printf("Ethernet Initialization \r\n");
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
	MX_LWIP_Init();

	//Waiting for an IP
	while(gnetif.ip_addr.addr==0){
	  printf("Waiting for IP\r\n");
    osDelay(100);  
  };

	//Showing which IP was assigned
	printf("IP: %s\r\n",ip4addr_ntoa(&gnetif.ip_addr));


  // WORKING UDP SERVER
  // static struct netconn *conn;
	// static struct netbuf *buf;
	// static ip_addr_t *addr;
	// static unsigned short port;
	// void *data;
	// err_t err;
  // /* init code for LWIP */


	// conn = netconn_new(NETCONN_UDP);
	// netconn_bind(conn, IP4_ADDR_ANY, 2500);

  // /* Infinite loop */
	// for(;;){
	//     //If no error, we return the same message
	//     while (netconn_recv(conn, &buf) == ERR_OK) {
	//     	addr = netbuf_fromaddr(buf);// Saving the source address
	//     	port = netbuf_fromport(buf);//Saving the port number
        
  //      memset(buffer,'0',4096); //Cleaning the buffer variable
	//     	netbuf_copy(buf, buffer, buf->p->tot_len);
	//     	buffer[buf->p->tot_len] = '\0';

  //       char aux[100];
  //       ipaddr_ntoa_r(addr,aux,100);
  //       printf("data from %s: %s\r\n",aux,buffer); //Received data

  //       // netconn_connect(conn, ip4addr_ntoa(&buf->addr), port); //Connecting to the client
  //       netconn_sendto(conn, buf, addr, port); //Sending back to the source
	//     	netbuf_delete(buf);
	//     }
	// 	osDelay(100);
	// 	printf(".\r\n");
	// }
	// printf("-\r\n");

  // ------------ WORKING UDP CLIENT nc -u -l 0.0.0.0 -p 2600
  // static struct netconn *conn;
	// conn = netconn_new(NETCONN_UDP);
	// netconn_bind(conn, IP4_ADDR_ANY, 3500);
  // const ip_addr_t addr = IPADDR4_INIT_BYTES(192, 168, 1, 79);
  // u16_t port = 2600;


  // while(1){
  // vTaskDelay(2000);

  // uxrUDPTransport transport;
  // uxrUDPPlatform udp_platform;

  // if(!uxr_init_udp_transport(&transport, &udp_platform, UXR_IPv4, "192.168.1.79", "8888")){
  //   printf("Error at create transport.\n");
  //   return 1;
  // }
  //   char *mem;
  //   struct netbuf *buf;
  //   buf = netbuf_new(); /* create a new netbuf */
  //   char data[] = "HOLAAAAA\n";
  //   mem = (char *)netbuf_alloc(buf, strlen(data));
  //   strncpy(mem, data, strlen(data));
  //   // netbuf_copy(buf, data, strlen(data));
  
  //   netconn_sendto(conn, buf, &addr, port); //Sending back to the source
      
  //   netbuf_delete(buf); /* deallocate netbuf */

  //   osDelay(1000);
  // }

  // WORKING POSIX CLIENT
  // char buffer[100]; 
  // char *message = "Hello Server\n"; 
  // int sockfd, n; 
  // struct sockaddr_in servaddr; 
    
  // // clear servaddr 
  // bzero(&servaddr, sizeof(servaddr)); 
  // servaddr.sin_addr.s_addr = inet_addr("192.168.1.115"); 
  // servaddr.sin_port = htons(2600); 
  // servaddr.sin_family = AF_INET; 
    
  // // create datagram socket 
  // sockfd = socket(AF_INET, SOCK_DGRAM, 0); 
    
  // // connect to server 
  // if(connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) 
  // { 
  //     printf("\n Error : Connect Failed \n"); 
  //     exit(0); 
  // } 

  // // request to send datagram 
  // // no need to specify server address in sendto 
  // // connect stores the peers IP and port 

  // struct timeval tv;
  // tv.tv_sec = 0;
  // tv.tv_usec = 500000;
  // setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  // while (1){
  //   sendto(sockfd, message, strlen(message), 0, (struct sockaddr*)&servaddr, sizeof(servaddr)); 
  //   char rx[100];
  //   memset(rx,0,100);
  //   if(recv(sockfd,&rx,100,0) > 0){
  //     printf("Received %s\n",rx);
  //   }else{
  //     printf("Nothing rx\n");
  //   }
  // }
  
  // // close the descriptor 
  // close(sockfd); 

  
  // WORKING POSIX SERVER
 
  //   int sockfd; 
  //   char buffer[100]; 
  //   char *hello = "Hello from server"; 
  //   struct sockaddr_in servaddr, cliaddr; 
      
  //   // Creating socket file descriptor 
  //   if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
  //       perror("socket creation failed"); 
  //       exit(EXIT_FAILURE); 
  //   } 
      
  //   memset(&servaddr, 0, sizeof(servaddr)); 
  //   memset(&cliaddr, 0, sizeof(cliaddr)); 
      
  //   // Filling server information 
  //   servaddr.sin_family    = AF_INET; // IPv4 
  //   servaddr.sin_addr.s_addr = INADDR_ANY; 
  //   servaddr.sin_port = htons(2600); 
      
  //   // Bind the socket with the server address 
  //   if ( bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0 ) 
  //   { 
  //       perror("bind failed"); 
  //       exit(EXIT_FAILURE); 
  //   } 
      
  //   int len, n; 
  
  //   len = sizeof(cliaddr);  //len is value/resuslt 
  //   while(1){
  //     n = recvfrom(sockfd, (char *)buffer, 100,  0, ( struct sockaddr *) &cliaddr, &len); 
  //     buffer[n] = '\0'; 
  //     printf("Client : %s\n", buffer); 
  //     sendto(sockfd, (const char *)buffer, strlen(buffer), 0, (const struct sockaddr *) &cliaddr, len);
  //   }
 

  // close(sockfd); 
  
  // vTaskDelay(2000);

  // Launch app thread when IP configured
  
  osThreadAttr_t attributes;
  memset(&attributes, 0x0, sizeof(osThreadAttr_t));
  attributes.name = "app";
  attributes.stack_size = 3000;
  attributes.priority = (osPriority_t) osPriorityNormal1;
  osThreadNew(appMain, NULL, &attributes);

  // Kill init thread
  // vTaskDelete(NULL);

  while (1){
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    osDelay(1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    osDelay(1000);
  }
  

  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
