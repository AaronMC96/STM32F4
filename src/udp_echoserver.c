/**
  ******************************************************************************
  * @file    udp_echoserver.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    07-October-2011
  * @brief   UDP echo server
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
#include <string.h>
#include <stdio.h>
#include "stm32f4_discovery.h"
#include <stm32f4xx_adc.h>
#include "adc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define UDP_SERVER_PORT    8000   /* define the UDP local connection port */
#define UDP_CLIENT_PORT    8001   /* define the UDP remote connection port */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
struct udp_pcb *client_upcb;
float tension;

/* Private function prototypes -----------------------------------------------*/
void udp_echoserver_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, struct ip_addr *addr, u16_t port);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize the server application.
  * @param  None
  * @retval None
  */
void udp_echoserver_init(void)
{
   struct udp_pcb *upcb;
   err_t err;
   
   /* Create a new UDP control block  */
   upcb = udp_new();
   
   if (upcb)
   {
     /* Bind the upcb to the UDP_PORT port */
     /* Using IP_ADDR_ANY allow the upcb to be used by any local interface */
      err = udp_bind(upcb, IP_ADDR_ANY, UDP_SERVER_PORT);
      
      if(err == ERR_OK)
      {
        /* Set a receive callback for the upcb */
        udp_recv(upcb, udp_echoserver_receive_callback, NULL);
      }
      else
      {
        printf("can not bind pcb");
      }
   }
   else
   {
     printf("can not create pcb");
   } 
}

/**
  * @brief This function is called when an UDP datagrm has been received on the port UDP_PORT.
  * @param arg user supplied argument (udp_pcb.recv_arg)
  * @param pcb the udp_pcb which received data
  * @param p the packet buffer that was received
  * @param addr the remote IP address from which the packet was received
  * @param port the remote port from which the packet was received
  * @retval None
  */
void udp_echoserver_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, struct ip_addr *addr, u16_t port)
{
	struct pbuf *rsp;

	rsp = pbuf_alloc(PBUF_TRANSPORT,80,PBUF_POOL);

	if (strcmp (p->payload, "LED1,ON")==0)
	{
		STM_EVAL_LEDOn(LED5);
		sprintf(rsp->payload,"LED1,ON");
	}

	if (strcmp (p->payload, "LED1,OFF")==0)
	{
		STM_EVAL_LEDOff(LED5);
		sprintf(rsp->payload,"LED1,OFF");
	}
	if (strcmp (p->payload, "VIN,V")==0)
	{
		tension=adc2_leer_cuentas()*2.93/4095;
		sprintf(rsp->payload,"VIN:%1.2f V",tension);
	}
	if (strcmp (p->payload, "VIN,mV")==0)
	{
		tension=(adc2_leer_cuentas()*2.93/4095)*1000;
		sprintf(rsp->payload,"VIN:%.2f mV",tension);
	}

	/* Connect to the remote client */
	udp_connect(upcb, addr, 8000); // Establece una coneccion con el servidor

	/* Tell the client that we have accepted it */
	udp_send(upcb, rsp);// envia el contenido de la cadena rsp, al servidor (puerto 8000)

	/* free the UDP connection, so we can accept new clients */
	udp_disconnect(upcb);

	/* Free the p buffer */
	pbuf_free(p);
	pbuf_free(rsp);
}

/**
  * @brief Esta funci??n inicializa un puerto UDP que va a ser usado por un cliente.
  * @param None
  * @retval None
  */

void udp_client_init()
{
  err_t err;

  /* Create a new UDP control block  */
  client_upcb = udp_new();

  if (client_upcb)
  {
    /* Bind the upcb to the UDP_PORT port */
    /* Using IP_ADDR_ANY allow the upcb to be used by any local interface */
     err = udp_bind(client_upcb, IP_ADDR_ANY, UDP_CLIENT_PORT);

     if(err == ERR_OK)
     {
       /* Set a receive callback for the upcb */
       udp_recv(client_upcb, udp_echoserver_receive_callback, NULL);
     }
     else
     {
       printf("can not bind pcb");
     }
  }
  else
  {
    printf("can not create pcb");
  }
}

/**
  * @brief Esta funci??n realiza el env??o de un mensaje de texto a la direcci??n IP y Nro de puerto indicado.
  * @param Direcci??n IPV4 de destino
  * @param Nro de puerto de destino
  * @param Mensaje a enviar (debe ser texto terminado en \0
  * @retval None
  */

void udp_client(struct ip_addr * addr, uint16_t port, char *txt)
{
	struct pbuf *msg;

	if (client_upcb)
	{
		msg = pbuf_alloc(PBUF_TRANSPORT, 80, PBUF_POOL);

/*		sprintf(msg->payload, "MSG FROM: %u.%u.%u.%u : %u msg: %s",
				(client_upcb->local_ip.addr) & 255,
				(client_upcb->local_ip.addr >> 8) & 255,
				(client_upcb->local_ip.addr >> 16) & 255,
				(client_upcb->local_ip.addr >> 24) & 255,
				client_upcb->local_port, txt);
*/

		sprintf(msg->payload, "%s", txt);

		/* Connect to the remote client */
		udp_connect(client_upcb, addr, port);

		/* Tell the client that we have accepted it */
		udp_send(client_upcb, msg);

		/* free the UDP connection, so we can accept new clients */
		udp_disconnect(client_upcb);

		/* Free the msg buffer */
		pbuf_free(msg);
	}
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
