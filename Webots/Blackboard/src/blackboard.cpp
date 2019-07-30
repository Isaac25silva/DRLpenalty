/*--------------------------------------------------------------------

******************************************************************************
* @file blackboard.h
* @author Isaac Jesus da Silva - ROBOFEI-HT - FEI
* @version V0.0.2
* @created 07/04/2014
* @Modified 11/04/2014
* @e-mail isaac25silva@yahoo.com.br
* @brief black board
****************************************************************************

Arquivo fonte contendo as funções que criam ou acopla a memória compartilhada

/--------------------------------------------------------------------*/

#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include "blackboard.h"

//#define DEBUG


int *mem ; //Variável que manipula memória compartilhada
int using_shared_memory();
int leitura_int(int index);
void escreve_int(int index, int valor);

//Depois de criado a memória compartilhada, para verificar se ela realmente foi criada
// e quantos processos estão utilizando, digite no terminal o comando $ipcs -m
// será a memoria criada ->   key = 0x0000007b    bytes = 2048
// nattch = number of attached processes

void escreve_int(int index, int valor)
{
    *(mem+index) = valor;
}

int leitura_int(int index)
{
    return *(mem+index);
}


int using_shared_memory()
{
    // --- Variaveis usada para memoria compartilhada -----
    int shmid ; // identificador da memoria comum //
    const unsigned int size = 4048000; // tamanho da memória em Bytes
    int flag = 0;
    //-----------------------------------------------------

     // Recuperando ou criando uma memoria compartilhada-------------------
     //

     //shmget:para criar um segmento de memória compartilhada
     if (( shmid = shmget((key_t)KEY, size,0)) == -1)
     {
          perror("Erro no shmget") ;
          printf("\nMemória será criada\n");
         //return(1) ;
        if (( shmid = shmget((key_t)KEY, size, IPC_CREAT|IPC_EXCL|SHM_R|SHM_W)) == -1)
        {
            perror("Erro no shmget") ;
            return(1) ;
        }

     }
#ifdef DEBUG
     printf("Sou o processo com pid: %d \n",getpid()) ;
     printf("Identificador do segmento recuperado: %d \n",shmid) ;
     printf("Este segmento e associado a chave unica: %d\n",(key_t)KEY);
#endif
    //
    // acoplamento do processo a zona de memoria
    //recuperacao do pornteiro sobre a area de memoria comum
    //
    //shmat:retorna um pointeiro para o segmento de memória compartilhada
     if ((mem = (int*)shmat (shmid, 0, flag)) == (int*)-1){
          perror("acoplamento impossivel") ;
          return (2) ;
     }
     //---------------------------------------------------------------------

            /* destruicao do segmento */
            //if ((shmctl(shmid, IPC_RMID, NULL)) == -1){
            // perror("Erro shmctl()");
             // return(1) ;
            //}
    return(0);
}


