/*--------------------------------------------------------------------

******************************************************************************
* @file blackboard.h
* @author Isaac Jesus da Silva - ROBOFEI-HT - FEI
* @version V0.0.0
* @created 07/04/2014
* @Modified 26/05/2015
* @e-mail isaac25silva@yahoo.com.br
* @brief Main header black board
****************************************************************************

Arquivo de cabeçalho contendo as funções e definições do black board

/--------------------------------------------------------------------*/

#ifndef BLACKBOARD_H
#define BLACKBOARD_H

#define KEY 10341678 //Chave de acesso a memoria

//---- Definições da memória compartilhada------------------------------

//----global variables------------------------------------------------
extern int *mem ; //Variável que manipula memória compartilhada

//----Functions prototype---------------------------------------------
int using_shared_memory(); //Função que cria e acopla a memória compartilhada

#endif

