#ifndef __DIAG_LIST_H__
#define __DIAG_LIST_H__

#include <stdint.h>
#include "Motor_Control.h"

typedef struct Diag_Node {
    struct Diag_Node *next;
    void (*update)(struct Diag_Node *node, Motor_Control_t *pControl);
} Diag_Node_t;

void Diag_List_Register(Diag_Node_t **head, Diag_Node_t *node, void (*update)(Diag_Node_t *node));
void Diag_List_Traverse(Diag_Node_t *head, Motor_Control_t *pControl);

#endif
