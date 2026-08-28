#include "Diag_List.h"
#include "Motor_Control.h"

#define NULL ((void *)0)

void Diag_List_Register(Diag_Node_t **head, Diag_Node_t *node, void (*update)(Diag_Node_t *node))
{
    node->update = update;
    node->next = *head;
    *head = node;
}

void Diag_List_Traverse(Diag_Node_t *head, Motor_Control_t *pControl)
{
    Diag_Node_t *node = head;
    while (node != NULL)
    {
        if (node->update != NULL)
        {
            node->update(node, pControl);
        }
        node = node->next;
    }
}
