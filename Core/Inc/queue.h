//
// Created by begre on 2024-06-18.
//

#ifndef FOC_DEMO_QUEUE_H
#define FOC_DEMO_QUEUE_H

#include "main.h"

#define MaxSize 128

typedef uint32_t ElemType;
typedef struct {
    ElemType data[MaxSize];
    int front;
    int rear;
    int size;    //���е�ǰ����
} SeqQueue;

void InitQueue(SeqQueue *q);

// �ж϶����Ƿ�Ϊ��
uint8_t QueueEmpty(SeqQueue *q);

// ���
uint8_t EnQueue(SeqQueue *q, ElemType x);

// ����
void DeQueue(SeqQueue *q);

// ��ȡ��ͷԪ��
uint8_t GetHead(SeqQueue *q, ElemType *x);

// ������Ԫ�صĸ���
int QueueNum(SeqQueue *q);
ElemType mean(SeqQueue *q);
#endif //FOC_DEMO_QUEUE_H
