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
    int size;    //队列当前长度
} SeqQueue;

void InitQueue(SeqQueue *q);

// 判断队列是否为空
uint8_t QueueEmpty(SeqQueue *q);

// 入队
uint8_t EnQueue(SeqQueue *q, ElemType x);

// 出队
void DeQueue(SeqQueue *q);

// 获取队头元素
uint8_t GetHead(SeqQueue *q, ElemType *x);

// 队列中元素的个数
int QueueNum(SeqQueue *q);
ElemType mean(SeqQueue *q);
#endif //FOC_DEMO_QUEUE_H
