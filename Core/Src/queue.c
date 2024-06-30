//
// Created by begre on 2024-06-18.
//


// 方案2：不浪费存储空间
// 设置一个变量size，记录队列当中存储元素的个数
// 队尾指针指向 队尾元素的后一个位置（下一个应该插入的位置）

#include "queue.h"


// 初始化队列
void InitQueue(SeqQueue *q) {
    q->front = q->rear = 0;
    q->size = 0;        //队列当前长度为0
}

// 判断队列是否为空
uint8_t QueueEmpty(SeqQueue *q) {
    if (q->size == 0)    //队空条件
        return true;
    else
        return false;
}

// 入队
uint8_t EnQueue(SeqQueue *q, ElemType x) {
    if (q->size == MaxSize)
        DeQueue(q);

    q->data[q->rear] = x;        //将x插入队尾
    q->rear = (q->rear + 1) % MaxSize;    //队尾指针后移
    q->size++;
    return true;
}

// 出队
void DeQueue(SeqQueue *q) {
    if (q->size == 0) return;

    q->front = (q->front + 1) % MaxSize; //队头指针后移
    q->size--;
    return;
}

// 获取队头元素
uint8_t GetHead(SeqQueue *q, ElemType *x) {
    if (q->size == 0)
        return false;    //队空则报错

    *x = q->data[q->front];
    return true;
}

// 队列中元素的个数
int QueueNum(SeqQueue *q) {
    return q->size;
}

ElemType mean(SeqQueue *q) {
    int i = q->front;
    int size_i = 0;
    ElemType sum = 0;

    while (size_i++ < q->size) {
        sum += q->data[i];
        i = (i + 1) % MaxSize;
    }
    return (sum >> 7);
}
