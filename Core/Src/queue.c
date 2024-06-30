//
// Created by begre on 2024-06-18.
//


// ����2�����˷Ѵ洢�ռ�
// ����һ������size����¼���е��д洢Ԫ�صĸ���
// ��βָ��ָ�� ��βԪ�صĺ�һ��λ�ã���һ��Ӧ�ò����λ�ã�

#include "queue.h"


// ��ʼ������
void InitQueue(SeqQueue *q) {
    q->front = q->rear = 0;
    q->size = 0;        //���е�ǰ����Ϊ0
}

// �ж϶����Ƿ�Ϊ��
uint8_t QueueEmpty(SeqQueue *q) {
    if (q->size == 0)    //�ӿ�����
        return true;
    else
        return false;
}

// ���
uint8_t EnQueue(SeqQueue *q, ElemType x) {
    if (q->size == MaxSize)
        DeQueue(q);

    q->data[q->rear] = x;        //��x�����β
    q->rear = (q->rear + 1) % MaxSize;    //��βָ�����
    q->size++;
    return true;
}

// ����
void DeQueue(SeqQueue *q) {
    if (q->size == 0) return;

    q->front = (q->front + 1) % MaxSize; //��ͷָ�����
    q->size--;
    return;
}

// ��ȡ��ͷԪ��
uint8_t GetHead(SeqQueue *q, ElemType *x) {
    if (q->size == 0)
        return false;    //�ӿ��򱨴�

    *x = q->data[q->front];
    return true;
}

// ������Ԫ�صĸ���
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
