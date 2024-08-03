#ifndef __UART_H__
#define __UART_H__
#include <stdint.h>

#include "control.h"


class UartConn
{
public:
	UartConn(Control * control0,Control * control1);
	void update(uint8_t rx_dat);
private:
	void do_uart();
	uint8_t buff[128] = {0};
	int8_t buff_len = 0;
	int8_t dat_len = 0;
	uint8_t state = 0;
	Control * control0 = nullptr;
	Control * control1 = nullptr;
};


#endif
