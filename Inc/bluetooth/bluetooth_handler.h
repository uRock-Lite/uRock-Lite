#ifndef BLUETHOOTH_HANDLER
#define BLUETHOOTH_HANDLER
int bluetooth_handler_init();
/*Private*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
#endif
