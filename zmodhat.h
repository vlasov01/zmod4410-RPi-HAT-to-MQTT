/**
 * @file   zmodhat.h
 * @brief  HAL for ZMOD4410 Indoor Air Quality Raspberry Pi Hat header
 * @version 0.2.0
 *
 * cloned from https://github.com/sicreative/zmod_airquality/blob/main/qt/zmodhat.h
 * modified  by Sergey Vlasov
 */

#ifndef ZMODHAT_H
#define ZMODHAT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "iaq_2nd_gen.h"

typedef struct {
  bool valid;
  float temperature;
  float humidity;
} hs300x_result;

int i2c_closeport(void);
int i2c_openport(void);
int zmodhat_init(void);
int8_t zmodhat_start_meassurement();
int8_t zmodhat_read_meassurement();
iaq_2nd_gen_results_t *zmodhat_get_iaq_result();
float zmodhat_get_iaq_stabilization();
int8_t zmodhat_hs300x_start_measurement();
hs300x_result zmodhat_hs300x_read_measurement();
int8_t zmodhat_tracking_num(uint8_t *tracking_number);
void read_zmod_hat(char *ibuf);

#ifdef __cplusplus
}
#endif

#endif // ZMODHAT_H
