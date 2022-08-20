
/*SPP Algorithm*/

#ifndef MYRTK_SPP_H
#define MYRTK_SPP_H
//#include "rnx.h"
#include "common.h"
#include "resform.h"

void spp_func_model(const Station &sit, const GnssDataArr gnss_d,
	Ref<MatrixXd> H, Ref<VectorXd> z);

void spp(Station &sit, const GnssDataArr gnss_d, GnssRes &gnss_res);

void spv(Station &sit, const GnssDataArr gnss_d, GnssRes &gnss_res);
#endif //MYRTK_SPP_H