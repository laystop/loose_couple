//
// Created by eureka on 6/22/21.
//

#ifndef MYRTK_GNSS_H
#define MYRTK_GNSS_H


#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

//#define GLOBAL_VAR

#include "rnx.h"
#include "global.h"
#include "preprocess.h"
#include "spp.h"
#include "rtk.h"
#include "resform.h"


#define INIT_NUM_EPO 16384  //2^14


void gnss_pv(vector<GnssRes> &gnss_res_vec);


#endif //MYRTK_GNSS_H
