//
// Created by eureka on 6/6/21.
//
#include <string.h>
#include "global.h"
#include "common.h"

void rtk_init(const char *pfilcfg, char *pfilbrdm,
	char *pfilobs_b, char *pfilobs_r, char *pfilres) {
	int isat, isys;
	memset(&CKF, 0, sizeof(CKDCFG));
	char line[150] = { '\0' };     //each line
	char varword[150] = { '\0' };

	FILE *fpcfg = fopen(pfilcfg, "r");
	if (!fpcfg) {
		printf("can't open config file to read!\n");
		exit(1);
	}

	int iyear, imon, iday, ihour, imin;
	double sec, tlen;
	while (!feof(fpcfg)) {
		fgets(line, LEN_STRING, fpcfg);
		if (line[0] == '#') continue;

		if (strstr(line, "CPU Intel accelerator") != NULL) {
			sscanf(line, "%*[^=]= %d",
				&(CKF.cpu_intel));
			continue;
		}

		if (strstr(line, "start time&session length") != NULL) {
			sscanf(line, "%*[^=]= %d%d%d%d%d%f%f",
				&iyear, &imon, &iday,
				&ihour, &imin, &sec, &tlen);
			continue;
		}

		if (strstr(line, "brdc file path") != NULL) {
			sscanf(line, "%*[^=]= %s", pfilbrdm);
			len_trim(pfilbrdm);
			continue;
		}

		if (strstr(line, "rover file path") != NULL) {
			sscanf(line, "%*[^=]= %s", pfilobs_r);
			len_trim(pfilobs_r);
			continue;
		}

		if (strstr(line, "base file path") != NULL) {
			sscanf(line, "%*[^=]= %s", pfilobs_b);
			len_trim(pfilobs_b);
			continue;
		}

		if (strstr(line, "baseline res file path") != NULL) {
			sscanf(line, "%*[^=]= %s", pfilres);
			len_trim(pfilres);
			continue;
		}

		if (strstr(line, "used system") != NULL) {
			sscanf(line, "%*[^=]= %s", varword);
			len_trim(varword);
			strcpy(SYS, varword);
			continue;
		}

		if (strstr(line, "observable type") != NULL) {
			sscanf(line, "%*[^=]= %s", varword);
			len_trim(varword);
			strcpy(OBSTYPE, varword);
			continue;
		}
	}

	CKF.mjd = (int)modified_julday(iyear, imon, iday);
	//    CKF.sod = 0.0;

		// initialize the cprn
	if (strstr(SYS, "C") != NULL) {
		CKF.nsys = 2;
		CKF.nprn = 32 + 61;
		for (isat = 0; isat < 61; isat++)
			sprintf(CKF.cprn[isat + 32], "C%02d", isat + 1);
	}
	else {
		CKF.nsys = 1;
		CKF.nprn = 32;
	}
	for (isat = 0; isat < 32; isat++)
		sprintf(CKF.cprn[isat], "G%02d", isat + 1);

	for (isys = 0; isys < MAXSYS; isys++) {
		switch (SYS[isys]) {
		case 'G':
			strcpy(CKF.freq[isys][0], "L1");
			strcpy(CKF.freq[isys][1], "L2");
			CKF.nfreq[isys] = 2;
			CKF.ffreq[isys][0] = FREQ1;
			CKF.ffreq[isys][1] = FREQ2;
			break;
		case 'C':
			strcpy(CKF.freq[isys][0], "L2");
			strcpy(CKF.freq[isys][1], "L6");
			CKF.nfreq[isys] = 2;
			CKF.ffreq[isys][0] = FREQ1_CMP;
			CKF.ffreq[isys][1] = FREQ3_CMP;
		default:
			break;
		}
	}
}
