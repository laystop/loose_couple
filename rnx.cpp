//
// Created by eureka on 4/21/21.
//
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "global.h"
#include "rnx.h"
#include "common.h"



void initialize() {
	int isat, isys;
	memset(&CKF, 0, sizeof(CKDCFG));
	strcpy(SYS, "GCRESJI");
	strcpy(OBSTYPE, "PCWIXSAQLDBYMZNX ");      //OBSTYPE priority
	// initialize the cprn
//    CKF.nprn = 32 + 61;
	CKF.nprn = 32;

	for (isat = 0; isat < 32; isat++) {
		sprintf(CKF.cprn[isat], "G%02d", isat + 1);
	}

	//    for (isat = 0; isat < 61; isat++) {
	//        sprintf(CKF.cprn[isat+32], "C%02d", isat + 1);
	//    }


	for (isys = 0; isys < MAXSYS; isys++) {
		switch (SYS[isys]) {
		case 'G':
			strcpy(CKF.freq[isys][0], "L1");
			strcpy(CKF.freq[isys][1], "L2");
			CKF.nfreq[isys] = 2;
			break;
		case 'C':
			strcpy(CKF.freq[isys][0], "L2");
			strcpy(CKF.freq[isys][1], "L6");
			CKF.nfreq[isys] = 2;
		default:
			break;
		}
	}
}

void fillobs(char *line, int nobs, int itemlen, double ver) {
	int OFFSET = 0, len, i;
	char tmp[256];
	if (ver > 3.0)
		OFFSET = 3;     //ex. G01 at the start. take over 3 character
	len = len_trim(line);
	for (i = len; i < itemlen * nobs + OFFSET; i++)
		line[i] = ' ';
	line[itemlen * nobs + OFFSET] = '\0';
	for (i = 0; i < nobs; i++) {
		memset(tmp, 0, sizeof(char) * 256);
		substringEx(tmp, line + OFFSET + i * itemlen, 0, itemlen - 2); // last is signal strength
		if (len_trim(tmp) == 0) {
			line[itemlen * i + OFFSET] = '0';
		}
	}
}


void read_rnxobs_head(RNXHEAD *HD, FILE *fp) {
	int i, j, id, lastpos;
	char line[LEN_STRING] = { '\0' };
	char label[20];   //header label(Col 61-80)
	char varword[LEN_STRING];    //description

	//initialize
	HD->nsys = 0;
	memset(HD->nobstype, 0, sizeof(HD->nobstype));
	memset(HD->obstype, 0, sizeof(HD->obstype));

	//set max header lines 200, to void case that file broken, no EOH
	for (int header_line_counter = 0; header_line_counter < 2000; header_line_counter++) {
		lastpos = ftell(fp);
		fgets(line, LEN_STRING, fp);
		substringEx(label, line, 60, 20);

		if (strstr(label, "END OF HEADER") != NULL)
			break;

		if (strstr(label, "RINEX VERSION") != NULL) {
			substringEx(varword, line, 0, 9);
			HD->ver = strtod(varword, NULL);

			substringEx(varword, line, 40, 1);
			HD->sys[0] = varword[0];        //only the first char
			HD->sys[1] = '\0';
			if (HD->sys[0] == ' ')
				HD->sys[0] = 'G';

			if (HD->ver < 3.0 || HD->ver > 4.0)
				printf("rinex version error!(only support for RINEX3.x)\n");
		}

		if (strstr(label, "SYS / # / OBS TYPES") != NULL) {
			id = index_string(SYS, line[0]);
			if (-1 == id)
				continue;       //not included in SYS

			substringEx(varword, line, 3, 3);
			HD->nobstype[id] = atoi(varword);
			int nline = 0;
			if (HD->nobstype[id] % 13 != 0)     //max obs type number = 13 in a line
				nline = (int)(HD->nobstype[id] / 13) + 1;
			else
				nline = (int)(HD->nobstype[id] / 13);

			fseek(fp, lastpos - ftell(fp), SEEK_CUR);   //back to line start

			for (j = 0; j < nline; j++) {
				memset(line, 0, sizeof(line));
				fgets(line, LEN_STRING, fp);
				int nobs_left = HD->nobstype[id] - 13 * j;
				for (i = 0; i < (MIN(nobs_left, 13)); i++) {
					substringEx(varword, line, 7 + i * 4, 3);
					strcpy(HD->obstype[id][i + 13 * j], varword);
				}
			}
			for (i = HD->nobstype[id]; i < MAXOBSTYPE; i++)         //item left in obstype[id][i][]
				HD->obstype[id][i][0] = '\0';
		}

		//label below can be ignored to speed up(though time saving is little)
		//get it back if information needed
		//ignored by Junqiang Lee, 22, Apr, 2021
//        if (strstr(label, "MARKER NAME") != NULL) {
//            substringEx(varword, line, 0, 4);
//            strncpy(HD->mark, varword, 4);
//        }
//
//        if (strstr(label, "REC #") != NULL) {
//
//            substringEx(varword, line, 0, 20);
//            strncpy(HD->recnum, varword, 20);
//            substringEx(varword, line, 20, 20);
//            strncpy(HD->rectype, varword, 20);
//
//        }
//
//        if (strstr(label, "ANT #") != NULL) {
//            substringEx(varword, line, 0, 20);
//            strncpy(HD->antnum, varword, 20);
//            substringEx(varword, line, 20, 20);
//            strncpy(HD->anttype, varword, 20);
//        }
//
//        if (strstr(label, "APPROX POSITION") != NULL) {
//            sscanf(line, "%lf%lf%lf", &HD->x, &HD->y, &HD->z);
//        }
//
//        if (strstr(label, "ANTENNA: DELTA") != NULL) {
//            sscanf(line, "%lf%lf%lf", &HD->h, &HD->e, &HD->n);
//        }
//
//        if (strstr(label, "TYPES OF OBSERV") != NULL) {
//
//            substringEx(varword, line, 0, 6);
//            HD->nobstype[0] = atoi(varword);
//
//            if (HD->nobstype[0] == 0)
//                continue;
//            int nline = 0;
//            if (HD->nobstype[0] % 9 != 0)
//                nline = (int) (HD->nobstype[0] / 9) + 1;
//            else
//                nline = (int) (HD->nobstype[0] / 9);
//
//            fseek(fp, lastpos - ftell(fp), SEEK_CUR);
//            for (j = 0; j < nline; j++) {
//                memset(line, 0, sizeof(line));
//                fgets(line, LEN_STRING, fp);
//                for (i = 0; i < (MIN((HD->nobstype[0] - 9 * j), 9)); i++) {
//                    substringEx(varword, line, 6 + 6 * i, 6);
//                    strcpy(HD->obstype[0][i + 9 * j], left_justify_string(varword));
//                }
//            }
//            for (i = HD->nobstype[0]; i < MAXOBSTYPE; i++)
//                HD->obstype[0][i][0] = '\0';
//
//            for (i = 1; i < MAXSYS; i++) {
//                HD->nobstype[i] = HD->nobstype[0];
//                for (j = 0; j < HD->nobstype[0]; j++)
//                    strcpy(HD->obstype[i][j], HD->obstype[0][j]);
//            }
//
//            //for whu's tracking stations,the C1 is C2,C2 is C7
//            id = index_string(SYS, 'C');
//            if (-1 != id) {
//                for (i = 0; i < HD->nobstype[id]; i++) {
//                    if ('2' == HD->obstype[id][i][1])
//                        HD->obstype[id][i][1] = '7';
//                    if ('1' == HD->obstype[id][i][1])
//                        HD->obstype[id][i][1] = '2';
//                }
//            }
//        }
//
//        if (strstr(label, "INTERVAL") != NULL) {
//            substringEx(varword, line, 0, 16);
//            HD->intv = atof(varword);
//        }
//
//        if (strstr(label, "TIME OF FIRST OBS") != NULL) {
//            double sec;
//            char tmetag[256] = {0};
//            sscanf(line, "%d%d%d%d%d%lf%s", HD->t0, HD->t0 + 1, HD->t0 + 2, HD->t0 + 3,
//                   HD->t0 + 4, &sec, tmetag);
//            HD->t0[5] = (int) sec;
//        }
//
//        if (strstr(label, "TIME OF LAST OBS") != NULL) {
//            for (i = 0; i < 5; i++) {
//                substringEx(varword, line, 0 + i * 6, 6);
//                HD->t1[i] = atoi(varword);
//            }
//            substringEx(varword, line, 30, 13);
//            HD->t1[5] = (int) atof(varword);
//        }
//
//        if (strstr(label, "COMMENT") != NULL)
//            continue;

//        memset(line, 0, sizeof(line));
//        memset(label, 0, sizeof(label));
	}

	for (i = 0; i < MAXSYS; i++)
		if (HD->nobstype[i] != 0)
			HD->nsys++;
}


int read_rnxobs(FILE *fp, RNXHEAD *HD, RNXOBS *OB) {
	char line[MAXCOL] = { '\0' };     //each line
	char varword[LEN_STRING] = { '\0' };
	char code[LEN_OBSTYPE] = { '\0' };
	int nprn = 0, iflag = 0, i, j, k, isat;
	int iy, im, id, ih, imi, lastpos;
	double sec, ds;
	double obs[MAXOBSTYPE];

	//1. initialize
	char prn[MAXSAT][LEN_PRN];
	for (isat = 0; isat < MAXSAT; isat++)
		strncpy(prn[isat], "\0", LEN_PRN);  //initialize prn

	for (i = 0; i < CKF.nprn; i++) {
		for (j = 0; j < 4 * MAXFREQ; j++)
			OB->obs[i][j] = 0.0;    //initialize obs
	}


	//2. get epoch line
	lastpos = ftell(fp);
	fgets(line, LEN_STRING, fp);
	if (feof(fp))       //end of file
		return -1;
	if (len_trim(line) == 0)    //empty line
		return -1;

	substringEx(varword, line, 32, 3);  //number of satellite
	nprn = atoi(varword);
	if (nprn > MAXSAT) {
		printf("satellite number > MAXSAT!(set as {:d} \n", MAXSAT);
		exit(1);
	}

	substringEx(varword, line, 29, 3);  //epoch flag
	iflag = atoi(varword);

	sscanf(line, "%*s%d%d%d%d%d%lf", &iy, &im, &id, &ih, &imi, &sec);   //epoch time
	yr2year(&iy);
	OB->jd = modified_julday(iy, im, id);
	OB->tsec = ROUNDUP(ih * 3600.0 + imi * 60.0 + sec);

	//change start and end epoch time
	if (HD->t0[0] == 0) {
		HD->t0[0] = iy;
		HD->t0[1] = im;
		HD->t0[2] = id;
		HD->t0[3] = ih;
		HD->t0[4] = imi;
		HD->t0[5] = (int)sec;
	}
	HD->t1[0] = iy;
	HD->t1[1] = im;
	HD->t1[2] = id;
	HD->t1[3] = ih;
	HD->t1[4] = imi;
	HD->t1[5] = (int)sec;


	//3. read obs data under the epoch line
	char cprn[MAXSAT][LEN_PRN];
	for (i = 0; i < MAXSAT; i++) {
		cprn[i][0] = 0;
	}

	isat = -1;
	int isys, iobs;
	for (i = 0; i < nprn; i++) {
		memset(line, 0, sizeof(char) * LEN_STRING);
		fgets(line, LEN_STRING, fp);

		//3.1 index of systems
		isys = index_string(SYS, line[0]);
		if (isys == -1)
			continue;   //not included in the SYS

		//3.2 index and prn of satellite
		if (line[1] == ' ')
			line[1] = '0';          //ex. G 1->G01
		memset(varword, 0, sizeof(varword));
		substringEx(varword, line, 0, 3);
		isat = pointer_string(CKF.nprn, LEN_PRN, *(CKF.cprn), varword);
		// if isat equals -1 then the observations is totally disordered, satellite not included
		if (isat == -1)
			continue;
		strcpy(cprn[isat], varword);

		//3.3 parse obs
		//3.3.1 get obs data in obs[](local array)
		substringEx(varword, line, 3, strlen(line) - 3);
		fillobs(varword, HD->nobstype[isys], 16, HD->ver);  //F14.3 + I1 + I1 = 16
		strcpy(line + 3, varword);
		memset(obs, 0, sizeof(double) * MAXOBSTYPE);
		for (j = 0; j < HD->nobstype[isys]; j++) {
			substringEx(varword, line, 3 + j * 16, 14);     //TODO  ignored LLI and SSI
			obs[j] = strtod(varword, NULL);
		}


		//3.3.2 get code type and store obs in OBS(global)
		for (j = 0; j < CKF.nfreq[isys]; j++) {
			OB->obs[isat][j] = 0.0;
			OB->obs[isat][j + MAXFREQ] = 0.0;
			OB->obs[isat][j + 2 * MAXFREQ] = 0.0;
			OB->obs[isat][j + 3 * MAXFREQ] = 0.0;       //LCDS
			// remember to initialize the HD->usetype

			//haven't get the code type for sys and freq
			if (!OB->lstored[isys][j]) {
				// because of the \0
				for (iobs = 0; iobs < strlen(OBSTYPE); iobs++) {
					//find phase
					code[0] = 'L';
					code[1] = CKF.freq[isys][j][1];
					code[2] = OBSTYPE[iobs];    //OBSTYPE show priority here
					code[3] = '\0';
					//k: code index, respond with renix header
					k = pointer_string(HD->nobstype[isys], LEN_OBSTYPE,
						(char *)HD->obstype[isys], code);
					if (k != -1) {
						if (fabs(obs[k]) > 1.0) {
							OB->obs[isat][j] = obs[k];
							strcpy(OB->fob[isat][j], code);
						}
					}

					//find pseudorange
					code[0] = 'C';
					k = pointer_string(HD->nobstype[isys], LEN_OBSTYPE,
						(char *)HD->obstype[isys], code);
					if (k != -1) {      //found
						if (fabs(obs[k]) > 1.0) {
							OB->obs[isat][j + MAXFREQ] = obs[k];
							strcpy(OB->fob[isat][j + MAXFREQ], code);
						}
					}

					//same struct as L, C
					code[0] = 'D';
					k = pointer_string(HD->nobstype[isys], LEN_OBSTYPE,
						(char *)HD->obstype[isys], code);
					if (k != -1) {      //found
						if (fabs(obs[k]) > 1.0) {
							OB->obs[isat][j + 2 * MAXFREQ] = obs[k];
							strcpy(OB->fob[isat][j + 2 * MAXFREQ], code);
						}
					}

					code[0] = 'S';
					k = pointer_string(HD->nobstype[isys], LEN_OBSTYPE,
						(char *)HD->obstype[isys], code);
					if (k != -1) {      //found
						if (fabs(obs[k]) > 1.0) {
							OB->obs[isat][j + 3 * MAXFREQ] = obs[k];
							strcpy(OB->fob[isat][j + 3 * MAXFREQ], code);
						}
					}

					//??? L and C use same code for sure?
					if (OB->obs[isat][j] != 0 && OB->obs[isat][MAXFREQ + j] != 0)
						break;      //L and C already get
				}

				//find the correct obs type for sys and freq
				if (iobs < strlen(OBSTYPE)) {
					HD->usetype[isys][j] = OBSTYPE[iobs];
					OB->lstored[isys][j] = true;
				}
			}
			//get the code type for sys and freq
			//the difference between bellow and above code
			//OBSTYPE - - - usetype, no loop
			//maybe can be deleted
			else {
				//??? can find code for sure?
				code[0] = 'L';
				code[1] = CKF.freq[isys][j][1];
				code[2] = HD->usetype[isys][j];
				code[3] = '\0';
				k = pointer_string(HD->nobstype[isys], LEN_OBSTYPE,
					(char *)HD->obstype[isys], code);
				if (fabs(obs[k]) > 1.0) {
					OB->obs[isat][j] = obs[k];
					strcpy(OB->fob[isat][j], code);
				}

				code[0] = 'C';
				k = pointer_string(HD->nobstype[isys], LEN_OBSTYPE,
					(char *)HD->obstype[isys], code);
				if (fabs(obs[k]) > 1.0) {
					OB->obs[isat][MAXFREQ + j] = obs[k];
					strcpy(OB->fob[isat][MAXFREQ + j], code);
				}

				code[0] = 'D';
				k = pointer_string(HD->nobstype[isys], LEN_OBSTYPE,
					(char *)HD->obstype[isys], code);
				if (fabs(obs[k]) > 1.0) {
					OB->obs[isat][2 * MAXFREQ + j] = obs[k];
					strcpy(OB->fob[isat][2 * MAXFREQ + j], code);
				}

				code[0] = 'S';
				k = pointer_string(HD->nobstype[isys], LEN_OBSTYPE,
					(char *)HD->obstype[isys], code);
				if (fabs(obs[k]) > 1.0) {
					OB->obs[isat][3 * MAXFREQ + j] = obs[k];
					strcpy(OB->fob[isat][3 * MAXFREQ + j], code);
				}
			}
		}


		//no available obs, no right code included
		if (OB->obs[isat][0] == 0 || OB->obs[isat][MAXFREQ + 0] == 0) {
			OB->obs[isat][0] = 0.0;
			memset(OB->fob[isat][0], 0, sizeof(char) * LEN_OBSTYPE);

			OB->obs[isat][MAXFREQ] = 0.0;
			memset(OB->fob[isat][MAXFREQ], 0, sizeof(char) * LEN_OBSTYPE);
		}
	}

	OB->nprn = nprn;        //actual number of satellites
	for (i = 0; i < CKF.nprn; i++)
		strcpy(OB->cprn[i], cprn[i]);

	return 0;
}


/* Read GNSS NAVIGATION MESSAGE FILE, RENIX
 *
 * Param:
 * csys -> system, 'G' for GPS, 'M' for 'Mixed';
 * flnbrd -> file name brdc
 * */
void read_rnxnav(char csys, const char *flnbrd, double mjd0, double mjd1, BRDHEAD *hd,
	int *neph, GPS_BRDEPH ephm[MAXSYS][MAXEPH]) {
	FILE *fp;
	int i, k, isys, iy, im, id, ih, imin, lastpos, len;
	char line[LEN_STRING], buf[LEN_STRING];
	char *ptr;
	double dt0, dt1, isec;
	double data1, data2, data3;
	int already;

	GPS_BRDEPH eph = { 0 };

	if ((fp = fopen(flnbrd, "r")) == NULL) {
		printf("***ERROR(read_rnxnav):cant open brdc file %s\n", flnbrd);
		exit(1);
	}

	memset(neph, 0, sizeof(int) * MAXSYS);
	strncpy(line, "\0", LEN_STRING);

	//Read RENIX Header
	while (strstr(line, "END OF HEADER") == NULL) {
		strncpy(line, "\0", LEN_STRING);
		fgets(line, LEN_STRING, fp);
		if (strstr(line, "RINEX VERSION / TYPE") != NULL)
			sscanf(line, "%lf", &hd->ver);

		if (strstr(line, "LEAP SECONDS") != NULL)
			sscanf(line, "%d", &hd->leap);

		//label below can be ignored to speed up(though time saving is little)
		if (strstr(line, "IONOSPHERIC CORR") != NULL) {
			if (!strncmp(line, "GPS ", 4) || !strncmp(line, "GPSA", 4)) {
				i = index_string(SYS, 'G');
				k = 0;
			}
			else if (!strncmp(line, "GPSB", 4)) {
				i = index_string(SYS, 'G');
				k = 1;
			}
			else if (!strncmp(line, "GAL ", 4)) {
				i = index_string(SYS, 'E');
				k = 0;
			}
			else if (!strncmp(line, "BDS ", 4) || !strncmp(line, "BDSA", 4)) {
				i = index_string(SYS, 'C');
				k = 0;
			}
			else if (!strncmp(line, "BDSB", 4)) {
				i = index_string(SYS, 'C');
				k = 1;
			}
			else if (!strncmp(line, "QZS ", 4) || !strncmp(line, "QZSA", 4)) {
				i = index_string(SYS, 'J');
				k = 0;
			}
			else if (!strncmp(line, "QZSB", 4)) {
				i = index_string(SYS, 'J');
				k = 1;
			}
			else
				printf("###WARNING(read_rnxnav):unknown ionospheric corr!\n");

			strncpy(hd->ionc[i][k], line, 4);
			ptr = line;
			while (*ptr != '\0') {
				if (*ptr == 'D')
					*ptr = 'e';
				ptr++;
			}
			sscanf(line, "%*s%lf%lf%lf%lf", hd->ion[i][k], hd->ion[i][k] + 1,
				hd->ion[i][k] + 2, hd->ion[i][k] + 3);
		}
		if (strstr(line, "DELTA-UTC: A0,A1,T,W") != NULL) {
			i = index_string(SYS, csys);
			if (i == -1) {
				printf(
					"$$$MESSAGE(read_rnxnav):DELTA-UTC: A0,A1,T,W is only valid for single system!\n");
				continue;
			}
			ptr = line;
			while (*ptr != '\0') {
				if (*ptr == 'D')
					*ptr = 'e';
				ptr++;
			}
			sscanf(line, "%lf%lf%lf%lf", hd->tim[i][0], hd->tim[i][0] + 1,
				hd->tim[i][0] + 2, hd->tim[i][0] + 3);
		}
		if (strstr(line, "TIME SYSTEM CORR") != NULL) {
			if (!strncmp(line, "GPUT", 4)) {
				i = index_string(SYS, 'G');
				k = 0;
			}
			else if (!strncmp(line, "GLUT", 4)) {
				i = index_string(SYS, 'R');
				k = 0;
			}
			else if (!strncmp(line, "GAUT", 4)) {
				i = index_string(SYS, 'E');
				k = 0;
			}
			else if (!strncmp(line, "BDUT", 4)) {
				i = index_string(SYS, 'C');
				k = 0;
			}
			else if (!strncmp(line, "SBUT", 4)) {
				i = index_string(SYS, 'S');
				k = 0;
			}
			else if (!strncmp(line, "QZUT", 4)) {
				i = index_string(SYS, 'J');
				k = 0;
			}
			else if (!strncmp(line, "GAGP", 4)) {
				i = index_string(SYS, 'G');
				k = 1;
			}
			else if (!strncmp(line, "GLGP", 4)) {
				i = index_string(SYS, 'R');
				k = 1;
			}
			else if (!strncmp(line, "QZGP", 4)) {
				i = index_string(SYS, 'J');
				k = 1;
			}
			else {
				printf(
					"$$$MESSAGE(read_rnxnav):unknown unknown TIME SYSTEM CORR!\n");
				continue;
			}
			strncpy(hd->timc[i][k], line, 4);
			ptr = line;
			while (*ptr != '\0') {
				if (*ptr == 'D')
					*ptr = 'e';
				ptr++;
			}
			sscanf(line, "%*5c%lf%lf%lf%lf", hd->tim[i][k], hd->tim[i][k] + 1,
				hd->tim[i][k] + 2, hd->tim[i][k] + 3);
		}
	}


	//Read RENIX Body
	while (!feof(fp)) {
		lastpos = ftell(fp);
		fgets(line, LEN_STRING, fp);
		fseek(fp, lastpos - ftell(fp), SEEK_CUR);   //back to line beginning
		memset(&eph, 0, sizeof(eph));

		buf[0] = line[0];
		switch (buf[0]) {
		case 'G':
		case 'C':
			isys = index_string(SYS, buf[0]);
			if (isys == -1) {
				for (i = 0; i < 8; i++)
					fgets(line, LEN_STRING, fp);
				continue;
			}

			len = 0;
			strncpy(buf, "", LEN_STRING);

			//read all brdm contents(8 lines) -> ptr
			for (i = 0; i < 8; i++) {
				fgets(line, LEN_STRING, fp);
				if (i != 7)
					filleph(line, hd->ver);
				strncpy(buf + len, line, strlen(line));
				len += strlen(line);
			}
			ptr = buf;

			//replace char
			while (*ptr != '\0') {
				if (*ptr == '\n' || *ptr == '\r')
					*ptr = ' ';

				if (*ptr == 'D')
					*ptr = 'e';
				ptr++;
			}


			if (buf[0] == 'C') {
				sscanf(buf,
					"%s%d%d%d%d%d%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf",
					eph.cprn, &iy, &im, &id, &ih, &imin, &isec, &eph.a0, &eph.a1, &eph.a2,
					&eph.aode, &eph.crs, &eph.dn, &eph.m0,
					&eph.cuc, &eph.e, &eph.cus, &eph.roota,
					&eph.toe, &eph.cic, &eph.Omega0, &eph.cis,
					&eph.i0, &eph.crc, &eph.omega, &eph.Omega_dot,
					&eph.i_dot, &eph.resvd0, &eph.week, &eph.resvd1,
					&eph.accu, &eph.hlth, &eph.tgd, &eph.aodc,
					&data1, &eph.iodc, &data3, &eph.signal_idx);
				eph.signal_idx = (int)eph.signal_idx == 0 ? 1 : (int)eph.signal_idx;
				if (eph.signal_idx > 6 && eph.signal_idx <= 13) {
					eph.delta_A = eph.roota;
					eph.A_DOT = eph.resvd0;
					eph.delta_n_dot = eph.resvd1;
				}
				if (eph.signal_idx == 1 || eph.signal_idx == 0) { //B1I
					eph.tgd_BDS[0] = eph.tgd;
					eph.isc_BDS[0] = 0.0;
				}
				else if (eph.signal_idx == 2) { //B2I
					eph.tgd_BDS[1] = eph.aodc;
					eph.isc_BDS[1] = 0.0;
				}
				else if (eph.signal_idx == 3) { //B3I
					eph.tgd_BDS[2] = 0.0;
					eph.isc_BDS[2] = 0.0;
				}
				else if (eph.signal_idx == 4) { //B1Q
					eph.tgd_BDS[3] = eph.tgd;
					eph.isc_BDS[3] = 0.0;
				}
				else if (eph.signal_idx == 5) { //B1Q
					eph.tgd_BDS[4] = eph.aodc;
					eph.isc_BDS[4] = 0.0;
				}
				else if (eph.signal_idx == 6) { //B3Q
					eph.tgd_BDS[5] = 0.0;
					eph.isc_BDS[5] = 0.0;
				}
				else if (eph.signal_idx == 7) { //B1C
					eph.tgd_BDS[6] = eph.tgd;
					eph.isc_BDS[6] = data3;
				}
				else if (eph.signal_idx == 8) { //B2a
					eph.tgd_BDS[7] = eph.tgd;
					eph.isc_BDS[7] = data3;
				}
				else if (eph.signal_idx == 9) { //B2bI
					eph.tgd_BDS[8] = eph.tgd;
					eph.isc_BDS[8] = 0.0;
				}
				else if (eph.signal_idx == 10) { //B2bQ
					eph.tgd_BDS[9] = eph.tgd;
					eph.isc_BDS[9] = 0.0;
				}
				else if (eph.signal_idx == 11) { //B1A
					eph.tgd_BDS[10] = eph.tgd;
					eph.isc_BDS[10] = data3;
				}
				else if (eph.signal_idx == 12) { //B3A
					eph.tgd_BDS[11] = 0.0;
					eph.isc_BDS[11] = data3;
				}
				else if (eph.signal_idx == 13) { //B3AE
					eph.tgd_BDS[12] = eph.aodc;
					eph.isc_BDS[12] = data3;
				}
			}
			///GPS
			else {
				sscanf(buf,
					"%s%d%d%d%d%d%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf",
					eph.cprn, &iy, &im, &id, &ih, &imin, &isec, &eph.a0,
					&eph.a1, &eph.a2, &eph.aode, &eph.crs, &eph.dn, &eph.m0,
					&eph.cuc, &eph.e, &eph.cus, &eph.roota, &eph.toe, &eph.cic,
					&eph.Omega0, &eph.cis, &eph.i0, &eph.crc, &eph.omega,
					&eph.Omega_dot, &eph.i_dot, &eph.resvd0, &eph.week,
					&eph.resvd1, &eph.accu, &eph.hlth, &eph.tgd, &eph.aodc);
			}

			if (eph.hlth > 0)
				continue;
			yr2year(&iy);
			if (iy < 2000)
				continue;

			// time of clock
			// TIME IN BDS TIME AND SHOULD NOT CHANGE IT INTO GPST
			// BECAUSE THERE ARE OTHER TIME TAG IN THE EPHEMERIS
			eph.mjd = modified_julday(iy, im, id);
			eph.sod = ih * 3600.0 + imin * 60.0 + isec;


			// adapter to bds
//                if (eph.cprn[0] == 'C') {
//                    //the week in broadcast file generated by WHU is GPS week,
//                    //but that in IGS meraged file is BDS week
//                    mjd2wksow(eph.mjd, eph.sod, &k, &dt1);
//                    if (k != (int) eph.week)
//                        eph.week = 1356 + eph.week;
//                    eph.tgd1 = eph.aodc;
//                }

				//check time
			dt0 = 0.0;
			dt1 = 0.0;
			if (mjd0 != 0)
				dt0 = eph.mjd + eph.sod / 86400.0 - mjd0;
			if (mjd1 != 0)
				dt1 = eph.mjd + eph.sod / 86400.0 - mjd1;
			if (dt0 < -1.0 / 24.0 || dt1 > 1.0 / 24.0)
				continue;

			already = false;
			for (i = 0; i < neph[isys]; i++) {
				if (strstr(ephm[isys][i].cprn, eph.cprn) && ephm[isys][i].mjd == eph.mjd &&
					ephm[isys][i].sod == eph.sod) {
					already = true;
					break;
				}
			}
			if (!already) {
				neph[isys] = neph[isys] + 1;
				if (neph[isys] > MAXEPH) {
					printf("ERROR(read_rnxnav):exceed the maxium ephemeris number!\n");
					exit(1);
				}
				memcpy(&ephm[isys][neph[isys] - 1], &eph, sizeof(eph));
			}
			break;

		default:
			fgets(line, LEN_STRING, fp);
			break;
		}
	}
	fclose(fp);
}


void filleph(char *line, double ver) {
	int i, len, cpre = 4, ncount;
	char tmp[128];
	char xline[1024];
	if (ver < 3.0)
		cpre = 3;
	len = len_trim(line);
	for (i = len; i < cpre + 19 * 4; i++) {
		line[i] = ' ';
	}
	line[cpre + 19 * 4] = '\0';
	for (i = 0; i < 4; i++) {
		ncount = cpre + 19 * (i + 1);
		if (strlen(line) < ncount) {
			line[cpre + 19 * i + 1] = '0';
		}
		else {
			substringEx(tmp, line + cpre + 19 * i, 0, 19);
			if (len_trim(tmp) == 0) {
				line[cpre + 19 * i + 1] = '0';
			}
		}
	}
}

