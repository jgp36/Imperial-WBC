


typedef struct _visinfo {
	double q[6];
	double R[1];
	double T[1];
	double cp[3];
	double eedes[3];
	double sp[5][3];//Assuming 5 markers, there must be a way to have a more general setup here
} visinfo;