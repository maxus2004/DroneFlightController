enum state_t {
	STARTING = 0,
	LANDED,
	FLYING,
	AUTO_LANDING,
	EMERGENCY
};

enum error_t {
	OK = 0,
	TOO_HIGH_THRUST_ON_STARTUP,
	TOO_HIGH_THRUST_CHANGE,
	ORIENTATION_FAIL
};

extern enum state_t STATE;
extern enum error_t ERROR_ID;