enum state_t {
	STARTING,
	LANDED,
	FLYING,
	AUTO_LANDING,
	EMERGENCY
};

enum error_t {
	OK = 0,
	TOO_HIGH_THRUST_ON_STARTUP = 1,
	TOO_HIGH_THRUST_CHANGE = 2
};

extern enum state_t STATE;
extern enum error_t ERROR_ID;