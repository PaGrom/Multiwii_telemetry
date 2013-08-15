typedef struct {
   int ax;		// Accelerometer data X axis (Roll)
   int ay;		// Accelerometer data Y axis (Pitch)
   int az;		// Accelerometer data Z axis (Yaw)
   int gx;		// Gyroscope data X axis (Roll)
   int gy;		// Gyroscope data Y axis (Pitch)
   int gz;		// Gyroscope data Z axis (Yaw)
   int magx;	// Magnetometer data X axis (Roll)
   int magy;	// Magnetometer data Y axis (Pitch)
   int magz;	// Magnetometer data X axis (Yaw)
   float alt;	// Altitude
   int head;	// Attitude
} telem;

int set_interface_attribs(int fd, int speed, int parity);
void set_blocking(int fd, int should_block);

int read_ax(char *buf);
int read_ay(char *buf);
int read_az(char *buf);
int read_gx(char *buf);
int read_gy(char *buf);
int read_gz(char *buf);
int read_magx(char *buf);
int read_magy(char *buf);
int read_magz(char *buf);
float read_alt(char *buf);
int read_head(char *buf);

void read_acc_gyro_mag(int fd);
void read_altitude(int fd);
void read_attitude(int fd);