// File: keyboard-jogging-code.c
// Date: Thu 18 May 2023 11:41:23 PM +08
// Origin: /home/wruslan/CNC-MMU-jogging-Sun-06May2010# ./main-cnc-jogging.x 
// 
// ==============================================
// DESCRIPTION: 
// This program drives the CNC servo motors to 
// manually set the CNC spindle starting location.
// 
// This program allows the user to select the 
// direction to move the CNC spindle on-the-run
// in fixed steps of 500 pulses. The directions
// cover y-up, y-down, x-right, x-left, z-up
// and z-down. We use 500 pulses for z-up and z-down.

// This adjustment for the starting location 
// in CNC jargon is called jogging.
//
// This program runs continuously, until the
// key 'q' on the keyboard is pressed. 

// ============================================== 
// COMPILATION AND EXECUTION INSTRUCTIONS
// gcc -o keyboard-jogging-code.cx keyboard-jogging-code.c

// ==============================================
// INCLUDE FILE HEADERS
#include <stdio.h>
#include <stdlib.h>     // Use for exit(0)
#include <time.h>	    // For local date-time with usec
#include <signal.h>     // Signal for User interrupt Ctrl-C to stop 
#include <fcntl.h>
#include <math.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>	// TO HANDLE strlen IN gcode . h
#include <curses.h>	// TO HANDLE getch () , wgetch ()
#include <math.h>
#include <sys/ioctl.h>
#include <sys/io.h>
#include <sys/mman.h>
#include <sys/time.h>	// For local date-time with usec
#include <sched.h>

// HEADERS FOR KEYBOARD_HIT
#include <termios.h>
#include <term.h>
#include <curses.h>

// ==================================================================
// PARALLEL PORT HARDWARE INFORMATION
// EXAMPLE SETTING THE PARALLEL PORT ADDRESS 
// int   PARPORT_ADDRESS = 0x378;   // Set Standard On-board parallel port address
// int   PARPORT_ADDRESS = 0x5010;  // Set Add-on PCMCIA card parallel port address
// USING dmesg | grep par
/*
[    6.839566] ppdev: user-space parallel port driver
[    6.842247] parport_pc 00:05: reported by Plug and Play ACPI
[    6.842333] parport0: PC-style at 0x378 (0x778), irq 5, using FIFO [PCSPP,TRISTATE,COMPAT,EPP,ECP]
[    6.934742] lp0: using parport0 (interrupt-driven).
*/
// ==================================================================
// PARPORT GLOBAL VARIABLES
// ==================================================================
#define PARPORT_DEVICE   "/dev/lp0"
#define PARPORT_ADDRESS  0x378		
#define PARPORT_IRQ      5

#define PERIOD		500000		// nanoseconds
#define TICK_TIME	1000000		// nanoseconds
// #define CPUMAP 	0xF  		// 16-cpus
#define CPUMAP		0x8         // 8-cpus   (HPNotebook ) 
// #define CPUMAP	0x4         // 4-cpus   (Blue-Workstation)


int DATA_REG, STATUS_REG, CONTROL_REG, BASE_ADDRESS; 

int  io_prio_lvl;   // I/O priority level
int  io_perm;		// I/O permissions
int  parport_fd;	// File descriptors

void check_io_priority_level(void); 
void check_io_permission(void);
void open_parallel_port(void);
void close_parallel_port(void);

// ==================================================================
// DATE-TIME HIGH RESOLUTION (nanoseconds)
// ==================================================================
// Used for DTStamp(void) ONLY in order 
// to avoid repetitive declarations (reused) 

time_t 		    WRYtimer;
char 		    WRYbuffer[26];
struct tm* 		WRYtm_info;
struct timeval  WRYtval_now;

void DTStamp(void);

// ==================================================================
// CNC MACHINE JOGGING DATA AND FUNCTIONS
// ===================================================================
static struct termios     initial_settings, new_settings;
static int                peek_character = -1;

//SPEED AND DISTANCE
int delaytime = 1000; // usec microsecond = 1 msec millisecond 
int distance  = 500;

// VARIABLE DECLARATIONS
int count;
 
// FUNCTION PROTOTYPES
void    drive_cnc_machine(void);

// PROTOTYPE FUNCTION DEFINITIONS
void    init_keyboard();
void    close_keyboard();
int     keyboard_hit();
int     read_charkey();
 
// FUNCTION PROTOTYPES DECLARATION
void    reset_CNC(void);
void    cmd_interpreter(int);
void    run_menu(void);


// DRIVE CNC ALONG X-AXIS
// ==================================================================
void    drive_right(int valdrive);        // DRIVE (3,2 CW)  
void    drive_left(int valdrive);         // DRIVE (1,0 CCW)
        
// DRIVE CNC ALONG Y-AXIS 
// ==================================================================
void    drive_forward(int valdrive);      // DRIVE (12,8 CW)  
void    drive_backward(int valdrive);     // DRIVE (4,0, CCW)

// DRIVE CNC ALONG Z-AXIS
// ==================================================================
void    drive_up(int valdrive);          // DRIVE (16,0  CW)
void    drive_down(int valdrive);        // DRIVE (48,32 CCW)


// ==================================================================
void DTStamp(void) {  // High resolution timer Date-Time stamp
// ==================================================================

    time(&WRYtimer);
    WRYtm_info = localtime(&WRYtimer);
    strftime(WRYbuffer, 26, "%Y-%m-%d %H:%M:%S", WRYtm_info);
    gettimeofday(&WRYtval_now, NULL);

    printf("%s", WRYbuffer);
    printf(".%09ld \t", (long int)WRYtval_now.tv_usec);
}

// ========================================================
void check_io_priority_level(void) {
// ========================================================
    if (io_prio_lvl != 0) {
	DTStamp(); printf("ERROR  : Set max priority level io_prio_lvl value \t= %d\n", io_prio_lvl);
	perror("iopl");
    } else {
	DTStamp(); printf("SUCCESS: Set max priority level io_prio_lvl value \t= %d\n", io_prio_lvl);
    }
}
// ========================================================
void check_io_permission(void) {
// ========================================================
    if (io_perm != 0) {
	DTStamp(); printf("ERROR  : Set port i/o permissions io_perm value \t= %d\n", io_perm);
	perror("ioperm");
    } else {
	DTStamp(); printf("SUCCESS: Set port i/o permissions io_perm value \t= %d\n", io_perm);
    }
}
// ========================================================
void open_parallel_port(void) {
// ========================================================
	printf("\n");
	DTStamp(); 
	printf("EXECUTING  open_parallel_port(void).\n");

	if(parport_fd < 0) {
		DTStamp(); printf("ERROR: Cannot open PARPORT_DEVICE (/dev/lp0).\n");
		perror(PARPORT_DEVICE);
		exit(1);
	} else {
		DTStamp(); printf("SUCCESS: Open PARPORT_DEVICE (/dev/lp0).\n");
		DTStamp(); printf("SUCCESS: Display file descriptor parport_fd \t= %d \n", parport_fd);
	}

	// Display details of PARALLEL_PORT 
	DTStamp(); printf("SUCCESS: Display PARPORT_IRQ = %d\n", PARPORT_IRQ);
    DTStamp(); printf("SUCCESS: Display BASE_ADDRESS \t= 0x%02X\n", BASE_ADDRESS);
    DTStamp(); printf("SUCCESS: Display CPUMAP \t= 0x%02X\n", CPUMAP);
	DTStamp(); printf("SUCCESS: Display PERIOD \t= %d (ns)\n", PERIOD);
    DTStamp(); printf("SUCCESS: Display TICK_TIME \t= %d (ns)\n", TICK_TIME);

    DTStamp(); printf("COMPLETED open_parallel_port(void).\n");
}

// ================================================
void close_parallel_port(void) {
// ================================================
    printf("\n");
    DTStamp(); printf("EXECUTING close_parallel_port(void).\n");

	int close_parport = close(parport_fd);
	if (close_parport != 0) {
		DTStamp(); printf("ERROR: Cannot close PARALLEL_PORT (/dev/lp0).\n");
		DTStamp(); printf("ERROR: Display close_parport value \t= %d\n", close_parport);
		perror("close");
	} else {
		DTStamp(); printf("SUCCESS: Close PARALLEL_PORT (/dev/lp0).\n");
		DTStamp(); printf("SUCCESS: Display close_parport value \t= %d\n", close_parport);
	}

DTStamp(); printf("COMPLETED close_parallel_port(void).\n");
}


// ================================================
void reset_CNC(){
// ================================================
    for (count=0; count < 10; count++)
    { 
        outb(0, DATA_REG);     // Send 00000000 to parallel port DATA_REG
        usleep(500);
    }
}

// ================================================
void  cmd_interpreter(int pressed_key) {
// ================================================
/*
// PRESSED KEY CONVERSIONS FOR CNC MOVEMENTS
Right    r = 01110010 = 114
Left     l = 01101100 = 108
Forward  f = 01100110 = 102
Backward b = 01100010 = 98
Up       u = 01110101 = 117
Down     d = 01100100 = 100 
Quit     q = 01110001 = 113
*/

switch (pressed_key) {

	case 114 :    
        // pressed_key char = r or int = 114 
        // DRIVE CNC ALONG X-AXIS
        printf(" r \tdrive_right   (500) X-axis (3,2 CW)\t==> (1/0)(1) (0)(0) (0)(0) running ... ");
        fflush(stdout);
	    drive_right(distance);  // DRIVE (3,2 CW) = 00000011 and 00000010 binary
        printf("done.\n");
        break;

	case 108 :    
        // pressed_key char = l or int = 108 
        // DRIVE CNC ALONG X-AXIS
        printf(" l \tdrive_left    (500) X-axis (1,0 CCW)\t==> (1)(0) (0)(0) (0)(0)   running ... ");
        fflush(stdout);
	    drive_left(distance);   // DRIVE (1,0 CCW) = 00000001 and 00000000 binary
        printf("done.\n");
        break;	

	case 102 :    
        // pressed_key char = f or int = 102 
        // DRIVE CNC ALONG Y-AXIS //front
	    printf(" f \tdrive_forward (500) Y-axis (12,8 CW)\t==> (0)(0) (1/0)(1) (0)(0) running ... ");
	    fflush(stdout);
	    drive_up(distance);    // DRIVE (12,8 CW) = 00001100 and 00001000 binary
        printf("done.\n");
        break;	

	case 98 :    
        // pressed_key char = b or int = 98 
        // DRIVE CNC ALONG Y-AXIS //back
        printf(" b \tdrive_backward(500) Y-axis (4,0 CCW)\t==> (0)(0) (1)(0) (0)(0)   running ... ");
	    fflush(stdout);
        drive_down(distance);  // DRIVE (4,0 CCW) = 00000100 and 00000000 binary
        printf("done.\n");
        break;	

	case 117 :    
        // pressed_key char = u or int = 117 
        // DRIVE CNC ALONG Z-AXIS
        printf(" u \tdrive_up      (500) Z-axis (16,0 CW)\t==> (0)(0) (0)(0) (1)(0)   running ... ");
        fflush(stdout);
	    drive_up(distance);    // DRIVE (16,0 CW) = 00010000 and 00000000 binary
        printf("done.\n");
        break;

	 case 100 :    
        // pressed_key char = d or int = 100 
        // DRIVE CNC ALONG Z-AXIS
        printf(" d \tdrive_down    (500) Z-axis (48,32 CCW)=> (0)(0) (0)(0) (1/0)(1) running ... ");
        fflush(stdout);
	    drive_down(distance);   // DRIVE (48,32 CCW) = 00110000 and 00100000 binary
        printf("done.\n");
        break;
	
        case 113 :
        // pressed_key char = q or int = 113 
        // QUIT AND EXIT PROGRAM
        printf(" q \tQuit and exit. \t\t==> Alhamdulillah. Done. \n\n");
        
        reset_CNC();
        
        // CLOSE PARALLEL PORT AND KEYBOARD
        close_parallel_port();
        close_keyboard();
        DTStamp();printf("Alhamdulillah. Finished CNC keyboard jogging. \n\n");
        exit(0);

     default :
        printf(" %c \tERROR: Invalid command: ==> char %c or int %d \n", pressed_key, pressed_key, pressed_key);
    
} // END switch..case
		
}
// ================================================
void    run_menu(void) {
// ================================================

	printf("\n\tMENU OF COMMANDS (ACTIONS). \n");
	printf("\t===========================\n");

	printf(" r Drive RIGHT-X     the x-axis (3,2    CW) PINS = (1/0)(1) (0)(0) (0)(0)\n");
	printf(" l Drive LEFT-X      the x-axis (1,0   CCW) PINS =   (1)(0) (0)(0) (0)(0)\n");
	
	printf(" f Drive FORWARD-Y   the y-axis (12,8   CW) PINS = (0)(0) (1/0)(1) (0)(0)\n"); 
    printf(" b Drive BACKWARD-Y  the y-axis (4,0   CCW) PINS = (0)(0)   (1)(0) (0)(0)\n");

    printf(" u Drive UP-Z        the z-axis (16,0   CW) PINS = (0)(0) (0)(0)   (1)(0)\n");
	printf(" d Drive DOWN-Z      the z-axis (48,32 CCW) PINS = (0)(0) (0)(0) (1/0)(0)\n");

	printf(" q QUIT and exit this program.\n\n");

	printf("Enter your command: (please wait after pressing a key until '... done'). \n\n");
}
// ==============================================
void init_keyboard() {
// ==============================================
    tcgetattr(0,&initial_settings);
    new_settings = initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_lflag &= ~ISIG;
    new_settings.c_cc[VMIN] = 1;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_settings);
}
// ===============================================
void close_keyboard() {
// ===============================================
    printf("\n");
    DTStamp(); printf("EXECUTING close_keyboard(void).\n");
    int close_keyboard_value = tcsetattr(0, TCSANOW, &initial_settings);
    DTStamp(); printf("SUCCESS: Display close_keyboard_value \t= %d\n", close_keyboard_value);
    DTStamp(); printf("COMPLETED close_keyboard(void).\n");   
}
// ==============================================
int keyboard_hit() {
// ==============================================
    char ch;
    int nread;

    if(peek_character != -1)
      { return 1; }

    new_settings.c_cc[VMIN]=0;
    tcsetattr(0, TCSANOW, &new_settings);
    nread = read(0,&ch,1);
    new_settings.c_cc[VMIN]=1;
    tcsetattr(0, TCSANOW, &new_settings);

    if(nread == 1) {
        peek_character = ch;
        return 1;
    }
return(0);
}
// ==============================================
int read_charkey() {
// ==============================================
    char ch;
    if(peek_character != -1) {
        ch = peek_character;
        peek_character = -1;
        return ch;
    }
    read(0,&ch,1);
return (ch);
}


// ==============================================
// DRIVE CNC ALONG X-AXIS
// ==============================================
void    drive_right(int valdrive) {    
        // DRIVE (3,2 CW) BINARY OUTPUT
        for (count=0; count<valdrive; count++) {
            outb(3, DATA_REG); usleep(500);     // 00000011
            outb(2, DATA_REG); usleep(500);     // 00000010
        }
        reset_CNC();
}
void    drive_left(int valdrive) {    
        // DRIVE (1,0 CCW) BINARY OUTPUT
        for (count=0; count<valdrive; count++) {
            outb(1, DATA_REG); usleep(500);     // 00000001
            outb(0, DATA_REG); usleep(500);     // 00000000
        }
        reset_CNC();
}
// ==============================================
// DRIVE CNC ALONG Y-AXIS 
// ==============================================
void    drive_forward(int valdrive)     {    
        // DRIVE (12,8 CW) BINARY OUTPUT 
        for (count=0; count<valdrive; count++) {
            outb(12, DATA_REG); usleep(500);    // 00001100 
            outb(8,  DATA_REG); usleep(500);    // 00001000
        }
        reset_CNC();
}
void    drive_backward(int valdrive) {    
        // DRIVE (4,0, CCW) BINARY OUTPUT
        for (count=0; count<valdrive; count++) {
            outb(4, DATA_REG); usleep(500);     // 00000100
            outb(0, DATA_REG); usleep(500);     // 00000000
        }
        reset_CNC();
}
// ==============================================
// DRIVE CNC ALONG Z-AXIS
// ==============================================
void    drive_down(int valdrive){        
        // DRIVE (48,32 CCW) BINARY OUTPUT 
        for (count=0; count<valdrive; count++) {
            outb(48, DATA_REG); usleep(500);   // 00110000
            outb(32, DATA_REG); usleep(500);   // 00100000
        }
        reset_CNC();
}
void    drive_up(int valdrive){        
        // DRIVE (16,0  CW) BINARY OUTPUT    
        for (count=0; count<valdrive; count++) {
            outb(16, DATA_REG); usleep(500);    // 00010000
            outb(0,  DATA_REG); usleep(500);    // 00000000
        }
        reset_CNC();
}

// ==================================================================    
int main(int argc, char *argv[]) {
// ==================================================================
    DTStamp(); printf("Bismillah. Start CNC keyboard jogging. \n"); 
    DTStamp(); printf("Note: You must run with root permission. \n\n");  
    
    DATA_REG     = PARPORT_ADDRESS + 0;
    STATUS_REG   = PARPORT_ADDRESS + 1;
    CONTROL_REG  = PARPORT_ADDRESS + 2;
    BASE_ADDRESS = PARPORT_ADDRESS;
    
    // STEP (1) iopl - set I/O priority privilege level
	io_prio_lvl = iopl(3);  		
	check_io_priority_level();

	// STEP (2) ioperm - set port input/output permissions
	io_perm = ioperm(BASE_ADDRESS, 5, 1);
	check_io_permission();
	
    // STEP (3) open parallel port devices (read/write) 
	parport_fd = open(PARPORT_DEVICE, O_WRONLY); 
	open_parallel_port();
  
  
    // STEP (4) BEGIN CNC JOGGING
    int charkey = 0;
    run_menu();
    init_keyboard();
    
    reset_CNC();
    
    // Forever running this for..loop until key q is pressed
    for (; ;) {
        if(keyboard_hit()) {
            charkey = read_charkey();
	        // printf("You hit keyboard key: char = %c or int = %d \n", charkey, charkey);
	        cmd_interpreter(charkey);
        } //END IF 
    } // END FOR
    
    reset_CNC(); 
    
    // STEP (5) Close parallel port 
	// close_parallel_port();  PLACED INSIDE cmd_interpreter(charkey);
    
    // STEP (6)cmd_interpreter(charkey);
    // close_keyboard(); PLACED INSIDE cmd_interpreter(charkey);
    
   
return(0);
}

// ==================================================================
// ALHAMDULILLAH 3 TIMES.
// ==================================================================
/*
COMPILATION AND EXECUTION 

wruslan@HPLaptop-01-ub20:~/github/wruslancnc/Linux-CNCDriver-RTOS-Parport/CNC-Manual-Keyboard-Jogging-C-code$ gcc -o jog-keyboard-manually.cx jog-keyboard-manually.c 
wruslan@HPLaptop-01-ub20:~/github/wruslancnc/Linux-CNCDriver-RTOS-Parport/CNC-Manual-Keyboard-Jogging-C-code$ ls -al
total 116
drwxrwxr-x 2 wruslan wruslan  4096 Jun  2 22:14 .
drwxrwxr-x 5 wruslan wruslan  4096 Jun  2 20:57 ..
-rw-rw-r-- 1 wruslan wruslan 29020 Jun  2 21:57 Gmail-WRY-UMP-PhD-Success-keyboard-jogging-using-parallel-port.pdf
-rw-rw-r-- 1 wruslan wruslan 22035 Jun  2 22:13 jog-keyboard-manually.c
-rwxrwxr-x 1 wruslan wruslan 27056 Jun  2 22:14 jog-keyboard-manually.cx
-rw-rw-r-- 1 wruslan wruslan 22102 Jun  2 21:54 keyboard-jogging-code.c
wruslan@HPLaptop-01-ub20:~/github/wruslancnc/Linux-CNCDriver-RTOS-Parport/CNC-Manual-Keyboard-Jogging-C-code$ sudo ./jog-keyboard-manually.cx 
[sudo] password for wruslan: 
2023-06-02 22:14:54.000668176 	Bismillah. Start CNC keyboard jogging. 
2023-06-02 22:14:54.000668198 	Note: You must run with root permission. 

2023-06-02 22:14:54.000668207 	SUCCESS: Set max priority level io_prio_lvl value 	= 0
2023-06-02 22:14:54.000668218 	SUCCESS: Set port i/o permissions io_perm value 	= 0

2023-06-02 22:14:54.000713410 	EXECUTING  open_parallel_port(void).
2023-06-02 22:14:54.000713428 	SUCCESS: Open PARPORT_DEVICE (/dev/lp0).
2023-06-02 22:14:54.000713451 	SUCCESS: Display file descriptor parport_fd 	= 3 
2023-06-02 22:14:54.000713471 	SUCCESS: Display PARPORT_IRQ = 5
2023-06-02 22:14:54.000713482 	SUCCESS: Display BASE_ADDRESS 	= 0x378
2023-06-02 22:14:54.000713493 	SUCCESS: Display CPUMAP 	= 0x08
2023-06-02 22:14:54.000713519 	SUCCESS: Display PERIOD 	= 500000 (ns)
2023-06-02 22:14:54.000713542 	SUCCESS: Display TICK_TIME 	= 1000000 (ns)
2023-06-02 22:14:54.000713555 	COMPLETED open_parallel_port(void).

	MENU OF COMMANDS (ACTIONS). 
	===========================
 r Drive RIGHT-X     the x-axis (3,2    CW) PINS = (1/0)(1) (0)(0) (0)(0)
 l Drive LEFT-X      the x-axis (1,0   CCW) PINS =   (1)(0) (0)(0) (0)(0)
 f Drive FORWARD-Y   the y-axis (12,8   CW) PINS = (0)(0) (1/0)(1) (0)(0)
 b Drive BACKWARD-Y  the y-axis (4,0   CCW) PINS = (0)(0)   (1)(0) (0)(0)
 u Drive UP-Z        the z-axis (16,0   CW) PINS = (0)(0) (0)(0)   (1)(0)
 d Drive DOWN-Z      the z-axis (48,32 CCW) PINS = (0)(0) (0)(0) (1/0)(0)
 q QUIT and exit this program.

Enter your command: (please wait after pressing a key until '... done'). 

 r 	drive_right   (500) X-axis (3,2 CW)	==> (1/0)(1) (0)(0) (0)(0) running ... done.
 l 	drive_left    (500) X-axis (1,0 CCW)	==> (1)(0) (0)(0) (0)(0)   running ... done.
 f 	drive_forward (500) Y-axis (12,8 CW)	==> (0)(0) (1/0)(1) (0)(0) running ... done.
 b 	drive_backward(500) Y-axis (4,0 CCW)	==> (0)(0) (1)(0) (0)(0)   running ... done.
 u 	drive_up      (500) Z-axis (16,0 CW)	==> (0)(0) (0)(0) (1)(0)   running ... done.
 d 	drive_down    (500) Z-axis (48,32 CCW)=> (0)(0) (0)(0) (1/0)(1) running ... done.
 w 	ERROR: Invalid command: ==> char w or int 119 
 o 	ERROR: Invalid command: ==> char o or int 111 
 n 	ERROR: Invalid command: ==> char n or int 110 
 g 	ERROR: Invalid command: ==> char g or int 103 
 q 	Quit and exit. 		==> Alhamdulillah. Done. 

2023-06-02 22:15:36.000831362 	EXECUTING close_parallel_port(void).
2023-06-02 22:15:36.000831397 	SUCCESS: Close PARALLEL_PORT (/dev/lp0).
2023-06-02 22:15:36.000831412 	SUCCESS: Display close_parport value 	= 0
2023-06-02 22:15:36.000831423 	COMPLETED close_parallel_port(void).

2023-06-02 22:15:36.000831439 	EXECUTING close_keyboard(void).
2023-06-02 22:15:36.000831457 	SUCCESS: Display close_keyboard_value 	= 0
2023-06-02 22:15:36.000831468 	COMPLETED close_keyboard(void).
2023-06-02 22:15:36.000831477 	Alhamdulillah. Finished CNC keyboard jogging. 

wruslan@HPLaptop-01-ub20:~/github/wruslancnc/Linux-CNCDriver-RTOS-Parport/CNC-Manual-Keyboard-Jogging-C-code$ 

*/
// ==================================================================

