#ifndef CAMERA_H
#define CAMERA_H

#include <stdbool.h>

typedef struct {
	const char *label;
	int x;
	int y;
} BlobDetection;

typedef struct {
	bool detected;   // true if any boundary line is visible
	bool left;       // line detected in left third of frame
	bool center;     // line detected in center third of frame
	bool right;      // line detected in right third of frame
	int  line_y;     // topmost row where a line pixel was found (-1 if none)
} LineDetection;

void setup(void);
void camera_bus_diagnostic(void);
void init_cam(void);
bool findblobs(BlobDetection *result);
bool detect_boundary_line(LineDetection *result);

#endif
