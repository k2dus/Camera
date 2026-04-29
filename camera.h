#ifndef CAMERA_H
#define CAMERA_H

#include <stdbool.h>

typedef struct {
	const char *label;
	int x;
	int y;
} BlobDetection;

void setup(void);
void camera_bus_diagnostic(void);
void init_cam(void);
bool findblobs(BlobDetection *result);

#endif
