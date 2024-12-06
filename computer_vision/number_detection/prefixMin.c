#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define MIN(a,b) ((a) < (b) ? (a) : (b))

void prefix_min(uint8_t* arr, uint8_t* prefix_min_arr, int rows, int cols, int axis) {
	//init prefix_min_arr as a copy of arr
	
	switch(axis) {
		// left -> right
		case 1:
			for (int j = 0; j < cols; j++) {
				for (int i = 0; i < rows; i++) {
					if (j == 0) {
						prefix_min_arr[i*cols + j] = arr[i*cols+j];
					} else {
						prefix_min_arr[i*cols + j] = MIN(prefix_min_arr[i*cols + (j-1)], arr[i*cols + j]); 
					}
				}
			}
			break;
		// bottom-> top
		case 2: 
			for (int i = rows-1; i > -1; i--) {
				for (int j = 0; j < cols; j++) {
					if (i == rows-1) {
						prefix_min_arr[i*cols + j] = arr[i*cols + j];
					} else {
						prefix_min_arr[i*cols + j] = MIN(prefix_min_arr[(i+1)*cols + j], arr[i*cols + j]);
					}
				}
			}
			break;
		// right -> left
		case 3:
			for (int j = cols-1; j > -1; j--) {
				for (int i = 0; i < rows; i++) {
					if (j == cols-1) {
						prefix_min_arr[i*cols + j] =  arr[i*cols + j];
					} else {
						prefix_min_arr[i*cols + j] = MIN(prefix_min_arr[i*cols + (j+1)], arr[i*cols + j]);
					}
				}
			}
			break;
		//case: 0
		default:
			// top -> bottom
			for (int i = 0; i < rows; i++) {
				for (int j = 0; j < cols; j++) {
					if (i == 0) {
						prefix_min_arr[i*cols + j] =  arr[i*cols + j];
					} else {
						prefix_min_arr[i*cols + j] = MIN(prefix_min_arr[(i-1)*cols + j], arr[i*cols + j]);
					}
				}
			}
			break;
	}
	// done

}