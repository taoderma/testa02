/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file process_frame.c
 * @brief Contains the actual algorithm and calculations.
 */

/* Definitions specific to this application. Also includes the Oscar main header file. */
#include "template.h"
#include <string.h>
#include <stdlib.h>

OSC_ERR OscVisDrawBoundingBoxBW(struct OSC_PICTURE *picIn, struct OSC_VIS_REGIONS *regions, uint8 Color);

int ownOtsu(void);

void ProcessFrame(uint8 *pInputImg)
{
	int c, r;
	int nc = OSC_CAM_MAX_IMAGE_WIDTH/2;
	int siz = sizeof(data.u8TempImage[GRAYSCALE]);

	struct OSC_PICTURE Pic1, Pic2;//we require these structures to use Oscar functions
	struct OSC_VIS_REGIONS ImgRegions;//these contain the foreground objects

	int Threshold;

	if(data.ipc.state.nThreshold == 0)	/* bei nThreshold = 0 wird der Otsu-Threshold verwendet*/
	{
		Threshold = ownOtsu();
	}	else	{
		Threshold = data.ipc.state.nThreshold;
	}

	/* this is the default case */
	for(r = 0; r < siz; r+= nc)/* we strongly rely on the fact that them images have the same size */
	{
		for(c = 0; c < nc; c++)
		{
			/* first determine the foreground estimate */
			data.u8TempImage[THRESHOLD][r+c] = data.u8TempImage[GRAYSCALE][r+c] < (uint8) Threshold ? 0xff : 0;
		}
	}

		
	for(r = nc; r < siz-nc; r+= nc)/* we skip the first and last line */
	{
		for(c = 1; c < nc-1; c++)/* we skip the first and last column */
		{
			unsigned char* p = &data.u8TempImage[THRESHOLD][r+c];
			data.u8TempImage[DILATION][r+c] = *(p-nc-1) | *(p-nc) | *(p-nc+1) |
											  *(p-1)    | *p      | *(p+1)    |
											  *(p+nc-1) | *(p+nc) | *(p+nc+1);
		}
	}

	for(r = nc; r < siz-nc; r+= nc)/* we skip the first and last line */
	{
		for(c = 1; c < nc-1; c++)/* we skip the first and last column */
		{
			unsigned char* p = &data.u8TempImage[DILATION][r+c];
			data.u8TempImage[EROSION][r+c] = *(p-nc-1) & *(p-nc) & *(p-nc+1) &
											 *(p-1)    & *p      & *(p+1)    &
											 *(p+nc-1) & *(p+nc) & *(p+nc+1);
		}
	}

	//wrap image DILATION in picture struct
	Pic1.data = data.u8TempImage[DILATION];
	Pic1.width = nc;
	Pic1.height = OSC_CAM_MAX_IMAGE_HEIGHT/2;
	Pic1.type = OSC_PICTURE_GREYSCALE;
	//as well as EROSION (will be used as output)
	Pic2.data = data.u8TempImage[EROSION];
	Pic2.width = nc;
	Pic2.height = OSC_CAM_MAX_IMAGE_HEIGHT/2;
	Pic2.type = OSC_PICTURE_BINARY;//probably has no consequences
	//have to convert to OSC_PICTURE_BINARY which has values 0x01 (and not 0xff)
	OscVisGrey2BW(&Pic1, &Pic2, 0x80, false);

	//now do region labeling and feature extraction
	OscVisLabelBinary( &Pic2, &ImgRegions);
	OscVisGetRegionProperties( &ImgRegions);

	//OscLog(INFO, "number of objects %d\n", ImgRegions.noOfObjects);
	//plot bounding boxes both in gray and dilation image
	Pic2.data = data.u8TempImage[GRAYSCALE];
	OscVisDrawBoundingBoxBW( &Pic2, &ImgRegions, 255);
	OscVisDrawBoundingBoxBW( &Pic1, &ImgRegions, 128);
}


/* Drawing Function for Bounding Boxes; own implementation because Oscar only allows colored boxes; here in Gray value "Color"  */
/* should only be used for debugging purposes because we should not drawn into a gray scale image */
OSC_ERR OscVisDrawBoundingBoxBW(struct OSC_PICTURE *picIn, struct OSC_VIS_REGIONS *regions, uint8 Color)
{
	 uint16 i, o;
	 uint8 *pImg = (uint8*)picIn->data;
	 const uint16 width = picIn->width;
	 for(o = 0; o < regions->noOfObjects; o++)//loop over regions
	 {
		 /* Draw the horizontal lines. */
		 for (i = regions->objects[o].bboxLeft; i < regions->objects[o].bboxRight; i += 1)
		 {
				 pImg[width * regions->objects[o].bboxTop + i] = Color;
				 pImg[width * (regions->objects[o].bboxBottom - 1) + i] = Color;
		 }

		 /* Draw the vertical lines. */
		 for (i = regions->objects[o].bboxTop; i < regions->objects[o].bboxBottom-1; i += 1)
		 {
				 pImg[width * i + regions->objects[o].bboxLeft] = Color;
				 pImg[width * i + regions->objects[o].bboxRight] = Color;
		 }
	 }
	 return SUCCESS;
}

int ownOtsu(void)
{
	int c = 0, r = 0, max = 0, i = 0;
	int siz = sizeof(data.u8TempImage[GRAYSCALE]);
	uint32 hist[255];
	memset(hist, 0, sizeof(hist));
	uint32 w0[255];
	memset(w0, 0, sizeof(w0));
	uint32 w1[255];
	memset(w1, 0, sizeof(w1));
	uint32 m0[255];
	memset(m0, 0, sizeof(m0));
	uint32 m1[255];
	memset(m1, 0, sizeof(m1));

	float sB2[255];
	memset(sB2, 0, sizeof(sB2));

	float mean[255];
	memset(mean, 0, sizeof(mean));

	float m0w0[255];
	memset(m0w0, 0, sizeof(m0w0));

	float m1w1[255];
	memset(m1w1, 0, sizeof(m1w1));

	uint8 *p = data.u8TempImage[GRAYSCALE];

	// Histogramm berechnen
	for (r = 0; r < siz; r++) {
		hist[p[r]] += 1;
	}

	// Mean Berechnen
	for (r = 0; r < 256; r++) {
		mean[r] = hist[r] * r;
	}

	// Threshold mit otsu berechnen
	i = 1;
	for (r = 0; r < 256; r++) {

		// Berechnung von m0, m1, w0, w1
		for (c = r+1; c < 256; c++) {
			w1[r] += hist[c];		// Berechnung w1
			m1[r] += mean[c];		// Berechnung m1
		}

		for (c = 0; c <= r; c++) {
			m0[r] += mean[c];		// Berechnung m0
			w0[r] += hist[c];		// Berechnung w0
		}

		// Berechnung der Divisionen m0/w0, m1/w1
		if (w0[r] == 0 || w1[r] == 0) {		// Falls ein Divisor = 0, wird Divisor auf 1 gesetzt
			m0w0[r] = (float) m0[r];
			m1w1[r] = (float) m1[r];
		} else {							// Sonst Division mit m0/w0, m1/w1
			m0w0[r] = (float) m0[r] / w0[r];
			m1w1[r] = (float) m1[r] / w1[r];
		}

		// Differenz der beiden Quotienten m0/w0, m1/w1 berechnen
		sB2[r] = (m0w0[r] - m1w1[r]);

		// Sigma B berechnen (Quotientendifferenz quadrieren und um 24Bit nach rechts schieben, multiplizieren mit w0*w1)
		sB2[r] = sB2[r] * (m0w0[r] - m1w1[r]) / 16777216 * w0[r] * w1[r];

		// Array-Index suchen, and dem sich das Maximum von Sigma B sich befindet
		if (r > 0) {
			if (sB2[r - 1] < sB2[r]) {
				max = r;
			}
		}
	}
   OscLog(INFO,"thr=%d\n", max);
	return max;
}


