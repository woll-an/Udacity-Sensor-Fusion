# Radar Target Generation and Detection

## Implementation steps for the 2D CFAR process.

* create zero-filled Matrix CFAR2d with the dimensions of the Range Doppler Map
* define number of training and guard cells
* move the CUT across the Range Doppler Map
* compute the mean of all training cells for current CUT
* compute threshold from mean and SNR offset
* if value of CUT is above threshold, set value in CFAR2d to 1

## Selection of Training, Guard cells and offset.

I started with values for training cells, guard cells and offset which were used in the lessons of this course. I slightly modified them, to see which effect they had on the result and chose values where the result looked promising.

## Steps taken to suppress the non-thresholded cells at the edges.

As I used a zero-filled matrix there was no need to suppress non-thresholded cells.