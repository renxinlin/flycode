
#include "data.h"
extern float rMat[3][3];



void imuUpdate(acc_data acc, gyro_data gyro, self_data *state , float dt);

/*机体到地球*/
void imuTransformVectorBodyToEarth(acc_data * v);

/*地球到机体*/
void imuTransformVectorEarthToBody(acc_data * v);
