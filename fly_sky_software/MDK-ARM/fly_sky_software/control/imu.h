
#include "data.h"
extern float rMat[3][3];



void imuUpdate(acc_data acc, gyro_data gyro, self_data *state , float dt);

/*���嵽����*/
void imuTransformVectorBodyToEarth(acc_data * v);

/*���򵽻���*/
void imuTransformVectorEarthToBody(acc_data * v);
