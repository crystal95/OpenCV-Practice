void mcvTransformImage2Ground(const CvMat *inPoints,
                              CvMat *outPoints, const CameraInfo *cameraInfo)
{

  //add two rows to the input points
  CvMat *inPoints4 = cvCreateMat(inPoints->rows+2, inPoints->cols,
      cvGetElemType(inPoints));
  CvMat inPoints2, inPoints3, inPointsr4, inPointsr3;
  cvGetRows(inPoints4, &inPoints2, 0, 2);
  cvGetRows(inPoints4, &inPoints3, 0, 3);
  cvGetRow(inPoints4, &inPointsr3, 2);
 cvGetRow(inPoints4, &inPointsr4, 3);
  cvSet(&inPointsr3, cvRealScalar(1));
  cvCopy(inPoints, &inPoints2);
  float c1 = cos(cameraInfo->pitch);
  float s1 = sin(cameraInfo->pitch);
  float c2 = cos(cameraInfo->yaw);
  float s2 = sin(cameraInfo->yaw);
  float matp[] = {
    -cameraInfo->cameraHeight*c2/cameraInfo->focalLength.x,
    cameraInfo->cameraHeight*s1*s2/cameraInfo->focalLength.y,
    (cameraInfo->cameraHeight*c2*cameraInfo->opticalCenter.x/
      cameraInfo->focalLength.x)-
      (cameraInfo->cameraHeight *s1*s2* cameraInfo->opticalCenter.y/
      cameraInfo->focalLength.y) - cameraInfo->cameraHeight *c1*s2,

    cameraInfo->cameraHeight *s2 /cameraInfo->focalLength.x,
    cameraInfo->cameraHeight *s1*c2 /cameraInfo->focalLength.y,
    (-cameraInfo->cameraHeight *s2* cameraInfo->opticalCenter.x
      /cameraInfo->focalLength.x)-(cameraInfo->cameraHeight *s1*c2*
      cameraInfo->opticalCenter.y /cameraInfo->focalLength.y) -
      cameraInfo->cameraHeight *c1*c2,

    0,
    cameraInfo->cameraHeight *c1 /cameraInfo->focalLength.y,
    (-cameraInfo->cameraHeight *c1* cameraInfo->opticalCenter.y /
      cameraInfo->focalLength.y) + cameraInfo->cameraHeight *s1,

    0,
    -c1 /cameraInfo->focalLength.y,
    (c1* cameraInfo->opticalCenter.y /cameraInfo->focalLength.y) - s1,
  };
  CvMat mat = cvMat(4, 3, CV_32FC1, matp);
  cvMatMul(&mat, &inPoints3, inPoints4);
  int i;
  for (int i=0; i<inPoints->cols; i++)
  {
    float div = CV_MAT_ELEM(inPointsr4, float, 0, i);
    CV_MAT_ELEM(*inPoints4, float, 0, i) =
        CV_MAT_ELEM(*inPoints4, float, 0, i) / div ;
    CV_MAT_ELEM(*inPoints4, float, 1, i) =
        CV_MAT_ELEM(*inPoints4, float, 1, i) / div;
  }
  cvCopy(&inPoints2, outPoints);
  cvReleaseMat(&inPoints4);
}