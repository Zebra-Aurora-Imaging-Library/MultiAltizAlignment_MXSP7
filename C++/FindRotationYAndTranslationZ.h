//***************************************************************************************/
// 
// File name: FindRotationYAndTranslationZ.h
//
// Synopsis: Implementation of the method to find the rotation Y and the translation Z
//           of the visible part of the bar.
// 
// Copyright © Matrox Electronic Systems Ltd., 1992-YYYY.
// All Rights Reserved
//*************************************************************************************/

//******************************************************************************
// Constants
//******************************************************************************
static const MIL_DOUBLE BAR_WIDTH              = 110.0;
static const MIL_DOUBLE APPROX_VIEW_BAR_LENGTH = 120.0;
static const MIL_DOUBLE BAR_DIM_TOLERANCE      = 60.0;

static const MIL_DOUBLE PLANE_RECT_OPACITY = 80.0;

static const MIL_DOUBLE PLANE_DATA_CROP_BOX_DEPTH = 0.5;
static const MIL_DOUBLE PLANE_DATA_CROP_BOX_SCALE = 2.0;

//******************************************************************************
// Utility structures.
//******************************************************************************
struct STransformation
   {
   MIL_DOUBLE TX;
   MIL_DOUBLE TY;
   MIL_DOUBLE TZ;
   MIL_DOUBLE RX;
   MIL_DOUBLE RY;
   MIL_DOUBLE RZ;
   };

struct SFindBarPlaneResult
   {
   bool IsValid = false;
   MIL_UNIQUE_BUF_ID MilTransformedPointCloud;
   STransformation Transformation;
   };

//****************************************************************************
// Use 3D rectangle finder and line fit to get Ry and Tz.
//****************************************************************************
SFindBarPlaneResult FindRotationYAndTranslationZ(MIL_ID MilSystem, MIL_ID MilPointCloud, MIL_ID MilGraphicList, MIL_INT Iteration)
   {
   SFindBarPlaneResult FindResult;

   if (Iteration == 0)
      {
      MosPrintf(MIL_TEXT("The first step is to determine rotation around the Y axis and translation along Z\n")
                MIL_TEXT("using the calibration tool plane and its edge.\n\n"));

      MosPrintf(MIL_TEXT("3D rectangle plane finder is used to find planes of the scene.\n"));
      MosPrintf(MIL_TEXT("The highest plane will be selected as our plane of interest.\n\n"));
      }

   // Allocate the working point clouds.
   FindResult.MilTransformedPointCloud = MbufAllocContainer(M_DEFAULT_HOST, M_PROC + M_DISP, M_DEFAULT, M_UNIQUE_ID);
   auto MilLinePointClouds = MbufAllocContainer(M_DEFAULT_HOST, M_PROC + M_DISP, M_DEFAULT, M_UNIQUE_ID);

   // Use rectangle plane finder to find the tool plane. We make the assumption the the visible parts of the
   // bar appears close to a rectangle in the Altiz scans.
   auto MilModContext = M3dmodAlloc(MilSystem, M_FIND_RECTANGULAR_PLANE_CONTEXT, M_DEFAULT, M_UNIQUE_ID);
   auto MilModResult = M3dmodAllocResult(MilSystem, M_FIND_RECTANGULAR_PLANE_RESULT, M_DEFAULT, M_UNIQUE_ID);
   M3dmodDefine(MilModContext, M_ADD, M_RECTANGLE, APPROX_VIEW_BAR_LENGTH, BAR_WIDTH, BAR_DIM_TOLERANCE, BAR_DIM_TOLERANCE, M_DEFAULT, M_DEFAULT, M_DEFAULT);
   M3dmodControl(MilModContext, 0, M_NUMBER, M_ALL);
   M3dmodControl(MilModContext, M_CONTEXT, M_SORT, M_CENTER_Z);
   M3dmodPreprocess(MilModContext, M_DEFAULT);
   M3dmodFind(MilModContext, MilPointCloud, MilModResult, M_DEFAULT);

   if(M3dmodGetResult(MilModResult, M_DEFAULT, M_NUMBER, M_NULL > 0))
      {
      if (Iteration == 0)
         {
         auto PlaneLabel = M3dmodDraw3d(M_DEFAULT, MilModResult, 0, MilGraphicList, M_DEFAULT, M_DEFAULT);
         M3dgraControl(MilGraphicList, PlaneLabel, M_OPACITY + M_RECURSIVE, PLANE_RECT_OPACITY);

         MosPrintf(MIL_TEXT("The highest rectangle plane is displayed.\n"));
         MosPrintf(MIL_TEXT("Press any key to continue.\n\n"));
         MosGetch();

         M3dgraRemove(MilGraphicList, PlaneLabel, M_DEFAULT);
         }

      // Cropping the point cloud to only keep the plane.
      auto MilBox = M3dgeoAlloc(MilSystem, M_GEOMETRY, M_DEFAULT, M_UNIQUE_ID);
      M3dmodCopyResult(MilModResult, 0, MilBox, M_DEFAULT, M_BOUNDING_BOX, M_DEFAULT);
      M3dgeoBox(MilBox, M_CENTER_AND_DIMENSION + M_ORIENTATION_UNCHANGED, M_UNCHANGED, M_UNCHANGED, M_UNCHANGED, M_UNCHANGED, M_UNCHANGED, PLANE_DATA_CROP_BOX_DEPTH, M_DEFAULT);
      M3dimScale(MilBox, MilBox, PLANE_DATA_CROP_BOX_SCALE, PLANE_DATA_CROP_BOX_SCALE, PLANE_DATA_CROP_BOX_SCALE, M_GEOMETRY_CENTER, M_DEFAULT, M_DEFAULT, M_DEFAULT);
      M3dimCrop(MilPointCloud, FindResult.MilTransformedPointCloud, MilBox, M_NULL, M_SAME, M_DEFAULT);

      // Project the point on the Y = 0 plane.
      M3dimRemovePoints(FindResult.MilTransformedPointCloud, MilLinePointClouds, M_INVALID_POINTS_ONLY, M_DEFAULT);
      MIL_ID MilRange = MbufInquireContainer(MilLinePointClouds, M_COMPONENT_RANGE, M_COMPONENT_ID, M_NULL);
      auto MilRangeY = MbufChildColor(MilRange, 1, M_UNIQUE_ID);
      MbufClear(MilRangeY, 0.0);

      // Fit a line on the projected points. The line will give us Ry and Tz.
      MIL_UNIQUE_3DMET_ID MilFitResult = M3dmetAllocResult(MilSystem, M_FIT_RESULT, M_DEFAULT, M_UNIQUE_ID);
      M3dmetFit(M_DEFAULT, MilLinePointClouds, M_LINE, MilFitResult, 1, M_DEFAULT);

      // Determine Ry.
      MIL_DOUBLE LineAxisX = M3dmetGetResult(MilFitResult, M_AXIS_X, M_NULL);
      MIL_DOUBLE LineAxisZ = M3dmetGetResult(MilFitResult, M_AXIS_Z, M_NULL);
      MIL_DOUBLE RotAngle = atan(LineAxisZ / LineAxisX) * DIV_180_PI;
      M3dimRotate(MilLinePointClouds, MilLinePointClouds, M_ROTATION_Y, RotAngle, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT);
      M3dimRotate(FindResult.MilTransformedPointCloud, FindResult.MilTransformedPointCloud, M_ROTATION_Y, RotAngle, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT);

      // Determine Tz.
      M3dmetFit(M_DEFAULT, MilLinePointClouds, M_LINE, MilFitResult, M_INFINITE, M_DEFAULT);
      auto LineCenterZ = M3dmetGetResult(MilFitResult, M_CENTER_Z, M_NULL);
      M3dimTranslate(FindResult.MilTransformedPointCloud, FindResult.MilTransformedPointCloud, 0, 0, -LineCenterZ, M_DEFAULT);

      if (Iteration == 0)
         {
         MosPrintf(MIL_TEXT("Now that we have rotated the point cloud around the Y axis we determine Tz\n")
                   MIL_TEXT("based on the Z-coordinate of the center point of the fitted plane.\n"));
         MosPrintf(MIL_TEXT("Press any key to continue.\n\n"));
         MosGetch();
         }

      FindResult.Transformation = {0.0, 0.0, -LineCenterZ, 0.0, RotAngle, 0.0};
      FindResult.IsValid = true;
      }
   else
      MosPrintf(MIL_TEXT("The bar plane was not found.\n"));

   return FindResult;
   }
