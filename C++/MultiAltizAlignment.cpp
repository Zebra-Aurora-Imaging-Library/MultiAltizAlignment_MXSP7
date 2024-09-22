//***************************************************************************************/
// 
// File name: MultiAltizAlignment.cpp  
//
// Synopsis: This example shows how to automatically align point clouds acquired by multiple
//           Altiz using a simple bar with holes. The distance between holes must be known
//           precisely.
//           This example does not correct the rotations around the X-axis and the Z-axis.
//           It is therefore important to position the camera in such a way that there are
//           no rotations around these axes.
//           The distances between holes are measured along the Altiz X axis.
// 
// Copyright © Matrox Electronic Systems Ltd., 1992-YYYY.
// All Rights Reserved
//*************************************************************************************/
#include <mil.h>
#include <array>
#include "AutomaticAlignment.h"
#include "FindRotationYAndTranslationZ.h"

//***************************************************************************
// Example description.
//***************************************************************************
void PrintHeader()
   {
   MosPrintf(MIL_TEXT("[EXAMPLE NAME]\n"));
   MosPrintf(MIL_TEXT("MultiAltizAlignment\n\n"));
   MosPrintf(MIL_TEXT("[SYNOPSIS]\n"));
   MosPrintf(MIL_TEXT("This example shows how to automatically align point clouds acquired by\n"));
   MosPrintf(MIL_TEXT("multiple Altiz using a simple bar with holes.\n"));
   MosPrintf(MIL_TEXT("The distance between holes must be known precisely.\n"));
   MosPrintf(MIL_TEXT("This example does not correct the rotations around the X-axis and the Z-axis.\n"));
   MosPrintf(MIL_TEXT("It is therefore important to position the camera in such a way that there are\n"));
   MosPrintf(MIL_TEXT("no rotations around these axes.\n\n"));
   MosPrintf(MIL_TEXT("The distances between holes are measured along the Altiz X axis.\n\n"));

   MosPrintf(MIL_TEXT("[MODULES USED]\n"));
   MosPrintf(MIL_TEXT("Modules used: 3D Image Processing, 3D Display, 3D Geometry,\n"));
   MosPrintf(MIL_TEXT("3D Graphics,  3D Model Finder and Buffer.\n\n"));

   // Wait for a key to be pressed.
   MosPrintf(MIL_TEXT("Press any key to continue.\n\n"));
   MosGetch();
   }

//******************************************************************************
// Utility structures.
//******************************************************************************
struct SDisplayInfo
   {
   MIL_UINT   PositionX; // X position of the display.
   MIL_UINT   PositionY; // Y position of the display.
   MIL_UINT   Size;      // Size of the display (in both directions).
   MIL_STRING Title;     // Title of the display.
   };

//*****************************************************************************
// Constants.
//*****************************************************************************
static const MIL_INT NUM_SCANS = 3;
static MIL_CONST_TEXT_PTR FILE_POINT_CLOUD[NUM_SCANS] =
   {
   MIL_TEXT("../MR1_Alu.mbufc"),
   MIL_TEXT("../MR2_Alu.mbufc"),
   MIL_TEXT("../MR3_Alu.mbufc"),
   };
static MIL_CONST_TEXT_PTR FILE_POINT_CLOUD_KEYBOARD[NUM_SCANS] =
   {
   MIL_TEXT("../MR1_Keyboard.mbufc"),
   MIL_TEXT("../MR2_Keyboard.mbufc"),
   MIL_TEXT("../MR3_Keyboard.mbufc"),
   };

static const MIL_STRING FILE_MATRIX_PRIMARY = MIL_TEXT("TransformMatrixMaster.m3dgeo");
static const MIL_STRING FILE_MATRIX_PREFIX = MIL_TEXT("TransformationMatrixMR");

// 3D display.
static const MIL_UINT DISP3D_BORDER_SIZE_Y = 30;
static const MIL_UINT DISP3D_SIZE = 500;
static SDisplayInfo DST_DISPLAY_INFO[] =
   {
      {              0, 0, DISP3D_SIZE, MIL_TEXT("First part")},
      {    DISP3D_SIZE, 0, DISP3D_SIZE, MIL_TEXT("Second part")},
      {2 * DISP3D_SIZE, 0, DISP3D_SIZE, MIL_TEXT("Third part")},
   };

// Depth map display.
static const MIL_INT    DISP_DEPTH_MAP_POS_Y = DISP3D_BORDER_SIZE_Y + DISP3D_SIZE;
static const MIL_DOUBLE DISP_DEPTH_MAP_ZOOM = 0.6;

// Bar with holes information.
static const MIL_INT BAR_HOLES_DISTANCE_X = 100;

// Point cloud merge.
static const MIL_INT MERGE_DECIMATION_STEP = 4;

//****************************************************************************
// Structure of the example data.
//****************************************************************************
struct SAlignmentData
   {
   std::array<MIL_UNIQUE_3DDISP_ID, NUM_SCANS> MilDisplay3d;
   std::array<MIL_ID              , NUM_SCANS> MilGraphicList3d;
   std::array<MIL_UNIQUE_BUF_ID   , NUM_SCANS> MilToAlignPointClouds;
   bool IsValid = true;
   };

//****************************************************************************
// Structure representing a BGR32 color.
//****************************************************************************
struct SBGR32Color
   {
   MIL_UINT8 B;
   MIL_UINT8 G;
   MIL_UINT8 R;
   MIL_UINT8 A;
   };

//****************************************************************************
// Function declaration.
//****************************************************************************
MIL_INT FindTransformationMatrices(MIL_ID MilSystem);
MIL_INT MergeFromRestoredMatrices(MIL_ID MilSystem);
SAlignmentData RestoreAndShowAlignmentData(MIL_ID MilSystem, MIL_CONST_TEXT_PTR PointCloudFiles[], bool AddNormalIfMissing = false);
void MergeAndShowAligned(MIL_ID MilSystem, const std::vector<MIL_ID>& MilToAlignPointClouds);
MIL_UNIQUE_3DDISP_ID Alloc3dDisplayId(MIL_ID MilSystem);
MIL_UNIQUE_3DDISP_ID Alloc3dDisplayId(MIL_ID MilSystem, MIL_INT PositionX, MIL_INT PositionY,
                                      MIL_INT SizeX, MIL_INT SizeY, const MIL_STRING& Title);
void ColorCloud(MIL_ID MilPointCloud, MIL_INT Col);
bool CheckForRequiredMILFile(const MIL_STRING& FileName);
std::vector<SBGR32Color> GetDistinctColors(MIL_INT NbColors);
MIL_STRING BuildCameraTransformationMatrixName(MIL_INT CameraIndex);

//****************************************************************************
// Main.
//****************************************************************************
int MosMain(void)
   {
   // Print example description.
   PrintHeader();

   // Allocate objects
   auto MilApplication = MappAlloc(M_NULL, M_DEFAULT, M_UNIQUE_ID);
   auto MilSystem = MsysAlloc(MilApplication, M_SYSTEM_HOST, M_DEFAULT, M_DEFAULT, M_UNIQUE_ID);

   // Find transformation matrices using a tool.
   FindTransformationMatrices(MilSystem);

   // Restore transformation matrices to align PC.
   MergeFromRestoredMatrices(MilSystem);

   return 0;
   }

//****************************************************************************
// Restores and shows the alignment data.
//****************************************************************************
SAlignmentData RestoreAndShowAlignmentData(MIL_ID MilSystem, MIL_CONST_TEXT_PTR PointCloudFiles[], bool AddNormalIfMissing)
   {
   SAlignmentData AlignmentData;
   for(MIL_INT f = 0; f < NUM_SCANS; f++)
      {
      if(!CheckForRequiredMILFile(PointCloudFiles[f]))
         {
         AlignmentData.IsValid = false;
         return AlignmentData;
         }

      // Restore the point cloud.
      AlignmentData.MilToAlignPointClouds[f] = MbufImport(PointCloudFiles[f], M_DEFAULT, M_RESTORE, MilSystem, M_UNIQUE_ID);
      MbufConvert3d(AlignmentData.MilToAlignPointClouds[f], AlignmentData.MilToAlignPointClouds[f], M_NULL, M_DEFAULT, M_DEFAULT);

      // Allocate the display.
      const auto DispInfo = DST_DISPLAY_INFO[f];
      AlignmentData.MilDisplay3d[f] = Alloc3dDisplayId(MilSystem, DispInfo.PositionX, DispInfo.PositionY,
                                                       DispInfo.Size, DispInfo.Size, DispInfo.Title);
      AlignmentData.MilGraphicList3d[f] = M3ddispInquire(AlignmentData.MilDisplay3d[f], M_3D_GRAPHIC_LIST_ID, M_NULL);

      // Add the normals if required.
      if(AddNormalIfMissing && MbufInquireContainer(AlignmentData.MilToAlignPointClouds[f], M_COMPONENT_NORMALS_MIL, M_COMPONENT_ID, M_NULL) == M_NULL)
         M3dimNormals(M_NORMALS_CONTEXT_ORGANIZED, AlignmentData.MilToAlignPointClouds[f], AlignmentData.MilToAlignPointClouds[f], M_DEFAULT);

      // Show the point cloud.
      M3ddispControl(AlignmentData.MilDisplay3d[f], M_UPDATE, M_DISABLE);
      MIL_INT64 PointCloudLabel = M3dgraAdd(AlignmentData.MilGraphicList3d[f], M_DEFAULT, AlignmentData.MilToAlignPointClouds[f], M_NO_LINK);
      M3ddispSetView(AlignmentData.MilDisplay3d[f], M_AUTO, M_BOTTOM_TILTED, M_DEFAULT, M_DEFAULT, M_DEFAULT);
      M3ddispSelect(AlignmentData.MilDisplay3d[f], M_NULL, M_OPEN, M_DEFAULT);
      M3dgraControl(AlignmentData.MilGraphicList3d[f], PointCloudLabel, M_COLOR_USE_LUT, M_TRUE);
      M3dgraControl(AlignmentData.MilGraphicList3d[f], PointCloudLabel, M_COLOR_COMPONENT, M_COMPONENT_RANGE);
      M3dgraControl(AlignmentData.MilGraphicList3d[f], PointCloudLabel, M_COLOR_COMPONENT_BAND, 2);
      
      M3ddispControl(AlignmentData.MilDisplay3d[f], M_UPDATE, M_ENABLE);
      }

   MosPrintf(MIL_TEXT("All 3D point cloud are restored from files and displayed.\n"));
   MosPrintf(MIL_TEXT("Press any key to continue.\n\n"));
   MosGetch();

   return AlignmentData;
   }

//*****************************************************************************
// Find transformation matrices using simple bar with holes.
//*****************************************************************************
MIL_INT FindTransformationMatrices(MIL_ID MilSystem)
   {
   // Allocate the display for 2D processing.
   auto MilDisplay = MdispAlloc(MilSystem, M_DEFAULT, MIL_TEXT("M_DEFAULT"), M_WINDOWED, M_UNIQUE_ID);

   // Restore and show the point cloud data.
   const bool NeedNormal = true;
   auto AlignmentData = RestoreAndShowAlignmentData(MilSystem, FILE_POINT_CLOUD, NeedNormal);
   if(!AlignmentData.IsValid)
      return -1;

   // Init the reference hole position.
   MIL_DOUBLE RefCircleXPos = 0.0;
   MIL_DOUBLE RefCircleYPos = 0.0;

   auto Colors = GetDistinctColors(NUM_SCANS);

   for (MIL_INT i = 0; i < NUM_SCANS; i++)
      {
      MIL_ID MilToAlignPointCloud = AlignmentData.MilToAlignPointClouds[i];

      auto FindBarPlaneResult = FindRotationYAndTranslationZ(MilSystem, MilToAlignPointCloud, AlignmentData.MilGraphicList3d[i], i);
      if (!FindBarPlaneResult.IsValid)
         return -1;

      // Create depth map for primary scan.
      MIL_UNIQUE_BUF_ID MilDepthMap = CreateDepthMap(MilSystem, FindBarPlaneResult.MilTransformedPointCloud);
      if (i == 0)
         {
         // Display the depthmap for first iteration.
         MdispControl(MilDisplay, M_TITLE, MIL_TEXT("Depthmap"));
         MdispControl(MilDisplay, M_WINDOW_INITIAL_POSITION_Y, DISP_DEPTH_MAP_POS_Y);
         MdispZoom(MilDisplay, DISP_DEPTH_MAP_ZOOM, DISP_DEPTH_MAP_ZOOM);
         MdispSelect(MilDisplay, MilDepthMap);
         MosPrintf(MIL_TEXT("After correcting Ry and Tz of the 3D point cloud a depth map is created.\n"));
         MosPrintf(MIL_TEXT("We use circle finder to obtain the coordinates of the hole centers \n"));
         MosPrintf(MIL_TEXT("so that they can be superimposed on each other.\n"));
         }

      // Run modelfinder to find circle shape.
      MIL_INT NumOccurences;
      auto MilModResultCircle = SimpleShapeSearch<SCircleShapeParamAndResult>(MilSystem, MilDisplay, MilDepthMap, M_DEFAULT, HOLE_RADIUS, i);
      MmodGetResult(MilModResultCircle, M_DEFAULT, M_NUMBER + M_TYPE_MIL_INT, &NumOccurences);
      if (NumOccurences < 1)
         {
         MosPrintf(MIL_TEXT("At least one circle must be found to continue.\n\n"));
         MosPrintf(MIL_TEXT("Press any key to end.\n\n"));
         return -1;
         }

      // Run modelfinder to find segment.
      //auto MilModResultSegment = SegmentSearch(MilSystem, MilDisplay, MilDepthMap, i);
      auto MilModResultSegment = SimpleShapeSearch<SSegmentShapeParamAndResult>(MilSystem, MilDisplay, MilDepthMap, SEGMENT_LENGTH, M_DEFAULT, i);
      MmodGetResult(MilModResultSegment, M_DEFAULT, M_NUMBER + M_TYPE_MIL_INT, &NumOccurences);
      if (NumOccurences < 1)
         {
         MosPrintf(MIL_TEXT("No segment were found.\n\n"));
         MosPrintf(MIL_TEXT("Press any key to end.\n\n"));
         return -1;
         }

      // Get the position of the circle and the axis from the segment.
      MIL_DOUBLE CircleXPos;
      MIL_DOUBLE CircleYPos;
      MmodGetResult(MilModResultCircle, 0, M_POSITION_X, &CircleXPos);
      MmodGetResult(MilModResultCircle, 0, M_POSITION_Y, &CircleYPos);
      SUnitVector2d AxisVector = GetAxisFromSegments(MilModResultSegment, MilDisplay, i);
      
      if (i == 0)
         {
         RefCircleXPos = CircleXPos;
         RefCircleYPos = CircleYPos;
         MosPrintf(MIL_TEXT("The same process is performed for every other point cloud...\n\n"));

         MosPrintf(MIL_TEXT("|-------------|---------|---------|---------|---------|---------|---------|\n"));
         MosPrintf(MIL_TEXT("| Altiz Index |    X    |    Y    |    Z    |    RX   |    RY   |    RZ   |\n"));
         MosPrintf(MIL_TEXT("|-------------|---------|---------|---------|---------|---------|---------|\n"));
         }

      // Create the matrix and save it to file.
      auto MilTransformMatrix = M3dgeoAlloc(M_DEFAULT_HOST, M_TRANSFORMATION_MATRIX, M_DEFAULT, M_UNIQUE_ID);
      GetMatrixTransform(MilTransformMatrix, AxisVector, RefCircleXPos, RefCircleYPos, CircleXPos, CircleYPos, i * BAR_HOLES_DISTANCE_X);
      MIL_DOUBLE Rx, Ry, Rz, Tx, Ty, Tz;
      M3dgeoMatrixGetTransform(MilTransformMatrix, M_TRANSLATION, &Tx, &Ty, &Tz, M_NULL, M_DEFAULT);
      M3dgeoMatrixSetTransform(MilTransformMatrix, M_ROTATION_Y, FindBarPlaneResult.Transformation.RY, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_ASSIGN);
      M3dgeoMatrixSetTransform(MilTransformMatrix, M_TRANSLATION, Tx, Ty, FindBarPlaneResult.Transformation.TZ + Tz, M_DEFAULT, M_COMPOSE_WITH_CURRENT);
      M3dgeoSave(BuildCameraTransformationMatrixName(i), MilTransformMatrix, M_DEFAULT);

      // Print transformation.
      M3dgeoMatrixGetTransform(MilTransformMatrix, M_TRANSLATION, &Tx, &Ty, &Tz, M_NULL, M_DEFAULT);
      M3dgeoMatrixGetTransform(MilTransformMatrix, M_ROTATION_XYZ, &Rx, &Ry, &Rz, M_NULL, M_DEFAULT);
      MosPrintf(MIL_TEXT("|%13d|%9.2f|%9.2f|%9.2f|%9.2f|%9.2f|%9.2f|\n"), i, Tx, Ty, Tz, Rx, Ry, Rz);
      
      // Transform the point cloud.
      M3dimMatrixTransform(MilToAlignPointCloud, MilToAlignPointCloud, MilTransformMatrix, M_DEFAULT);

      // Color the points.
      ColorCloud(MilToAlignPointCloud, M_RGB888(Colors[i].R, Colors[i].G, Colors[i].B));
      }

   // Merge and show the aligned point cloud.
   MergeAndShowAligned(MilSystem, std::vector<MIL_ID>(AlignmentData.MilToAlignPointClouds.begin(), AlignmentData.MilToAlignPointClouds.end()));

   return 0;
   }

//*****************************************************************************
// Merge point clouds from restored transformation matrices.
//*****************************************************************************
MIL_INT MergeFromRestoredMatrices(MIL_ID MilSystem)
   {
   MosPrintf(MIL_TEXT("If you already have you transformation matrices, you can simply restore them.\n"));

   // Restore and show the point cloud data.
   auto AlignmentData = RestoreAndShowAlignmentData(MilSystem, FILE_POINT_CLOUD_KEYBOARD);
   if(!AlignmentData.IsValid)
      return -1;

   // Transform the point clouds.
   for (MIL_INT i = 0; i < NUM_SCANS; i++)
      {
      M3ddispControl(AlignmentData.MilDisplay3d[i], M_UPDATE, M_DISABLE);
      auto MilTransformMatrix = M3dgeoRestore(BuildCameraTransformationMatrixName(i), MilSystem, M_DEFAULT, M_UNIQUE_ID);
      M3dimMatrixTransform(AlignmentData.MilToAlignPointClouds[i], AlignmentData.MilToAlignPointClouds[i], MilTransformMatrix, M_DEFAULT);
      }

   // Merge and show the aligned point cloud.
   MergeAndShowAligned(MilSystem, std::vector<MIL_ID>(AlignmentData.MilToAlignPointClouds.begin(), AlignmentData.MilToAlignPointClouds.end()));

   return 0;
   }

//*****************************************************************************
// Merge and show the aligned point cloud.
//*****************************************************************************
void MergeAndShowAligned(MIL_ID MilSystem, const std::vector<MIL_ID>& MilToAlignPointClouds)
   {
   // Use decimation for subsampling.
   MIL_UNIQUE_3DIM_ID MilSubsampleContext = M3dimAlloc(MilSystem, M_SUBSAMPLE_CONTEXT, M_DEFAULT, M_UNIQUE_ID);
   M3dimControl(MilSubsampleContext, M_SUBSAMPLE_MODE, M_SUBSAMPLE_DECIMATE);
   M3dimControl(MilSubsampleContext, M_ORGANIZATION_TYPE, M_ORGANIZED);
   M3dimControl(MilSubsampleContext, M_STEP_SIZE_X, MERGE_DECIMATION_STEP);
   M3dimControl(MilSubsampleContext, M_STEP_SIZE_Y, MERGE_DECIMATION_STEP);

   // Merge the point clouds   
   auto MilMergedPointClouds = MbufAllocContainer(MilSystem, M_PROC + M_DISP, M_DEFAULT, M_UNIQUE_ID);
   M3dimMerge(MilToAlignPointClouds, MilMergedPointClouds, M_DEFAULT, MilSubsampleContext, M_DEFAULT);

   // Display the transformed grabbed point clouds.
   auto MilAligned3dDisp = Alloc3dDisplayId(MilSystem);
   M3ddispSetView(MilAligned3dDisp, M_AUTO, M_BOTTOM_TILTED, M_DEFAULT, M_DEFAULT, M_DEFAULT);
   M3ddispSetView(MilAligned3dDisp, M_FLIP, M_AXIS_Y, M_DEFAULT, M_DEFAULT, M_DEFAULT);
   M3ddispSelect(MilAligned3dDisp, MilMergedPointClouds, M_SELECT, M_DEFAULT);
   MosPrintf(MIL_TEXT("The alignment of the 3D data is displayed.\n"));
   MosPrintf(MIL_TEXT("Press any key to continue the example.\n\n"));
   MosGetch();
   }

//*****************************************************************************
// Allocates a 3D display and returns its MIL identifier.
//*****************************************************************************
MIL_UNIQUE_3DDISP_ID Alloc3dDisplayId(MIL_ID MilSystem)
   {
   MappControl(M_DEFAULT, M_ERROR, M_PRINT_DISABLE);
   MIL_UNIQUE_3DDISP_ID MilDisplay3d = M3ddispAlloc(MilSystem, M_DEFAULT, MIL_TEXT("M_DEFAULT"), M_DEFAULT, M_UNIQUE_ID);
   MappControl(M_DEFAULT, M_ERROR, M_PRINT_ENABLE);

   if (!MilDisplay3d)
      {
      MosPrintf(MIL_TEXT("\n")
         MIL_TEXT("The current system does not support the 3D display.\n")
         MIL_TEXT("Press any key to continue.\n"));
      MosGetch();
      }
   return MilDisplay3d;
   }

//*******************************************************************************
// Allocates a 3D display if possible and displays a container or geometry.
//*******************************************************************************
MIL_UNIQUE_3DDISP_ID Alloc3dDisplayId(MIL_ID MilSystem,
                                      MIL_INT PositionX, MIL_INT PositionY,
                                      MIL_INT SizeX, MIL_INT SizeY,
                                      const MIL_STRING& Title)
   {
   auto Mil3dDisp = Alloc3dDisplayId(MilSystem);

   M3ddispControl(Mil3dDisp, M_TITLE, Title);
   M3ddispControl(Mil3dDisp, M_WINDOW_INITIAL_POSITION_X, PositionX);
   M3ddispControl(Mil3dDisp, M_WINDOW_INITIAL_POSITION_Y, PositionY);
   M3ddispControl(Mil3dDisp, M_SIZE_X, SizeX);
   M3ddispControl(Mil3dDisp, M_SIZE_Y, SizeY);

   return Mil3dDisp;
   }

//****************************************************************************
// Gets a certain number of distinct colors.
//****************************************************************************
std::vector<SBGR32Color> GetDistinctColors(MIL_INT NbColors)
   {
   auto MilPointCloudColors = MbufAllocColor(M_DEFAULT_HOST, 3, NbColors, 1, 8 + M_UNSIGNED, M_LUT, M_UNIQUE_ID);
   MgenLutFunction(MilPointCloudColors, M_COLORMAP_DISTINCT_256, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT);
   std::vector<SBGR32Color> Colors(NbColors);
   MbufGetColor(MilPointCloudColors, M_PACKED + M_BGR32, M_ALL_BANDS, (MIL_UINT32*)(&Colors[0]));
   return Colors;
   }

//****************************************************************************
// Color the container.
//****************************************************************************
void ColorCloud(MIL_ID MilPointCloud, MIL_INT Col)
   {
   MIL_INT SizeX = MbufInquireContainer(MilPointCloud, M_COMPONENT_RANGE, M_SIZE_X, M_NULL);
   MIL_INT SizeY = MbufInquireContainer(MilPointCloud, M_COMPONENT_RANGE, M_SIZE_Y, M_NULL);

   auto MilRefelectance = MbufInquireContainer(MilPointCloud, M_COMPONENT_REFLECTANCE, M_COMPONENT_ID, M_NULL);
   if (MilRefelectance)
      MbufFreeComponent(MilPointCloud, M_COMPONENT_REFLECTANCE, M_DEFAULT);

   auto MilReflectance = MbufAllocComponent(MilPointCloud, 3, SizeX, SizeY, 8 + M_UNSIGNED, M_IMAGE + M_PLANAR, M_COMPONENT_REFLECTANCE, M_NULL);
   MbufClear(MilReflectance, static_cast<MIL_DOUBLE>(Col));
   }

//****************************************************************************
// Check for required files to run the example.
//****************************************************************************
bool CheckForRequiredMILFile(const MIL_STRING& FileName)
   {
   MIL_INT FilePresent = M_NO;

   MappFileOperation(M_DEFAULT, FileName, M_NULL, M_NULL, M_FILE_EXISTS, M_DEFAULT, &FilePresent);
   if (FilePresent == M_NO)
      {
      MosPrintf(MIL_TEXT("The footage needed to run this example is missing. You need \n")
         MIL_TEXT("to obtain and apply a separate specific update to have it.\n\n"));
      MosPrintf(MIL_TEXT("Press any key to end.\n\n"));
      MosGetch();
      }

   return (FilePresent == M_YES);
   }

//****************************************************************************
// Build the name of the camera transformation matrix based on its index.
//****************************************************************************
MIL_STRING BuildCameraTransformationMatrixName(MIL_INT CameraIndex)
   {
   return CameraIndex == 0 ? FILE_MATRIX_PRIMARY : FILE_MATRIX_PREFIX + M_TO_STRING(CameraIndex + 1) + MIL_TEXT(".m3dgeo");
   }


