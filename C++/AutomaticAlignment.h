//***************************************************************************************/
// 
// File name: AutomaticAlignment.h
//
// Synopsis: Implementation of the methods used to align the different point clouds of the
//           bar with holes.
// 
// Copyright © Matrox Electronic Systems Ltd., 1992-YYYY.
// All Rights Reserved
//*************************************************************************************/
#include<mil.h>

//*****************************************************************************
// Constants.
//*****************************************************************************
static const MIL_DOUBLE DIV_PI_180 = 0.017453292519943295769236907684886;
static const MIL_DOUBLE DIV_180_PI = 57.295779513082320866997945294156;

// Depth map.
static const MIL_INT FILL_GAPS_THRESHOLD_PIXEL = 2;

// Hole circles.
static const MIL_INT    MAX_NUMBER_OF_HOLE_CIRCLES       = 18;
static const MIL_DOUBLE CIRCLE_SEARCH_SMOOTHNESS         = 90.0;
static const MIL_DOUBLE CIRCLE_SEARCH_ACCEPTANCE         = 75.0;
static const MIL_DOUBLE CIRCLE_SEARCH_SAGITTA_TOLERANCE  = 50.0;
static const MIL_DOUBLE HOLE_RADIUS                      = 20.0;

// Bar segments.
static const MIL_INT    MAX_NUMBER_OF_SEGMENTS    = 30;
static const MIL_DOUBLE SEGMENT_SEARCH_SMOOTHNESS = 90.0;
static const MIL_DOUBLE SEGMENT_SEARCH_ACCEPTANCE = 75.0;
static const MIL_INT    SEGMENT_LENGTH            = 100;
static const MIL_DOUBLE MIN_VALID_SEGMENT_X       = 15;

//****************************************************************************
// Structure representing an axis with a position and direction.
//****************************************************************************
struct SUnitVector2d
   {
   SUnitVector2d(MIL_DOUBLE X0, MIL_DOUBLE Y0, MIL_DOUBLE X1, MIL_DOUBLE Y1)
      {
      Vx = X1 - X0;
      Vy = Y1 - Y0;
      MIL_DOUBLE Length = sqrt(Vx * Vx + Vy * Vy);
      Vx /= Length;
      Vy /= Length;
      }

   MIL_DOUBLE Vx;
   MIL_DOUBLE Vy;
   };

//****************************************************************************
// Create depth map.
//****************************************************************************
MIL_UNIQUE_BUF_ID CreateDepthMap(MIL_ID MilSystem, MIL_ID MilPointCloud)
   {
   // Set the pixel size aspect ratio to be unity.
   const MIL_DOUBLE PixelAspectRatio = 1.0;

   // Allocate context for calculating the depthmap sizes.
   auto MilMapSizeContext = M3dimAlloc(MilSystem, M_CALCULATE_MAP_SIZE_CONTEXT, M_DEFAULT, M_UNIQUE_ID);

   // Calculate the size required for the depth map.
   MIL_INT DepthMapSizeX = 0;
   MIL_INT DepthMapSizeY = 0;
   M3dimControl(MilMapSizeContext, M_CALCULATE_MODE, M_ORGANIZED);
   M3dimControl(MilMapSizeContext, M_PIXEL_ASPECT_RATIO, PixelAspectRatio);
   M3dimCalculateMapSize(MilMapSizeContext, MilPointCloud, M_NULL, M_DEFAULT, &DepthMapSizeX, &DepthMapSizeY);
   MIL_UNIQUE_BUF_ID MilDepthMap = MbufAlloc2d(MilSystem, DepthMapSizeX , DepthMapSizeY +5, M_UNSIGNED + 8, M_IMAGE | M_PROC | M_DISP, M_UNIQUE_ID);

   // Calibrate the depth map based on the given point cloud.
   M3dimCalibrateDepthMap(MilPointCloud, MilDepthMap, M_NULL, M_NULL, PixelAspectRatio, M_DEFAULT, M_DEFAULT);

   // Control the options of the fill gap context to yield better results.
   MIL_UNIQUE_3DIM_ID MilFillGapsContext = M3dimAlloc(MilSystem, M_FILL_GAPS_CONTEXT, M_DEFAULT, M_UNIQUE_ID);
   M3dimControl(MilFillGapsContext, M_FILL_THRESHOLD_X, FILL_GAPS_THRESHOLD_PIXEL);
   M3dimControl(MilFillGapsContext, M_FILL_THRESHOLD_Y, FILL_GAPS_THRESHOLD_PIXEL);
   M3dimControl(MilFillGapsContext, M_INPUT_UNITS, M_PIXEL);

   // Project the point cloud in a point based mode.
   M3dimProject(MilPointCloud, MilDepthMap, M_NULL, M_POINT_BASED, M_DEFAULT, M_DEFAULT, M_DEFAULT);
   M3dimFillGaps(MilFillGapsContext, MilDepthMap, M_NULL, M_DEFAULT);

   return MilDepthMap;
   }

//****************************************************************************
// Circle shape finder parameters and results management.
//****************************************************************************
struct SCircleShapeParamAndResult
   {
   static const MIL_INT ShapeFinderType = M_SHAPE_CIRCLE;
   static const MIL_INT ShapeDefineType = M_CIRCLE;
   static const MIL_CONST_TEXT_PTR FindMessage() { return MIL_TEXT("Circle finder is used to find the position of the bar.\n\n"); }

   static void SetupShapeContextResult(MIL_ID MilSearchContext, MIL_ID MilResult)
      {
      MmodControl(MilSearchContext, M_CONTEXT, M_DETAIL_LEVEL, M_VERY_HIGH);
      MmodControl(MilSearchContext, M_CONTEXT, M_SMOOTHNESS, CIRCLE_SEARCH_SMOOTHNESS);
      MmodControl(MilSearchContext, M_ALL, M_ACCEPTANCE, CIRCLE_SEARCH_ACCEPTANCE);
      MmodControl(MilSearchContext, M_ALL, M_SAGITTA_TOLERANCE, CIRCLE_SEARCH_SAGITTA_TOLERANCE);
      MmodControl(MilSearchContext, 0, M_NUMBER, MAX_NUMBER_OF_HOLE_CIRCLES);
      MmodControl(MilResult, M_GENERAL, M_RESULT_OUTPUT_UNITS, M_WORLD);
      }

   static void PrintShapeResult(MIL_ID MilResult, MIL_INT NumResults)
      {
      MosPrintf(MIL_TEXT("The circles were found in the target image:\n\n"));
      MosPrintf(MIL_TEXT("|--------------|---------|---------|---------|---------|\n"));
      MosPrintf(MIL_TEXT("| Circle Index |    X    |    Y    | Radius  |  Score  |\n"));
      MosPrintf(MIL_TEXT("|--------------|---------|---------|---------|---------|\n"));

      for(int i = 0; i < NumResults; i++)
         {
         MIL_DOUBLE X, Y, Radius, Score;
         MmodGetResult(MilResult, i, M_POSITION_X, &X);
         MmodGetResult(MilResult, i, M_POSITION_Y, &Y);
         MmodGetResult(MilResult, i, M_RADIUS, &Radius);
         MmodGetResult(MilResult, i, M_SCORE, &Score);
         MosPrintf(MIL_TEXT("|%14d|%9.2f|%9.2f|%9.2f|%9.2f|\n"), i, X, Y, Radius, Score);
         }
      }
   };

//****************************************************************************
// Segment shape finder parameters and results management.
//****************************************************************************
struct SSegmentShapeParamAndResult
   {
   static const MIL_INT ShapeFinderType = M_SHAPE_SEGMENT;
   static const MIL_INT ShapeDefineType = M_SEGMENT;
   static const MIL_CONST_TEXT_PTR FindMessage() { return MIL_TEXT("Segment finder is used to find the displacement axis.\n\n"); }

   static void SetupShapeContextResult(MIL_ID MilSearchContext, MIL_ID MilResult)
      {
      MmodControl(MilSearchContext, M_CONTEXT, M_SMOOTHNESS, SEGMENT_SEARCH_SMOOTHNESS);
      MmodControl(MilSearchContext, M_ALL, M_ACCEPTANCE, SEGMENT_SEARCH_ACCEPTANCE);
      MmodControl(MilSearchContext, 0, M_NUMBER, MAX_NUMBER_OF_SEGMENTS);
      MmodControl(MilResult, M_GENERAL, M_RESULT_OUTPUT_UNITS, M_WORLD);
      }

   static void PrintShapeResult(MIL_ID MilResult, MIL_INT NumResults)
      {
      MosPrintf(MIL_TEXT("The segments were found in the target image:\n\n"));
      MosPrintf(MIL_TEXT("|---------------|---------|---------|\n"));
      MosPrintf(MIL_TEXT("| Segment Index | Length  |  Score  |\n"));
      MosPrintf(MIL_TEXT("|---------------|---------|---------|\n"));

      for(int i = 0; i < NumResults; i++)
         {
         MIL_DOUBLE Score, Length;
         MmodGetResult(MilResult, i, M_SCORE, &Score);
         MmodGetResult(MilResult, i, M_LENGTH, &Length);
         MosPrintf(MIL_TEXT("|%15d|%9.2f|%9.2f|\n"), i, Length, Score);
         }
      }
   };

//****************************************************************************
// Find shapes on depth map.
//****************************************************************************
template <class CModShapeFinder>
MIL_UNIQUE_MOD_ID SimpleShapeSearch(MIL_ID MilSystem, MIL_ID MilDisplay, MIL_ID MilDepthMap,
                                    MIL_DOUBLE DefineParam1, MIL_DOUBLE DefineParam2, MIL_INT Iteration)
   {
   // Allocate a graphic list to hold the subpixel annotations to draw.
   auto MilGraphicList2d = MgraAllocList(MilSystem, M_DEFAULT, M_UNIQUE_ID);

   // Associate the graphic list to the display for annotations.
   MdispControl(MilDisplay, M_ASSOCIATED_GRAPHIC_LIST_ID, MilGraphicList2d);

   // Allocate a shape finder context.
   auto MilSearchContext = MmodAlloc(MilSystem, CModShapeFinder::ShapeFinderType, M_DEFAULT, M_UNIQUE_ID);

   // Allocate a shape finder result buffer.
   auto MilResult = MmodAllocResult(MilSystem, CModShapeFinder::ShapeFinderType, M_UNIQUE_ID);

   // Define the model.
   MmodDefine(MilSearchContext, CModShapeFinder::ShapeDefineType, DefineParam1, DefineParam2, M_DEFAULT, M_DEFAULT, M_DEFAULT);

   // Setup the context and result.
   CModShapeFinder::SetupShapeContextResult(MilSearchContext, MilResult);

   // Preprocess the search context.
   MmodPreprocess(MilSearchContext, M_DEFAULT);

   // Find the model.
   MmodFind(MilSearchContext, MilDepthMap, MilResult);

   if(Iteration == 0)
      {
      MosPrintf(CModShapeFinder::FindMessage());

      // Get the number of models found.
      MIL_INT NumResults;
      MmodGetResult(MilResult, M_DEFAULT, M_NUMBER + M_TYPE_MIL_INT, &NumResults);

      // If a model was found above the acceptance threshold.
      if((NumResults >= 1) && (NumResults <= MAX_NUMBER_OF_HOLE_CIRCLES))
         {
         // Print the results.
         CModShapeFinder::PrintShapeResult(MilResult, NumResults);

         // Draw edges, position and box over the occurrences that were found.
         MgraControl(M_DEFAULT, M_COLOR, M_COLOR_RED);
         MmodDraw(M_DEFAULT, MilResult, MilGraphicList2d, M_DRAW_POSITION, M_DEFAULT, M_DEFAULT);
         MgraControl(M_DEFAULT, M_COLOR, M_COLOR_GREEN);
         MmodDraw(M_DEFAULT, MilResult, MilGraphicList2d, M_DRAW_EDGES, M_DEFAULT, M_DEFAULT);
         }
      else
         {
         MosPrintf(MIL_TEXT("The model was not found or the number of models ")
                   MIL_TEXT("found is greater than\n"));
         MosPrintf(MIL_TEXT("the specified maximum number of occurrences!\n\n"));
         }

      // Wait for a key to be pressed.
      MosPrintf(MIL_TEXT("Press any key to continue.\n\n"));
      MosGetch();
      }
   return MilResult;
   }

//****************************************************************************
// Get indices of segments that could be the displacement axis of the bar.
//****************************************************************************
std::vector<MIL_INT> FindValidSizeXSegmentIndices(const std::vector<MIL_DOUBLE>& XPosition,
                                                  const std::vector<MIL_DOUBLE>& CenterXPosition)
   {
   std::vector<MIL_INT> ValidIndex;
   for(int i = 0; i < XPosition.size(); i++)
      {
      MIL_DOUBLE Vx = CenterXPosition[i] - XPosition[i];
      if(abs(Vx) > MIN_VALID_SEGMENT_X)
         ValidIndex.push_back(i);
      }

   return ValidIndex;
   }

//****************************************************************************
// Get the displacement axis according to the edge of the bar.
//****************************************************************************
SUnitVector2d GetAxisFromSegments(MIL_ID MilResult, MIL_ID MilDisplay, MIL_INT Iteration)
   {
   // Get the results of the segment search.
   std::vector<MIL_DOUBLE>   EndXPos, EndYPos, CenterXPosition, CenterYPosition;
   MmodGetResult(MilResult, M_DEFAULT, M_END_POS_X, EndXPos);
   MmodGetResult(MilResult, M_DEFAULT, M_END_POS_Y, EndYPos);
   MmodGetResult(MilResult, M_DEFAULT, M_CENTER_X, CenterXPosition);
   MmodGetResult(MilResult, M_DEFAULT, M_CENTER_Y, CenterYPosition);

   auto ValidSegmentIndices = FindValidSizeXSegmentIndices(EndXPos, CenterXPosition);

   // Get the valid segment whose center Y is the greatest. If no segments are considered
   // valid, the first one will be kept.
   MIL_INT MaxIndex = 0;
   if(ValidSegmentIndices.size() > 0)
      {
      MIL_DOUBLE MaxCenterY = CenterYPosition[ValidSegmentIndices[0]];
      for(int i = 0; i < ValidSegmentIndices.size(); i++)
         {
         if(CenterYPosition[ValidSegmentIndices[i]] >= MaxCenterY)
            {
            MaxCenterY = CenterYPosition[ValidSegmentIndices[i]];
            MaxIndex = ValidSegmentIndices[i];
            }
         }
      }

   if (Iteration == 0)
      {
      // Draw the edge in a graphic list associated to the display.
      MIL_UNIQUE_GRA_ID MilGraphicList2d = MgraAllocList(M_DEFAULT_HOST, M_DEFAULT, M_UNIQUE_ID);
      MdispControl(MilDisplay, M_ASSOCIATED_GRAPHIC_LIST_ID, MilGraphicList2d);
      MgraControl(M_DEFAULT, M_LINE_THICKNESS, 3);
      MmodDraw(M_DEFAULT, MilResult, MilGraphicList2d, M_DRAW_EDGES, MaxIndex, M_DEFAULT);

      MosPrintf(MIL_TEXT("This edge will be used to get the displacement axis.\n"));
      MosPrintf(MIL_TEXT("Press any key to continue.\n\n"));
      MosGetch();
      }

   return {EndXPos[MaxIndex], EndYPos[MaxIndex], CenterXPosition[MaxIndex], CenterYPosition[MaxIndex]};
   }

//****************************************************************************
// Get alignement matrix relative to first camera.
//****************************************************************************
void GetMatrixTransform(MIL_ID MilMatrix, SUnitVector2d SegmentVector, MIL_DOUBLE RefCircleX, MIL_DOUBLE RefCircleY, MIL_DOUBLE CircleX, MIL_DOUBLE CircleY, MIL_INT DistanceX)
   {
   MIL_DOUBLE TxCircle = RefCircleX - CircleX;
   MIL_DOUBLE TyCircle = RefCircleY - CircleY;
   M3dgeoMatrixSetTransform(MilMatrix, M_TRANSLATION, TxCircle - DistanceX * SegmentVector.Vx, TyCircle - DistanceX * SegmentVector.Vy, 0, M_DEFAULT, M_ASSIGN);
   }



