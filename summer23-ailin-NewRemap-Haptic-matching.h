#include "targetver.h"

#include <stdio.h>
#include <cstdlib>
#include <cmath>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <queue>
#include <string>


/**** BOOOST MULTITHREADED LIBRARY *********/
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>	//include asio in order to avoid the "winsock already declared problem"


#ifdef _WIN32
#include <windows.h>
#include <gl/glew.h> // http://glew.sourceforge.net/
#include <gl/wglew.h>
#include <gl\gl.h>            // Header File For The OpenGL32 Library
#include <gl\glu.h>            // Header File For The GLu32 Library
#include "glut.h"            // Header File For The GLu32 Library



#include <MMSystem.h>
#endif
/************ INCLUDE CNCSVISION LIBRARY HEADERS ****************/
#include "CShaderProgram.h"
#include "Mathcommon.h"
#include "GLUtils.h"
#include "VRCamera.h"
#include "CoordinatesExtractor.h"
#include "GLText.h"

#include "ParametersLoader.h"
#include "Util.h"
#include "VRCamera.h"
#include "BalanceFactor.h"
#include "ParStaircase.h"
#include "Staircase.h"
#include "TrialGenerator.h"
#include "BrownPhidgets.h"
#include <direct.h>
#include "Optotrak2.h"
#include "Marker.h"
#include "BrownMotorFunctions.h"

#include <random>
#include <functional> // to help with thread safe randomization thread



/********* NAMESPACE DIRECTIVES ************************/
using namespace std;
using namespace mathcommon;
using namespace Eigen;
using namespace util;
using namespace BrownMotorFunctions;
using namespace BrownPhidgets;

/*************** Variable Declarations ********************/
static const bool gameMode = true;
const float DEG2RAD = M_PI / 180;

/********* VARIABLES OBJECTS  **********************/
VRCamera cam;
Optotrak2 optotrak;
Screen screen;
CoordinatesExtractor headEyeCoords;

/***** CALIBRATION FILE *****/
#include "Calibration_017B.h"
static const Vector3d center(0, 0, focalDistance);
double mirrorAlignment = 45.0, screenAlignmentY = 0.0, screenAlignmentZ = 0.0;

/********** EYES AND MARKERS **********************/
Vector3d eyeLeft, eyeRight, eyeMiddle;
vector <Marker> markers;
double interoculardistance = 60.0;
int screen1 = 19, screen2 = 20, screen3 = 21;
int mirror1 = 6, mirror2 = 22;


//////////////////////////////// usually no change is needed until this point //////////////////////


/********** TRIAL SPECIFIC PARAMETERS ***************/
ParametersLoader parameters;
ParametersLoader par_s1;
ParametersLoader par_s2;
ParametersLoader par_s3;
ParametersLoader par_s4;
//BalanceFactor<double> trial; //if using costant stimuli
TrialGenerator<double> trial;


/*************************** INPUT AND OUTPUT ****************************/
// experiment directory
string experiment_directory = "C:/Users/labdomin/Documents/data/ailin/spring23-ailin-motionRemap/";

// paramters file directory and name
ifstream parametersFile;
string parametersFileName = experiment_directory + "parameters_spring23-ailin-motionRemap-matching-staircase.txt";

// response file
ofstream responseFile;
string responseFile_headers = "subjName\tIOD\tblockN\ttrialN\tstdDepth_text\tstdDepth_disp\tcmpDepth\tstd_is_first\trespond_first_deeper\trespond_cmp_deeper\tRT\tstairID\tstair_reversal\tascending\tdisplayDist_std\tdisplayDist_cmp\tvisualAngle\tTexDotsDensity\tTexDotRadius\tTexDotSepRatio";

string subjectName;

/**********	TRIALS **************/
int sessionNum = 0;
int totalBlkNum = 4;
int blkNum = 1;
int trialNum = 0;

enum parNames{A0, A1, B0, B1}; // 0 1 2 3
parNames use_par = A0;

parNames stairOrder[] = {A0, B1, A1, B0};

int trainNum_cap = 20;

double percentComplete = 0;
int repetition = 2;
int totalTrNum = 110; 
int stairID = 0, stair_reversal = 0, ascending = 0;

/********** STIMULUS SHAPE ***************/
// stimulus shape
double display_distance;
double visual_angle = 6.4; // stim diangonal size

//height and width
double stimulus_height_std = 70; //tan((DEG2RAD * visual_angle)/2) * 2 * (abs(display_distance));
double stimulus_height_cmp = 70; //tan((DEG2RAD * visual_angle)/2) * 2 * (abs(display_distance));
double stimulus_width = 70; //ratio_bgwidth_height * stimulus_height_std;
double stimulus_visiblewidth_std = 70; //ratio_visiblewidth_height * stimulus_height_std;
double stimulus_visiblewidth_cmp = 70; //ratio_visiblewidth_height * stimulus_height_std;
double ratio_width_height = 1.36;//1.3;
double ratio_visiblewidth_height = 1.1;//1.1;

// depths
double depth_std = 32;
double depth_stdDelta = 0;
double depth_std_text = 32;
double depth_std_disp = 32;

double depth_cmp = 30;
double depth_stdcmpDiff = depth_std - depth_cmp;

double depth_training_min = 10; // set in the subject file
int depth_training_range = 35; // set in the subject file
double depth_inc = 2; 


// jitter in distance
double jitter_z_std = 0;
double display_distance_jittered_std = display_distance + jitter_z_std;

double jitter_z_cmp = 0;
double display_distance_jittered_cmp = display_distance + jitter_z_cmp;
double dist_std_toEye = 0;
double dist_cmp_toEye = 0;

/********** STIMULUS VERTICES ***************/
struct CurveYLMap {
	std::vector<double> y_vec;
	std::vector<double> l_vec;
	double curve_depth;
	double curve_height;
	double step_size;
};

struct CurvePtsData {
	std::vector<double> y_vec;
	std::vector<double> z_vec;
	std::vector<double> l_vec;
	double curve_depth;
	double curve_height;
	double step_size;
};

struct Vec2 {
	float x, y;
};

struct ProjTexDots_ResizeMap {
	std::vector<Vec2> R_resize_vec;
	double depth_proj;
	double depth_origin;
	double del_l_proj;
};


struct TextureDotsData {
	std::vector<Vec2> dot_center_vec;
	std::vector<Vec2> R_resize_vec;
	Vec2 TexMapSize;
	float Radius;
	float margin_y;
};

struct VerticesData {
	std::vector<GLfloat> vertices_vec;
	std::vector<GLfloat> colors_vec;
	std::vector<GLfloat> light_normals_vec;
	std::vector<GLuint> indices_draw_triangle_vec;

};
VerticesData my_vertices_data_std;
VerticesData my_vertices_data_cmp;

struct ContourData {
	std::vector<Vector3f> vert_Rcontour;
	std::vector<Vector3f> vert_Lcontour;
};
ContourData my_contour_data_std;
ContourData my_contour_data_cmp;

/********** VERTEX RESOLUTION ***************/
int nr_curve_map = 10001;
int i_map_mid = (nr_curve_map - 1) / 2;

double del_l = 0.4;
int nr_points_width = 301; // nr of points in x direction
int nr_points_height_default = 251; // default
int nr_points_height = nr_points_height_default;
int total_ind = 0;

/********* TEXTURE *********/
float Tex_dot_density = 0.0145; //0.02;
float Tex_dot_radius = 2.6; // 2.2;
float Tex_dot_separation_ratio = 1.43;
int nr_X_Lat_TexDot = 5;
float jitter_Lat_TexDot = 0.4;

//blur edge
double drop_off_rate = 0.45;
double R_blur_fac = 2 / (1 + drop_off_rate);//1.28;

float vertex_col_max = 0.9;
float vertex_col_min = 0.2;

/********** LIGHTING ***************/
// light setting
float max_intensity = 0.8;
float amb_intensity_std = 0.4, amb_intensity_cmp = 0.4;
float lightDir_z = 0.8;

float depth_range_light = 28;
float depth_flat_light = 20;
float depth_deep_light = 48;
float ambVDif_flat_light = 1.0;
float ambVDif_deep_light = 0.6;
float max_intensity_range = 0.12;
float max_intensity_flat_light = 0.75;
float max_intensity_deep_light = 0.95;

GLfloat LightAmbient_std[] = { 0.4, 0.0f, 0.0f, 1.0f };
GLfloat LightDiffuse_std[] = { 0.4, 0.0f, 0.0f, 1.0f };
GLfloat LightAmbient_cmp[] = { 0.4, 0.0f, 0.0f, 1.0f };
GLfloat LightDiffuse_cmp[] = { 0.4, 0.0f, 0.0f, 1.0f };
GLfloat LightPosition[] = { 0.0f, 1.f, lightDir_z, 0.0f };

/********** ONLINE CONTROL ***************/
enum Stages { stimulus_preview, prep_trial, trial_fixate_first, trial_present_first, trial_fixate_second, trial_present_second, trial_respond, break_time, exp_completed };
Stages current_stage = stimulus_preview; // if just want to look at the stimuli, use the constant present stage

bool std_vs_cmp = true;
bool std_is_first = true;
bool respond_cmp_deeper = true;
bool respond_first_deeper = true;
bool stimulusDim_retinal_vs_physical = true;
bool training = true;
bool visibleInfo = true;

bool resetScreen_betweenRuns = false;
bool screenDistanceCorrect = false;
int errorID = 0;

// Timer variable, set for each trial 
Timer trial_timer;
double ElapsedTime, lastTimeStamp, responseTime;

double fixateTime = 500;
double viewTime = 650;

int TimerFrameCnt = 0;
int last_timer_frame_cnt = 0;
int fixateFrameCnt = 40;
int viewFrameCnt = 50;



/*********** for DEBUGGING **********/
bool testVisualStimuliOnly = false;

/*************************** FUNCTIONS ***********************************/
void initOptotrak();
void initMotors();
void initRendering();
void initVariables();
void initStreams();
void handleResize(int w, int h);
void initProjectionScreen(double _focalDist, const Affine3d& _transformation = Affine3d::Identity(), bool synchronous = true);
void updateTheMarkers();
void check_apparatus_alignment();

void cleanup();
void beepOk(int tone);

void initBlock();
void initTrial();
void onlineTrial();
void advanceTrial();
void initStimulus_std();
void initStimulus_cmp();

void drawStimulus();
void drawVertices(bool isStandard, double distShapeToEye);
void drawProgressBar();
void drawFixation(double displayDist);
void drawContours(const ContourData& contours_vert);
void drawInfo();

void buildSurface_congruent(double shapeWidth, double shapeHeight, double shapeDepth, double distShapeToEye, VerticesData& vertices_data, double contourPanelSeparation, ContourData& contour_data);
void buildSurface_incongruent(double shapeWidth, double shapeHeight, double dispDepth, double textDepth, double distShapeToEye, VerticesData& vertices_data, double contourPanelSeparation, ContourData& contour_data);
void buildSurfaceVertices_congruent(double shapeWidth, const CurvePtsData& textYCurve, TextureDotsData& TexDotsOnText, VerticesData& vertices_data);
void buildSurfaceVertices_TexDotsOnText(double shapeWidth, const CurvePtsData& dispYCurve, const CurvePtsData& textYCurve, double distShapeToEye, TextureDotsData& TexDotsOnText, VerticesData& vertices_data);
void buildContour(double ContourWidth, const CurvePtsData& dispYCurve, const CurvePtsData& textYCurve, float distShapeToEye, ContourData& new_contours_vert);
int buildCurve_byDelL(const CurveYLMap& input_curve_ylmap, CurvePtsData& output_curve);
void scanCurve(double shapeHeight, double shapeDepth, CurveYLMap& output_curve_ylmap);
void projectCurve(const CurveYLMap& curve_map_proj, double distShapeToEye, const CurvePtsData& origin_curve, CurvePtsData& output_curve_proj);
int buildCurve_byDelY(const CurveYLMap& input_curve_ylmap, CurvePtsData& output_curve);
void generateTexDots_new(float TM_X, float TM_Y, float dotDensity, float dotRadius, float dotSeparationRatio, TextureDotsData& outputTexDots);
void generateTexDots_hybrid(float TM_X, float TM_Y, float dotDensity, float dotRadius, float dotSeparationRatio, int nr_X_Lattice, float dotJitterScale_Lattice, TextureDotsData& outputTexDots);
void sampleTexDotsResize(const CurveYLMap& textCurveYLMap, const CurveYLMap& dispCurveYLMap, double distShapeToEye, ProjTexDots_ResizeMap& output_RszMap);
void projectTexDots(double distShapeToEye, const CurveYLMap& YLMap_origin, const CurveYLMap& YLMap_proj, const TextureDotsData& input_TexDots_origin, const ProjTexDots_ResizeMap& RszMap_proj, TextureDotsData& output_TexDots_proj);

double getZ(double shapeHeight, double shapeDepth, double vertexY);
double getTg(double shapeHeight, double shapeDepth, double Y);
double mapLtoY(const CurveYLMap& inputYLMap, double TargetL, int i_int = i_map_mid);
double mapYtoL(const CurveYLMap& inputYLMap, double input_y);
double SolveForZ_projected(double theHeight, double newDepth, double l, double y0, double z0);
float adjustAmbient(double textDepth, float maxInt, double rateAmbvsDiff_flat, double rateAmbvsDiff_deep, double Depth_flat, double Depth_deep);

// blur stuff
void initBlur();
void changeBlur();
void drawBlurred();
void blurPass(CShaderProgram blur_shader, unsigned int texID); // does one pass through using blur with a frame buffer
void drawBlurInput();