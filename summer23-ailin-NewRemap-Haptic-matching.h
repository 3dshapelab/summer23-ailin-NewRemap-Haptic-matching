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
#include "CShaderProgram.h" // shader program
#include "glext.h" // opengl extensions for multisampling
#include <gl\gl.h>            // Header File For The OpenGL32 Library
#include <gl\glu.h>            // Header File For The GLu32 Library
#include "glut.h"            // Header File For The GLu32 Library
#include <MMSystem.h>
#endif

/************ INCLUDE CNCSVISION LIBRARY HEADERS ****************/
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
double mirrorAlignment = 999.0, screenAlignmentY = 999.0, screenAlignmentZ = 999.0;

/********** EYES AND MARKERS **********************/
Vector3d eyeLeft, eyeRight, eyeMiddle;
vector <Marker> markers;
double interoculardistance = 60.0;
int screen1 = 19, screen2 = 20, screen3 = 21;
int mirror1 = 6, mirror2 = 22;


/*************************** INPUT AND OUTPUT ****************************/
string subjectName;
// experiment directory
string experiment_directory = "C:/Users/labdomin/Documents/data/ailin/summer23-ailin-NewRemap-Haptic/";

// paramters file directory and name
ParametersLoader parameters_subj;
ParametersLoader parameters;
string parametersFileName_subj = experiment_directory + "parameters_summer23-ailin-NewRemap-Haptic-MASTER.txt";
string parametersFileName = experiment_directory + "ParametersFiles/parameters_Haptic_matching_stair.txt";

// response file
ofstream responseFile;
string responseFile_headers = "subjName\treinforceTexture\tIOD\tblockN\ttrialN\tdisplayDist_std\tdisplayDist_cmp\tstdDepth_text\tstdDepth_disp\tcmpDepth\tstd_is_first\trespond_first_deeper\trespond_cmp_deeper\tstd_num_texDots\tcmp_num_texDots\tstair_reversal\tRT";

/********** TRIAL SPECIFIC PARAMETERS ***************/
//BalanceFactor<double> trial; //if using costant stimuli
TrialGenerator<double> trial;//if using staircase: 
int stairID = 0, stair_reversal = 0, ascending = 0;

int targetCueID; // 0 for texture; 1 for dispairty
bool reinforce_texture_disparity;

int sessionNum = 0;
int totalBlkNum = 1;
int blkNum = 1;

int totalTrNum = 90;
int trialNum = 0;
double percentComplete = 0;
int trainNum_cap = 10;

/********** STIMULUS SHAPE ***************/
// display
double display_distance;
double visualTarget_X = 19.6;
double visual_angle = 7.4; // stim diangonal size

// jitter in distance
double jitter_z_std = 0, jitter_z_cmp = 0;
double display_distance_jittered_std = display_distance + jitter_z_std;
double display_distance_jittered_cmp = display_distance + jitter_z_cmp;
double dist_toEye_std, dist_toEye_cmp;

//height and width
double stimulus_height = 70; //tan((DEG2RAD * visual_angle)/2) * 2 * (abs(display_distance));
double stimulus_width = 70; //ratio_bgwidth_height * stimulus_height;
double stimulus_visiblewidth = 70; //ratio_visiblewidth_height * stimulus_height;
double ratio_width_height = 1.36;//1.3;
double ratio_visiblewidth_height = 1.15;//1.1;

// depths
double depth_stdDelta = 0;
double depth_std_text = 32;
double depth_std_disp = 32;
double depth_std = 32;
double depth_cmp = 30;

double depth_training_min = 10; // set in the subject file
int depth_training_range = 35; // set in the subject file
double depth_inc = 2;


/********** STIMULUS VERTICES ***************/
struct Vec2 {
	float x, y;
};

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


struct TextureDotsData {
	std::vector<Vec2> dot_center_vec;
	Vec2 TexMapSize;
	float Radius;
	float margin_y;
};

struct VerticesData {
	std::vector<GLfloat> vertices_vec;
	std::vector<GLfloat> colors_vec;
	std::vector<GLfloat> light_normals_vec;
	std::vector<GLfloat> textuv_vec;
	std::vector<GLuint> indices_draw_triangle_vec;

};

struct ContourData {
	std::vector<Vector3f> vert_Rcontour;
	std::vector<Vector3f> vert_Lcontour;
};

VerticesData my_vertices_data_std, my_vertices_data_cmp;
ContourData my_contour_data_std, my_contour_data_cmp;
float dot_number, dot_number_std, dot_number_cmp;

/********** TEXTURE SURF ***************/
int nr_curve_map = 10001;

double lengthFactor_TM = 1.4; //for a depth with a curve length of l, the TM length is multiplied by this factor.
double del_l = 0.4;

int nr_points_width = 251; // nr of points in x direction
int nr_points_height_default = 201; // default
int nr_points_height = nr_points_height_default;
int total_ind = 0;

/********* TEXTURE *********/
// self-generated
float Tex_dot_radius = 2.7;
float Tex_dot_density = 0.019;
float Tex_dot_separation_ratio = 1.10;

float texture_col_max = 1.0;
float texture_col_min = 0.1;
int TexDot_Lat_nr = 4;
float TexDot_Lat_jitter = 0.4;

//blur edge
double drop_off_rate = 0.75;
double R_intersect_factor = 2 / (1 + drop_off_rate);


/********** LIGHT SHADING ***************/
float max_intensity = 1.0;
float light_amb = 0.3;
float light_dif_std = 0.5, light_dif_cmp = 0.5;
float lightDir_z = 0.5;
double light_depthMin = 24;
double light_depthMax = 40;

GLfloat LightAmbient[] = { light_amb, 0.0f, 0.0f, 1.0f };
GLfloat LightDiffuse_std[] = { 0.4, 0.0f, 0.0f, 1.0f };
GLfloat LightDiffuse_cmp[] = { 0.4, 0.0f, 0.0f, 1.0f };
GLfloat LightPosition[] = { 0.0f, 1.f, lightDir_z, 0.0f };

/********** STATE VARIABLES ***************/
enum Stages { stimulus_preview, prep_trial, trial_fixate_first, trial_present_first, trial_fixate_second, trial_present_second, trial_respond, break_time, exp_completed, trial_error };
Stages current_stage = stimulus_preview; // if just want to look at the stimuli, use the constant present stage

bool std_vs_cmp = true;
bool std_is_first = true;
bool respond_cmp_deeper = true;
bool respond_first_deeper = true;
bool stimulusDim_retinal_vs_physical = true;
bool training = true;
bool visibleInfo = true;
bool stimulus_std_built, stimulus_cmp_built;

/********** TIME ***************/
// Timer variable, set for each trial 
Timer trial_timer;
double ElapsedTime, lastTimeStamp, responseTime;

double fixateTime = 500;
double viewTime = 650;

int TimerFrameCnt = 0;
int last_timer_frame_cnt = 0;
int fixateFrameCnt = 40;
int viewFrameCnt = 50;

/********** CONTROL OPTIONS ***************/
bool resetScreen_betweenRuns = false;

/*********** for DEBUGGING **********/

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

void drawStimulus();
void drawInfo();
void drawProgressBar();
void drawFixation(double displayDist);

void initBlock();
void initTrial();
void onlineTrial();
void advanceTrial();

void initStimulus_std();
void initStimulus_cmp();

bool generateTexture(float TM_X, float TM_Y, float dotDensity, float dotRadius, float dotSeparationRatio_init, int nr_X_Lattice, float dotJitterScale_Lattice, TextureDotsData& outputTexDots);
void buildVertices_Texture(double shapeWidth, const CurvePtsData& dispYCurve, const CurvePtsData& textYCurve, double distShapeToEye, TextureDotsData& TexDotsOnText, VerticesData& vertices_data);
void buildContour_Texture(double ContourWidth, const CurvePtsData& dispYCurve, const CurvePtsData& textYCurve, float distShapeToEye, ContourData& new_contours_vert);
bool buildTextureSurface(double shapeWidth, double shapeHeight, double dispDepth, double textDepth, double distShapeToEye, double contourPanelSeparation, VerticesData& vertices_data, ContourData& contours_vert);
void drawTextureSurface(bool isStandard, double distShapeToEye);
void drawContours(const ContourData& contours_vert);

double getZ(double shapeHeight, double shapeDepth, double vertexY);
double getTg(double shapeHeight, double shapeDepth, double Y);
double SolveForZ_projected(double theHeight, double newDepth, double l, double y0, double z0);
void scanCurve(double shapeHeight, double shapeDepth, CurveYLMap& output_curve_ylmap);
void projectCurve(const CurveYLMap& curve_map_proj, double distShapeToEye, const CurvePtsData& origin_curve, CurvePtsData& output_curve_proj);
Vector3d projectPoint(double shapeHeight, double newDepth, double distShapeToEye, Vector3d fromPoint);
float adjustDiffLight(double textDepth, float maxInt, float ambInt, double Depth_flat, double Depth_deep);
void makeParsFileCopy(string filename_original, string filename_copy);