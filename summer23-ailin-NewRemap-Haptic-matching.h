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

#include "SOIL.h"//Library for texture mapping

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
//BalanceFactor<double> trial; //if using costant stimuli
TrialGenerator<double> trial;//if using staircase: 


/*************************** INPUT AND OUTPUT ****************************/
// experiment directory
string experiment_directory = "C:/Users/labdomin/Documents/data/ailin/summer23-ailin-NewRemap-Haptic/";

// paramters file directory and name
ParametersLoader parameters_subj;
ParametersLoader parameters;

// paramters file directory and name

string parametersFileName_subj = experiment_directory + "ParametersFiles/Haptic_Subj.txt";

string parametersFileName = experiment_directory + "ParametersFiles/parameters_Haptic_matching.txt";

// response file
ofstream responseFile;
string responseFile_headers = "subjName\tIOD\tblockN\ttrialN\tdisplayDistance\tvisualAngle\tshapeHeight\ttexnum_std\tnomralizer_std\ttexnum_cmp\tnomralizer_cmp\tstdDepth_text\tstdDepth_disp\tcmpDepth\tstd_is_first\trespond_first_deeper\trespond_cmp_deeper\tRT\tstairID\tstair_reversal\tascending";

string subjectName;

/**********	TRIALS **************/
int sessionNum = 0;
int totalBlkNum = 1;
int blkNum = 1;
int trialNum = 0;

int trainNum_cap = 20;

double percentComplete = 0;
int repetition = 2;
int totalTrNum = 120;
int stairID = 0, stair_reversal = 0, ascending = 0;
/********** STIMULUS SHAPE ***************/
// stimulus shape
double display_distance;
double visualTarget_X = 21;
double visual_angle = 8.0; // stim diangonal size

//height and width
double stimulus_height = 70; //tan((DEG2RAD * visual_angle)/2) * 2 * (abs(display_distance));
double stimulus_width = 70; //ratio_bgwidth_height * stimulus_height;

double ratio_bgwidth_height = 1.4;//1.3;
double stimulus_bgwidth = 70;
double ratio_visiblewidth_height = 1.2;//1.1;
double stimulus_visiblewidth = 70; //ratio_visiblewidth_height * stimulus_height;

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

// shapes
enum shapeTypes { Ridge, Gaussian, Cosine, CosineRidge };
shapeTypes current_shape = CosineRidge;
int shapeID = 3;
double gauss_sig_height_ratio = 0.16;

/********** BLOCKING PANELS ***************/
enum panelStates { no_aperture, black_aperture, red_aperture };
panelStates panel_state = black_aperture; //red_aperture;



/********** STIMULUS VERTICES ***************/
int nr_points = 201;
// vectors storing vertices data
std::vector<GLfloat> vertices_vec_std;
std::vector<GLfloat> texcoors_vec_std;
std::vector<GLfloat> colors_vec_std;
std::vector<GLfloat> normals_vec_std;
std::vector<GLuint> indices_draw_triangle_vec_std;

std::vector<Vector3d> vertContainer_std_Rcontour;
std::vector<Vector3d> vertContainer_std_Lcontour;


std::vector<GLfloat> vertices_vec_cmp;
std::vector<GLfloat> texcoors_vec_cmp;
std::vector<GLfloat> colors_vec_cmp;
std::vector<GLfloat> normals_vec_cmp;
std::vector<GLuint> indices_draw_triangle_vec_cmp;

std::vector<Vector3d> vertContainer_cmp_Rcontour;
std::vector<Vector3d> vertContainer_cmp_Lcontour;

//std::vector<Vector3d> vertContainer_std_Rcontour_Leye;
//std::vector<Vector3d> vertContainer_std_Rcontour_Reye;
//std::vector<Vector3d> vertContainer_std_Lcontour_Leye;
//std::vector<Vector3d> vertContainer_std_Lcontour_Reye;

//std::vector<Vector3d> vertContainer_cmp_Rcontour_Leye;
//std::vector<Vector3d> vertContainer_cmp_Rcontour_Reye;
//std::vector<Vector3d> vertContainer_cmp_Lcontour_Leye;
//std::vector<Vector3d> vertContainer_cmp_Lcontour_Reye;


// texture map
GLuint loaded_textures[51];
int texnum_std = 10;
int texnum_cmp = 1;
double normalizer_to_uv_base = 90;
double normalizer_to_uv_std = 90;
double normalizer_to_uv_cmp = 90;
double u_offset = 0.05;
double v_offset = 0.05;

// light setting
float max_intensity = 0.8;
float amb_intensity_std = 0.3, amb_intensity_cmp = 0.3;
float lightDir_z = 0.6;

GLfloat LightAmbient_std[] = { 0.4, 0.0f, 0.0f, 1.0f };
GLfloat LightDiffuse_std[] = { 0.4, 0.0f, 0.0f, 1.0f };
GLfloat LightAmbient_cmp[] = { 0.4, 0.0f, 0.0f, 1.0f };
GLfloat LightDiffuse_cmp[] = { 0.4, 0.0f, 0.0f, 1.0f };
GLfloat LightPosition[] = { 0.0f, 1.f, lightDir_z, 0.0f };

/********** STATE VARIABLES ***************/
enum Stages { stimulus_preview, prep_trial, trial_fixate_first, trial_present_first, trial_fixate_second, trial_present_second, trial_respond, break_time, exp_completed };
Stages current_stage = stimulus_preview; // if just want to look at the stimuli, use the constant present stage

bool std_vs_cmp = true;
bool std_is_first = true;
bool respond_cmp_deeper = true;
bool respond_first_deeper = true;
bool stimulusDim_retinal_vs_physical = true;
bool training = true;
bool visibleInfo = true;

bool resetScreen_betweenRuns = true;

int errorID = 0;
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

void initBlock();
void initTrial();
void onlineTrial();
void advanceTrial();

void initStimulus_std();
void initStimulus_cmp();
void drawStimulus();
void drawInfo();

double NewtonSolver_Cosine(double theHeight, double newDeth_zDenom, double constCos, double l_translatedDist, double y0, double z0);
void buildVertices_congruent(bool isStandard, double shapeDepth, double textNormalizer);
void buildVertices_incongruent(bool isStandard, double textDepth, double dispDepth, double displayDist, double textNormalizer);
void drawVertices(bool isStandard, int texNum, double displayDist, double dispDepth);

void drawProgressBar();
void drawBlockingPanels(double pandelSeparation);
void drawFixation(double displayDist);


void drawPanels(bool isStandard, double displayDist, double dispDepth);
int LoadGLTextures();
float adjustAmbient(double textDepth, float maxInt, double rateAmbvsDiff_flat, double rateAmbvsDiff_deep, double Depth_flat, double Depth_deep);
