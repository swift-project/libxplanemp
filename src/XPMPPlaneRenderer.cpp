/*
 * Copyright (c) 2004, Ben Supnik and Chris Serio.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "XPMPPlaneRenderer.h"
#include "XPMPMultiplayer.h"
#include "XPMPMultiplayerCSL.h"
#include "XPMPMultiplayerVars.h"
#include "XPMPMultiplayerObj.h"
#include "XPMPMultiplayerObj8.h"

#include "XPLMGraphics.h"
#include "XPLMDisplay.h"
#include "XPLMCamera.h"
#include "XPLMPlanes.h"
#include "XPLMUtilities.h"
#include "XPLMDataAccess.h"

#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <bitset>

#if IBM
#include <GL/gl.h>
#elif APL
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <vector>
#include <string>
#include <set>
#include <map>

// Turn this on to get a lot of diagnostic info on who's visible, etc.
#define		DEBUG_RENDERER 0
#define		DEBUG_TCAS 0
// Turn this on to put rendering stats in datarefs for realtime observatoin.
#define		RENDERER_STATS 0

// Maximum altitude difference in feet for TCAS blips
#define		MAX_TCAS_ALTDIFF		10000

// Even in good weather we don't want labels on things
// that we can barely see.  Cut labels at 5 km.
#define		MAX_LABEL_DIST			5000.0

extern bool	gHasControlOfAIAircraft;

struct MultiplayerDatarefs_t {
	XPLMDataRef x = nullptr;
	XPLMDataRef y = nullptr;
	XPLMDataRef z = nullptr;
	XPLMDataRef pitch = nullptr;	// theta
	XPLMDataRef roll = nullptr;		// phi
	XPLMDataRef heading = nullptr;	// psi
	bool isReserved = false;		// plane index is used?
	bool ok() const { return x && y && z && pitch && roll && heading; }
	void resetValues() {
		isReserved = false;
		if (x) XPLMSetDataf(x, 0);
		if (y) XPLMSetDataf(y, 0);
		if (z) XPLMSetDataf(z, 0);
		if (pitch)	XPLMSetDataf(pitch, 0);
		if (roll)	XPLMSetDataf(roll, 0);
		if (heading)	XPLMSetDataf(heading, 0);
	}
};
static std::vector<MultiplayerDatarefs_t> gMultiRefs;
bool isValidTcasIndex(int i)
{
	return i >= 0 && i < static_cast<int>(gMultiRefs.size());
}

int DebugRenderPhase(XPLMDrawingPhase, int isBefore, void *refcon)
{
    const std::string prefix = isBefore ? "before" : "after";
	const std::string phase = static_cast<const char *>(refcon);
	const std::string message = "DEBUG: " + prefix + " " + phase + "\n";
	XPLMDebugString(message.c_str());
	return 1;
}

#define REGISTER_PHASE_DEBUG(phase) \
XPLMRegisterDrawCallback(DebugRenderPhase, xplm_Phase_##phase, 1, static_cast<void *>(const_cast<char *>(#phase))); \
XPLMRegisterDrawCallback(DebugRenderPhase, xplm_Phase_##phase, 0, static_cast<void *>(const_cast<char *>(#phase)));

/**
static void init_debugRenderPhases()
{
	REGISTER_PHASE_DEBUG(FirstScene)
	REGISTER_PHASE_DEBUG(Terrain)
	REGISTER_PHASE_DEBUG(Airports)
	REGISTER_PHASE_DEBUG(Vectors)
	REGISTER_PHASE_DEBUG(Objects)
	REGISTER_PHASE_DEBUG(Airplanes)
	REGISTER_PHASE_DEBUG(LastScene)
	REGISTER_PHASE_DEBUG(FirstCockpit)
	REGISTER_PHASE_DEBUG(Panel)
	REGISTER_PHASE_DEBUG(Gauges)
	REGISTER_PHASE_DEBUG(Window)
	REGISTER_PHASE_DEBUG(LastCockpit)
	REGISTER_PHASE_DEBUG(LocalMap3D)
	REGISTER_PHASE_DEBUG(LocalMap2D)
	REGISTER_PHASE_DEBUG(LocalMapProfile)
}
**/

static bool gDrawLabels = true;

struct cull_info_t {					// This struct has everything we need to cull fast!
	float	model_view[16];				// The model view matrix, to get from local OpenGL to eye coordinates.
	float	proj[16];					// Proj matrix - this is just a hack to use for gluProject.
	float	nea_clip[4];				// Four clip planes in the form of Ax + By + Cz + D = 0 (ABCD are in the array.)
	float	far_clip[4];				// They are oriented so the positive side of the clip plane is INSIDE the view volume.
	float	lft_clip[4];
	float	rgt_clip[4];
	float	bot_clip[4];
	float	top_clip[4];
};

static bool				gCullInfoInitialised = false;
static XPLMDataRef		projectionMatrixRef = nullptr;
static XPLMDataRef		modelviewMatrixRef = nullptr;
static XPLMDataRef		viewportRef = nullptr;

static bool				gMSAAHackInitialised = false;
static XPLMDataRef		gMSAAXRatioRef = nullptr;
static XPLMDataRef		gMSAAYRatioRef = nullptr;
static XPLMDataRef		gHDROnRef = nullptr;

static XPLMDataRef		gOwnPlaneX = nullptr;
static XPLMDataRef		gOwnPlaneY = nullptr;
static XPLMDataRef		gOwnPlaneZ = nullptr;

static void init_cullinfo()
{
	modelviewMatrixRef = XPLMFindDataRef("sim/graphics/view/modelview_matrix");
	projectionMatrixRef = XPLMFindDataRef("sim/graphics/view/projection_matrix");
	viewportRef = XPLMFindDataRef("sim/graphics/view/viewport");
	gCullInfoInitialised = true;
}

static void setup_cull_info(cull_info_t * i)
{
	if (!gCullInfoInitialised) {
		init_cullinfo();
	}
	// First, just read out the current OpenGL matrices...do this once at setup
	// because it's not the fastest thing to do.
	//
	// if our X-Plane version supports it, pull it from the daatrefs to avoid a
	// potential driver stall.
	if (!modelviewMatrixRef || !projectionMatrixRef) {
		glGetFloatv(GL_MODELVIEW_MATRIX, i->model_view);
		glGetFloatv(GL_PROJECTION_MATRIX, i->proj);
	} else {
		XPLMGetDatavf(modelviewMatrixRef, i->model_view, 0, 16);
		XPLMGetDatavf(projectionMatrixRef, i->proj, 0, 16);
	}

	// Now...what the heck is this?  Here's the deal: the clip planes have values in "clip" coordinates of: Left = (1,0,0,1)
	// Right = (-1,0,0,1), Bottom = (0,1,0,1), etc.  (Clip coordinates are coordinates from -1 to 1 in XYZ that the driver
	// uses.  The projection matrix converts from eye to clip coordinates.)
	//
	// How do we convert a plane backward from clip to eye coordinates?  Well, we need the transpose of the inverse of the
	// inverse of the projection matrix.  (Transpose of the inverse is needed to transform a plane, and the inverse of the
	// projection is the matrix that goes clip -> eye.)  Well, that cancels out to the transpose of the projection matrix,
	// which is nice because it means we don't need a matrix inversion in this bit of sample code.
	
	// So this nightmare down here is simply:
	// clip plane * transpose (proj_matrix)
	// worked out for all six clip planes.  If you squint you can see the patterns:
	// L:  1  0 0 1
	// R: -1  0 0 1
	// B:  0  1 0 1
	// T:  0 -1 0 1
	// etc.
	
	i->lft_clip[0] = i->proj[0]+i->proj[3];	i->lft_clip[1] = i->proj[4]+i->proj[7];	i->lft_clip[2] = i->proj[8]+i->proj[11];	i->lft_clip[3] = i->proj[12]+i->proj[15];
	i->rgt_clip[0] =-i->proj[0]+i->proj[3];	i->rgt_clip[1] =-i->proj[4]+i->proj[7];	i->rgt_clip[2] =-i->proj[8]+i->proj[11];	i->rgt_clip[3] =-i->proj[12]+i->proj[15];
	
	i->bot_clip[0] = i->proj[1]+i->proj[3];	i->bot_clip[1] = i->proj[5]+i->proj[7];	i->bot_clip[2] = i->proj[9]+i->proj[11];	i->bot_clip[3] = i->proj[13]+i->proj[15];
	i->top_clip[0] =-i->proj[1]+i->proj[3];	i->top_clip[1] =-i->proj[5]+i->proj[7];	i->top_clip[2] =-i->proj[9]+i->proj[11];	i->top_clip[3] =-i->proj[13]+i->proj[15];

	i->nea_clip[0] = i->proj[2]+i->proj[3];	i->nea_clip[1] = i->proj[6]+i->proj[7];	i->nea_clip[2] = i->proj[10]+i->proj[11];	i->nea_clip[3] = i->proj[14]+i->proj[15];
	i->far_clip[0] =-i->proj[2]+i->proj[3];	i->far_clip[1] =-i->proj[6]+i->proj[7];	i->far_clip[2] =-i->proj[10]+i->proj[11];	i->far_clip[3] =-i->proj[14]+i->proj[15];
}

static int sphere_is_visible(const cull_info_t * i, float x, float y, float z, float r)
{
	// First: we transform our coordinate into eye coordinates from model-view.
	float xp = x * i->model_view[0] + y * i->model_view[4] + z * i->model_view[ 8] + i->model_view[12];
	float yp = x * i->model_view[1] + y * i->model_view[5] + z * i->model_view[ 9] + i->model_view[13];
	float zp = x * i->model_view[2] + y * i->model_view[6] + z * i->model_view[10] + i->model_view[14];

	// Now - we apply the "plane equation" of each clip plane to see how far from the clip plane our point is.
	// The clip planes are directed: positive number distances mean we are INSIDE our viewing area by some distance;
	// negative means outside.  So ... if we are outside by less than -r, the ENTIRE sphere is out of bounds.
	// We are not visible!  We do the near clip plane, then sides, then far, in an attempt to try the planes
	// that will eliminate the most geometry first...half the world is behind the near clip plane, but not much is
	// behind the far clip plane on sunny day.
	if ((xp * i->nea_clip[0] + yp * i->nea_clip[1] + zp * i->nea_clip[2] + i->nea_clip[3] + r) < 0)	return false;
	if ((xp * i->bot_clip[0] + yp * i->bot_clip[1] + zp * i->bot_clip[2] + i->bot_clip[3] + r) < 0)	return false;
	if ((xp * i->top_clip[0] + yp * i->top_clip[1] + zp * i->top_clip[2] + i->top_clip[3] + r) < 0)	return false;
	if ((xp * i->lft_clip[0] + yp * i->lft_clip[1] + zp * i->lft_clip[2] + i->lft_clip[3] + r) < 0)	return false;
	if ((xp * i->rgt_clip[0] + yp * i->rgt_clip[1] + zp * i->rgt_clip[2] + i->rgt_clip[3] + r) < 0)	return false;
	if ((xp * i->far_clip[0] + yp * i->far_clip[1] + zp * i->far_clip[2] + i->far_clip[3] + r) < 0)	return false;
	return true;
}

static float sphere_distance_sqr(const cull_info_t * i, float x, float y, float z)
{
	float xp = x * i->model_view[0] + y * i->model_view[4] + z * i->model_view[ 8] + i->model_view[12];
	float yp = x * i->model_view[1] + y * i->model_view[5] + z * i->model_view[ 9] + i->model_view[13];
	float zp = x * i->model_view[2] + y * i->model_view[6] + z * i->model_view[10] + i->model_view[14];
	return xp*xp+yp*yp+zp*zp;
}

static void convert_to_2d(const cull_info_t * i, const int * vp, float x, float y, float z, float w, float * out_x, float * out_y)
{
	float xe = x * i->model_view[0] + y * i->model_view[4] + z * i->model_view[ 8] + w * i->model_view[12];
	float ye = x * i->model_view[1] + y * i->model_view[5] + z * i->model_view[ 9] + w * i->model_view[13];
	float ze = x * i->model_view[2] + y * i->model_view[6] + z * i->model_view[10] + w * i->model_view[14];
	float we = x * i->model_view[3] + y * i->model_view[7] + z * i->model_view[11] + w * i->model_view[15];

	float xc = xe * i->proj[0] + ye * i->proj[4] + ze * i->proj[ 8] + we * i->proj[12];
	float yc = xe * i->proj[1] + ye * i->proj[5] + ze * i->proj[ 9] + we * i->proj[13];
	//	float zc = xe * i->proj[2] + ye * i->proj[6] + ze * i->proj[10] + we * i->proj[14];
	float wc = xe * i->proj[3] + ye * i->proj[7] + ze * i->proj[11] + we * i->proj[15];
	
	xc /= wc;
	yc /= wc;
	//	zc /= wc;

	*out_x = static_cast<float>(vp[0]) + (1.0f + xc) * static_cast<float>(vp[2]) / 2.0f;
	*out_y = static_cast<float>(vp[1]) + (1.0f + yc) * static_cast<float>(vp[3]) / 2.0f;
}


#if RENDERER_STATS

static int		GetRendererStat(void * inRefcon)
{
	return *((int *) inRefcon);
}

#endif

static	int		gTotPlanes = 0;			// Counters
static	int		gACFPlanes = 0;			// Number of Austin's planes we drew in full
static	int		gNavPlanes = 0;			// Number of Austin's planes we drew with lights only
static	int		gOBJPlanes = 0;			// Number of our OBJ planes we drew in full

static	XPLMDataRef		gVisDataRef  = nullptr;		// Current air visiblity for culling.
static	XPLMDataRef		gAltitudeRef = nullptr;;	// Current aircraft altitude (for TCAS)
static	XPLMProbeRef	terrainProbe = nullptr;;	// Probe to probe where the ground is for clamping


void			XPMPInitDefaultPlaneRenderer(void)
{
	XPLMDestroyProbe(terrainProbe);
	terrainProbe = XPLMCreateProbe(xplm_ProbeY);
	
	// SETUP - mostly just fetch datarefs.

	if (!gCullInfoInitialised) {
		init_cullinfo();
	}
	gVisDataRef = XPLMFindDataRef("sim/graphics/view/visibility_effective_m");
	if (!gVisDataRef) gVisDataRef = XPLMFindDataRef("sim/weather/visibility_effective_m");
	if (!gVisDataRef)
		XPLMDebugString("WARNING: Default renderer could not find effective visibility in the sim.\n");

	if(!gAltitudeRef) gAltitudeRef = XPLMFindDataRef("sim/flightmodel/position/elevation");

#if RENDERER_STATS
	XPLMRegisterDataAccessor("hack/renderer/planes", xplmType_Int, 0, GetRendererStat, NULL,
							 NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
							 &gTotPlanes, NULL);
	XPLMRegisterDataAccessor("hack/renderer/navlites", xplmType_Int, 0, GetRendererStat, NULL,
							 NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
							 &gNavPlanes, NULL);
	XPLMRegisterDataAccessor("hack/renderer/objects", xplmType_Int, 0, GetRendererStat, NULL,
							 NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
							 &gOBJPlanes, NULL);
	XPLMRegisterDataAccessor("hack/renderer/acfs", xplmType_Int, 0, GetRendererStat, NULL,
							 NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
							 &gACFPlanes, NULL);
#endif

	// init_debugRenderPhases(); // log all render phases if needed

	// We don't know how many multiplayer planes there are - fetch as many as we can.

	int 		n = 1;
	char		buf[100];
	MultiplayerDatarefs_t	d;
	while (1)
	{
		sprintf(buf,"sim/multiplayer/position/plane%d_x", n);
		d.x = XPLMFindDataRef(buf);
		sprintf(buf,"sim/multiplayer/position/plane%d_y", n);
		d.y = XPLMFindDataRef(buf);
		sprintf(buf,"sim/multiplayer/position/plane%d_z", n);
		d.z = XPLMFindDataRef(buf);
		sprintf(buf,"sim/multiplayer/position/plane%d_the", n);
		d.pitch = XPLMFindDataRef(buf);
		sprintf(buf,"sim/multiplayer/position/plane%d_phi", n);
		d.roll = XPLMFindDataRef(buf);
		sprintf(buf,"sim/multiplayer/position/plane%d_psi", n);
		d.heading = XPLMFindDataRef(buf);
		if (!d.ok()) break;
		gMultiRefs.push_back(d);
		++n;
	}
}

void XPMPDeinitDefaultPlaneRenderer() {
	XPLMDestroyProbe(terrainProbe);
	terrainProbe = nullptr;
}

double getCorrectYValue(double inX, double inY, double inZ, double inModelYOffset, bool inIsClampingOn) {
	if (!inIsClampingOn) {
		return inY;
	}
	XPLMProbeInfo_t info;
	info.structSize = sizeof(XPLMProbeInfo_t);
	XPLMProbeResult res = XPLMProbeTerrainXYZ(terrainProbe, static_cast<float>(inX), static_cast<float>(inY), static_cast<float>(inZ), &info);
	if (res != xplm_ProbeHitTerrain) {
		return inY;
	}
	double minY = info.locationY + inModelYOffset;
	return (inY < minY) ? minY : inY;
}

// PlaneToRender struct: we prioritize planes radially by distance, so...
// we use this struct to remember one visible plane.  Once we've
// found all visible planes, we draw the closest ones.

struct	PlaneToRender_t {
	float					x;			// Positional info
	float					y;
	float					z;
	XPMPPlanePtr			plane;
	bool					full;		// Do we need to draw the full plane or just lites?
	bool					cull;		// Are we visible on screen?
	bool					tcas;		// Are we visible on TCAS?
	XPLMPlaneDrawState_t	state;		// Flaps, gear, etc.
	float					dist;
};
typedef	std::map<double, PlaneToRender_t>	DistanceMap;

// We calculate the screen coordinates during 3D rendering
// and actually draw the labels during 2D rendering,
// so we need to store the coordinates somewhere:
struct LabelToRender_t {
	float x;
	float y;
	const char *text;
};
static std::vector<LabelToRender_t> gLabels;


void			XPMPDefaultPlaneRenderer(int is_blend)
{
	long	planeCount = XPMPCountPlanes();
#if DEBUG_RENDERER
	char	buf[50];
	sprintf(buf,"Renderer Planes: %d\n", planeCount);
	XPLMDebugString(buf);
#endif
	if (planeCount == 0)		// Quick exit if no one's around.
	{
		for (auto &ref : gMultiRefs) { ref.resetValues(); }
		gLabels.clear();
		gEnableCount = 1;
		if (gDumpOneRenderCycle)
		{
			gDumpOneRenderCycle = false;
			XPLMDebugString("No planes this cycle.\n");
		}
		return;
	}

	if (!gMSAAHackInitialised) {
		gMSAAHackInitialised = true;
		gMSAAXRatioRef = XPLMFindDataRef("sim/private/controls/hdr/fsaa_ratio_x");
		gMSAAYRatioRef = XPLMFindDataRef("sim/private/controls/hdr/fsaa_ratio_y");
		gHDROnRef      = XPLMFindDataRef("sim/graphics/settings/HDR_on");
	}

	if (!gOwnPlaneX) {
		gOwnPlaneX = XPLMFindDataRef("sim/flightmodel/position/local_x");
		gOwnPlaneY = XPLMFindDataRef("sim/flightmodel/position/local_y");
		gOwnPlaneZ = XPLMFindDataRef("sim/flightmodel/position/local_z");
	}

	cull_info_t			gl_camera;
	setup_cull_info(&gl_camera);
	XPLMCameraPosition_t x_camera;

	XPLMReadCameraPosition(&x_camera);	// only for zoom!

	// Culling - read the camera pos«and figure out what's visible.

	const double	maxDist = XPLMGetDataf(gVisDataRef);
	const double	labelDist = min(maxDist, MAX_LABEL_DIST) * x_camera.zoom;		// Labels get easier to see when users zooms.
	const double	fullPlaneDist = x_camera.zoom * (5280.0 / 3.2) * (gFloatPrefsFunc ? gFloatPrefsFunc("planes","full_distance", 3.0) : 3.0);	// Only draw planes fully within 3 miles.
	const int		maxFullPlanes = gIntPrefsFunc ? gIntPrefsFunc("planes","max_full_count", 100) : 100;						// Draw no more than 100 full planes!

	gTotPlanes = planeCount;
	gNavPlanes = gACFPlanes = gOBJPlanes = 0;

	int modelCount, active, plugin;
	XPLMCountAircraft(&modelCount, &active, &plugin);

	DistanceMap						myPlanes;		// Planes - sorted by camera distance so we can do the closest N and bail
	DistanceMap						tcasPlanes;		// Planes - sorted by aircraft distance

	/************************************************************************************
	 * CULLING AND STATE CALCULATION LOOP
	 ************************************************************************************/

	if (gDumpOneRenderCycle)
	{
		XPLMDebugString("Dumping one cycle map of planes.\n");
		char	fname[256], bigbuf[1024], foo[32];
		for (int n = 1; n < modelCount; ++n)
		{
			XPLMGetNthAircraftModel(n, fname, bigbuf);
			sprintf(foo, " [%d] - ", n);
			XPLMDebugString(foo);
			XPLMDebugString(fname);
			XPLMDebugString(" - ");
			XPLMDebugString(bigbuf);
			XPLMDebugString("\n");
		}
	}

	// Altitude for TCAS
	const double acft_alt = XPLMGetDatad(gAltitudeRef) / kFtToMeters;

#if DEBUG_TCAS
	{
		std::string debug = "TCAS max dist " + std::to_string(kMaxDistTCAS) + " aircraft alt " + std::to_string(acft_alt) + " max " + std::to_string(MAX_TCAS_ALTDIFF) + "\n";
		XPLMDebugString(debug.c_str());
	}
#endif

	// Go through every plane.  We're going to figure out if it is visible and if so remember it for drawing later.
	for (long index = 0; index < planeCount; ++index)
	{
		XPMPPlaneID id = XPMPGetNthPlane(index);
		XPMPPlanePosition_t	pos;
		pos.size = sizeof(pos);
		pos.label[0] = 0;

		if (XPMPGetPlaneData(id, xpmpDataType_Position, &pos) != xpmpData_Unavailable)
		{
			// First figure out where the plane is!

			double	x,y,z; // other plane x y z
			const double ownX = XPLMGetDatad(gOwnPlaneX);
			const double ownY = XPLMGetDatad(gOwnPlaneY);
			const double ownZ = XPLMGetDatad(gOwnPlaneZ);

			XPLMWorldToLocal(pos.lat, pos.lon, pos.elevation * kFtToMeters, &x, &y, &z);

			const double deltaOwnX = x-ownX;
			const double deltaOwnY = y-ownY;
			const double deltaOwnZ = z-ownZ;

			const float cameraDistMeters = sqrt(sphere_distance_sqr(&gl_camera,
												static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)));
			const double ownAircraftDistMeters = sqrt(deltaOwnX*deltaOwnX + deltaOwnY*deltaOwnY + deltaOwnZ*deltaOwnZ);

			// If the plane is farther than our TCAS range, it has no TCAS index
			bool tcas = true;
			if (ownAircraftDistMeters > kMaxDistTCAS) { tcas = false; static_cast<XPMPPlanePtr>(id)->tcasIndex = -1; }
			if (!tcas && cameraDistMeters > kMaxDistTCAS) { continue; } // aircraft are not shown outside TCAS distance

			// Only draw if it's in range.
			bool cull = (cameraDistMeters > maxDist);
			
			if (tcas) {
				XPMPPlaneRadar_t radar;
				radar.size = sizeof(radar);
				if (XPMPGetPlaneData(id, xpmpDataType_Radar, &radar) != xpmpData_Unavailable)
					if (radar.mode == xpmpTransponderMode_Standby) tcas = false;
			}

			// check for altitude - if difference exceeds a preconfigured limit, don't show
			if (tcas) {
				double alt_diff = pos.elevation - acft_alt;
				if(alt_diff < 0) alt_diff *= -1;
				if(alt_diff > MAX_TCAS_ALTDIFF) tcas = false;
			}

			// Create the render record for TCAS with distance to own plane 
			PlaneToRender_t renderRecord;
			renderRecord.x = static_cast<float>(x);
			renderRecord.y = static_cast<float>(y);
			renderRecord.z = static_cast<float>(z);
			renderRecord.plane = static_cast<XPMPPlanePtr>(id);
			renderRecord.cull = false;
			renderRecord.tcas = tcas; // tcas
			if (!renderRecord.tcas) { renderRecord.plane->tcasIndex = -1; }
            if (tcas) tcasPlanes.emplace(ownAircraftDistMeters, renderRecord);
#if DEBUG_TCAS
			if (tcas) {
				char icao[128], livery[128], debug[512];
				XPMPGetPlaneICAOAndLivery(id, icao, livery);
				sprintf(debug,"TCAS plane %ld (%s/%s) at lle %f, %f, %f (xyz=%f, %f, %f) distance=%f\n", index, icao, livery,
						pos.lat, pos.lon, pos.elevation, x, y, z, ownAircraftDistMeters);
				XPLMDebugString(debug);
			}
#endif
			// TCAS done here, do we need to continue
			if (cameraDistMeters > kMaxDistTCAS) { continue; } // aircraft are not shown outside TCAS distance

			// Calculate the heading from the camera to the target (hor, vert).
			// Calculate the angles between the camera angles and the real angles.
			// Cull if we exceed half the FOV.
			if(!cull && !sphere_is_visible(&gl_camera, static_cast<float>(x),
											static_cast<float>(y),
											static_cast<float>(z), 50.0))
			{
				cull = true;
			}

			// Full plane or lites based on distance.
			const bool	drawFullPlane = (cameraDistMeters < fullPlaneDist);

#if DEBUG_RENDERER
			char	icao[128], livery[128];
			char	debug[512];

			XPMPGetPlaneICAOAndLivery(id, icao, livery);
			sprintf(debug,"Queueing plane %d (%s/%s) at lle %f, %f, %f (xyz=%f, %f, %f) pitch=%f,roll=%f,heading=%f,model=1.\n", index, icao, livery,
					pos.lat, pos.lon, pos.elevation,
					x, y, z, pos.pitch, pos.roll, pos.heading);
			XPLMDebugString(debug);
#endif

			// Stash one render record with the plane's position, etc.
			{
				renderRecord.cull = cull;	// NO other planes.  Doing so causes a lot of things to go nuts!

				XPMPPlaneSurfaces_t	surfaces;
				surfaces.size = sizeof(surfaces);
				if (XPMPGetPlaneData(id, xpmpDataType_Surfaces, &surfaces) != xpmpData_Unavailable)
				{
					renderRecord.state.structSize = sizeof(renderRecord.state);
					renderRecord.state.gearPosition 	= surfaces.gearPosition 	;
					renderRecord.state.flapRatio 		= surfaces.flapRatio 		;
					renderRecord.state.spoilerRatio 	= surfaces.spoilerRatio 	;
					renderRecord.state.speedBrakeRatio 	= surfaces.speedBrakeRatio 	;
					renderRecord.state.slatRatio 		= surfaces.slatRatio 		;
					renderRecord.state.wingSweep 		= surfaces.wingSweep 		;
					renderRecord.state.thrust 			= surfaces.thrust 			;
					renderRecord.state.yokePitch 		= surfaces.yokePitch 		;
					renderRecord.state.yokeHeading 		= surfaces.yokeHeading 		;
					renderRecord.state.yokeRoll 		= surfaces.yokeRoll 		;
				} else {
					renderRecord.state.structSize = sizeof(renderRecord.state);
					renderRecord.state.gearPosition = (pos.elevation < 70) ?  1.0f : 0.0f;
					renderRecord.state.flapRatio = (pos.elevation < 70) ? 1.0f : 0.0f;
					renderRecord.state.spoilerRatio = renderRecord.state.speedBrakeRatio = renderRecord.state.slatRatio = renderRecord.state.wingSweep = 0.0;
					renderRecord.state.thrust = (pos.pitch > 30) ? 1.0f : 0.6f;
					renderRecord.state.yokePitch = pos.pitch / 90.0f;
					renderRecord.state.yokeHeading = pos.heading / 180.0f;
					renderRecord.state.yokeRoll = pos.roll / 90.0f;

					// use some smart defaults
					renderRecord.plane->surface.lights.bcnLights = 1;
					renderRecord.plane->surface.lights.navLights = 1;
				}
				if (renderRecord.plane->model && !renderRecord.plane->model->moving_gear)
					renderRecord.plane->surface.gearPosition = 1.0;
				renderRecord.full = drawFullPlane;
				renderRecord.dist = cameraDistMeters;
				myPlanes.emplace(cameraDistMeters, renderRecord);

			} // State calculation
			
		} // Plane has data available
		
	} // Per-plane loop

	if (gDumpOneRenderCycle)
		XPLMDebugString("End of cycle dump.\n");

	/************************************************************************************
	 * Prepare multiplayer indexes for TCAS
	 ************************************************************************************/

	if (gHasControlOfAIAircraft)
	{
		std::size_t blips = 0;

		// to avoid gaps in the indexes we create a vector of all indexes used and new ones
		std::bitset<32> originalTCASIndexes; // init to 0, we never expect more than 19+1 values
		std::size_t maxOriginalTcasIndex = 0;
		for (auto &pair : tcasPlanes)
		{
			if (blips >= gMultiRefs.size()) break;
			if (pair.second.tcas)
			{
				++blips; // should be possible to create one as we checked range already
				int index = pair.second.plane->tcasIndex;
				if (!isValidTcasIndex(index)) {
					// if it is a new one, try to find an index
					for (index = 0; isValidTcasIndex(index) && gMultiRefs[static_cast<size_t>(index)].isReserved; ++index)
					{}
				}
				// already used one or valid new one?
				if (!isValidTcasIndex(index)) { continue; }
				pair.second.plane->tcasIndex = index;
				const std::size_t i = static_cast<std::size_t>(index); // avoid compile issues with CLANG
				originalTCASIndexes.set(i, true);
				if (i > maxOriginalTcasIndex) { maxOriginalTcasIndex = i; }
			}
		}

		if (blips > originalTCASIndexes.size()) { blips = 0; } // paranoia, avoid crash if bitset size is too small
		const bool hasTcasIndexGaps = (maxOriginalTcasIndex >= blips && blips > 0);
		static std::vector<std::size_t> originalTCASIndexGaps; // local static to amortize the cost of allocation
		originalTCASIndexGaps.clear();
		if (hasTcasIndexGaps) {
			// find the gaps, those are the ones which are not in the blips range
			for (std::size_t index = 0; index < blips; ++index) {
				if (!originalTCASIndexes.test(index)) {
					originalTCASIndexGaps.push_back(index);
				}
			}
		}

		// reset slot reservations
		for (auto &ref : gMultiRefs) { ref.isReserved = false; }

		// re-reserve slots already used by planes in range, from the previous frame
#if DEBUG_TCAS
		{
			const std::string debug = "TCAS blips " + std::to_string(blips) + " planes " + std::to_string(tcasPlanes.size()) + "\n";
			XPLMDebugString(debug.c_str());
		}
#endif
		for (auto &pair : tcasPlanes)
		{
			if (pair.second.tcas)
			{
				int index = pair.second.plane->tcasIndex;
				if (isValidTcasIndex(index)) {
					std::size_t i = static_cast<std::size_t>(index); // avoid compile issues with CLANG
					if (i >= blips) {
						if (!originalTCASIndexGaps.empty()) {
							// we now close the gaps by filling them with values from the end
							i = originalTCASIndexGaps.front(); // first free gap
							originalTCASIndexGaps.erase(originalTCASIndexGaps.begin());
							pair.second.plane->tcasIndex = static_cast<int>(i); // now in the free gap
						}
					}
					gMultiRefs[i].isReserved = true;
#if DEBUG_TCAS
					{
						const std::string debug = "Reserving index " + std::to_string(i) + " in gMultiRefs\n";
						XPLMDebugString(debug.c_str());
					}
#endif
				}
			} // TCAS
		} // for planes
	}	// gHasControlOfAIAircraft

	/************************************************************************************
	 * ACTUAL RENDERING LOOP
	 ************************************************************************************/

	// We're going to go in and render the first N planes in full, and the rest as lites.
	// We're also going to put the x-plane multiplayer vars in place for the first N
	// TCAS-visible planes, so they show up on our moving map.
	// We do this in two stages: building up what to do, then doing it in the optimal
	// OGL order.

	size_t	renderedCounter = 0;
	int		lastMultiRefUsed = -1;

	vector<PlaneToRender_t *>			planes_obj_lites;
	multimap<int, PlaneToRender_t *>	planes_austin;
	vector<PlaneToRender_t *>			planes_obj;
	vector<PlaneToRender_t *>			planes_obj8;

	vector<PlaneToRender_t *>::iterator			planeIter;
	multimap<int, PlaneToRender_t *>::iterator	planeMapIter;

	// In our first iteration pass we'll go through all planes and handle TCAS, draw planes that have no
	// CSL model, and put CSL planes in the right 'bucket'.

	for (DistanceMap::iterator iter = myPlanes.begin(); iter != myPlanes.end(); ++iter)
	{
		// This is the case where we draw a real plane.
		if (!iter->second.cull)
		{
			// Max plane enforcement - once we run out of the max number of full planes the
			// user allows, force only lites for framerate
			if (gACFPlanes >= maxFullPlanes)
				iter->second.full = false;

#if DEBUG_RENDERER
			char	debug[512];
			sprintf(debug,"Drawing plane: %s at %f,%f,%f (%fx%fx%f full=%d\n",
					iter->second.plane->model ? iter->second.plane->model->file_path.c_str() : "<none>", iter->second.x, iter->second.y, iter->second.z,
					iter->second.plane->pos.pitch, iter->second.plane->pos.roll, iter->second.plane->pos.heading, iter->second.full ? 1 : 0);
			XPLMDebugString(debug);
#endif

			if (iter->second.plane->model)
			{
				// Fixme:
				// find or update the actual vert offset in the csl model data
				// cslVertOffsetCalc.findOrUpdateActualVertOffset(*iter->second.plane->model);
				// correct y value by real terrain elevation
				// const bool isClampingOn = gIntPrefsFunc("PREFERENCES", "CLAMPING", 0);
				// iter->second.y = static_cast<float>(getCorrectYValue(iter->second.x, iter->second.y, iter->second.z, iter->second.plane->model->actualVertOffset, isClampingOn));
				if (iter->second.plane->model->plane_type == plane_Austin)
				{
					planes_austin.insert(multimap<int, PlaneToRender_t *>::value_type(CSL_GetOGLIndex(iter->second.plane->model), &iter->second));
				}
				else if (iter->second.plane->model->plane_type == plane_Obj)
				{
					planes_obj.push_back(&iter->second);
					planes_obj_lites.push_back(&iter->second);
				}
				else if(iter->second.plane->model->plane_type == plane_Obj8)
				{
					planes_obj8.push_back(&iter->second);
				}

			} else {
				// If it's time to draw austin's planes but this one
				// doesn't have a model, we draw anything.
				glMatrixMode(GL_MODELVIEW);
				glPushMatrix();
				glTranslatef(iter->second.x, iter->second.y, iter->second.z);
				glRotatef(iter->second.plane->pos.heading, 0.0, -1.0, 0.0);
				glRotatef(iter->second.plane->pos.pitch, -1.0, 0.0, 0.0);
				glRotatef(iter->second.plane->pos.roll, 0.0, 0.0, -1.0);

				// Safety check - if plane 1 isn't even loaded do NOT draw, do NOT draw plane 0.
				// Using the user's planes can cause the internal flight model to get f-cked up.
				// Using a non-loaded plane can trigger internal asserts in x-plane.
				if (modelCount > 1)
					if(!is_blend)
						XPLMDrawAircraft(1,
											static_cast<float>(iter->second.x), static_cast<float>(iter->second.y), static_cast<float>(iter->second.z),
											iter->second.plane->pos.pitch, iter->second.plane->pos.roll, iter->second.plane->pos.heading,
											iter->second.full ? 1 : 0, &iter->second.state);

				glPopMatrix();
			}

		}

		// TCAS handling - if the plane needs to be drawn on TCAS and we haven't yet, move one of Austin's planes.
		if (iter->second.tcas && renderedCounter < gMultiRefs.size() && gHasControlOfAIAircraft)
		{
			int index = iter->second.plane->tcasIndex;
			if (isValidTcasIndex(index))
			{
				const std::size_t i = static_cast<std::size_t>(index);
				XPLMSetDataf(gMultiRefs[i].x, iter->second.x);
				XPLMSetDataf(gMultiRefs[i].y, iter->second.y);
				XPLMSetDataf(gMultiRefs[i].z, iter->second.z);
				XPLMSetDataf(gMultiRefs[i].pitch, iter->second.plane->pos.pitch);
				XPLMSetDataf(gMultiRefs[i].roll, iter->second.plane->pos.roll);
				XPLMSetDataf(gMultiRefs[i].heading, iter->second.plane->pos.heading);
				gMultiRefs[i].isReserved = true;
				iter->second.plane->tcasIndex = index;
				if (index > lastMultiRefUsed)
					lastMultiRefUsed = index;
				++renderedCounter;
			}
		}
	}

	// PASS 1 - draw Austin's planes.
	if(gHasControlOfAIAircraft && !is_blend)
		for (planeMapIter = planes_austin.begin(); planeMapIter != planes_austin.end(); ++planeMapIter)
		{
			CSL_DrawObject(	planeMapIter->second->plane,
							planeMapIter->second->dist,
							planeMapIter->second->x,
							planeMapIter->second->y,
							planeMapIter->second->z,
							planeMapIter->second->plane->pos.pitch,
							planeMapIter->second->plane->pos.roll,
							planeMapIter->second->plane->pos.heading,
							plane_Austin,
							planeMapIter->second->full ? 1 : 0,
							planeMapIter->second->plane->surface.lights,
							&planeMapIter->second->state);

			if (planeMapIter->second->full)
				++gACFPlanes;
			else
				++gNavPlanes;
		}

	// PASS 2 - draw OBJs
	// Blend for solid OBJ7s?  YES!  First, in HDR mode, they DO NOT draw to the gbuffer properly -
	// they splat their livery into the normal map, which is terrifying and stupid.  Then they are also
	// pre-lit...the net result is surprisingly not much worse than regular rendering considering how many
	// bad things have happened, but for all I know we're getting NaNs somewhere.
	//
	// Blending isn't going to hurt things in NON-HDR because our rendering is so stupid for old objs - there's
	// pretty much never translucency so we aren't going to get Z-order fails.  So f--- it...always draw blend.<
	if(is_blend)
		for (const auto &plane_obj : planes_obj)
		{
			CSL_DrawObject(
						plane_obj->plane,
						plane_obj->dist,
						plane_obj->x,
						plane_obj->y,
						plane_obj->z,
						plane_obj->plane->pos.pitch,
						plane_obj->plane->pos.roll,
						plane_obj->plane->pos.heading,
						plane_Obj,
						plane_obj->full ? 1 : 0,
						plane_obj->plane->surface.lights,
						&plane_obj->state);
			++gOBJPlanes;
		}

	if (!is_blend)
	{
		for (planeIter = planes_obj8.begin(); planeIter != planes_obj8.end(); ++planeIter)
		{
			CSL_DrawObject((*planeIter)->plane,
				(*planeIter)->dist,
				(*planeIter)->x,
				(*planeIter)->y,
				(*planeIter)->z,
				(*planeIter)->plane->pos.pitch,
				(*planeIter)->plane->pos.roll,
				(*planeIter)->plane->pos.heading,
				plane_Obj8,
				(*planeIter)->full ? 1 : 0,
				(*planeIter)->plane->surface.lights,
				&(*planeIter)->state);
		}
	}

	// PASS 3 - draw OBJ lights.
	if(is_blend)
	{
		if (!planes_obj_lites.empty())
		{
			OBJ_BeginLightDrawing();
			for (planeIter = planes_obj_lites.begin(); planeIter != planes_obj_lites.end(); ++planeIter)
			{
				// this thing draws the lights of a model
				CSL_DrawObject( (*planeIter)->plane,
								(*planeIter)->dist,
								(*planeIter)->x,
								(*planeIter)->y,
								(*planeIter)->z,
								(*planeIter)->plane->pos.pitch,
								(*planeIter)->plane->pos.roll,
								(*planeIter)->plane->pos.heading,
								plane_Lights,
								(*planeIter)->full ? 1 : 0,
								(*planeIter)->plane->surface.lights,
								&(*planeIter)->state);
			}
		}
	}

	// PASS 4 - draw translucent
	if(is_blend)
	{
		for (planeIter = planes_obj8.begin(); planeIter != planes_obj8.end(); ++planeIter)
		{
			CSL_DrawObject((*planeIter)->plane,
				(*planeIter)->dist,
				(*planeIter)->x,
				(*planeIter)->y,
				(*planeIter)->z,
				(*planeIter)->plane->pos.pitch,
				(*planeIter)->plane->pos.roll,
				(*planeIter)->plane->pos.heading,
				plane_Obj8_Transparent,
				(*planeIter)->full ? 1 : 0,
				(*planeIter)->plane->surface.lights,
				&(*planeIter)->state);
		}
	}
	
	// PASS 5 - Labels
	if(is_blend)
	{
		gLabels.clear();
		if ( gDrawLabels )
		{
			float	x_scale = 1.0;
			float	y_scale = 1.0;
			if (gHDROnRef && XPLMGetDatai(gHDROnRef)) {     // SSAA hack only if HDR enabled
				if (gMSAAXRatioRef) {
					x_scale = XPLMGetDataf(gMSAAXRatioRef);
				}
				if (gMSAAYRatioRef) {
					y_scale = XPLMGetDataf(gMSAAYRatioRef);
				}
			}

			GLint	vp[4];
			if (viewportRef != nullptr) {
				// sim/graphics/view/viewport	int[4]	n	Pixels	Current OpenGL viewport in device window coordinates.Note thiat this is left, bottom, right top, NOT left, bottom, width, height!!
				int vpInt[4] = {0,0,0,0};
				XPLMGetDatavi(viewportRef, vpInt, 0, 4);
				vp[0] = vpInt[0];
				vp[1] = vpInt[1];
				vp[2] = vpInt[2] - vpInt[0];
				vp[3] = vpInt[3] - vpInt[1];
			} else {
				glGetIntegerv(GL_VIEWPORT, vp);
			}

			glMatrixMode(GL_PROJECTION);
			glPushMatrix();
			glLoadIdentity();
			glOrtho(0, vp[2], 0, vp[3], -1, 1);

			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glLoadIdentity();
			if (x_scale > 1.0f || y_scale > 1.0f) {
				glScalef(x_scale, y_scale, 1.0);
			} else {
				x_scale = 1.0;
				y_scale = 1.0;
			}

			for (DistanceMap::iterator iter = myPlanes.begin(); iter != myPlanes.end(); ++iter)
				if(iter->first < labelDist)
					if(!iter->second.cull)		// IMPORTANT - airplane BEHIND us still maps XY onto screen...so we get 180 degree reflections.  But behind us acf are culled, so that's good.
					{
						gLabels.push_back({});
						convert_to_2d(&gl_camera, vp, iter->second.x, iter->second.y, iter->second.z, 1.0, &gLabels.back().x, &gLabels.back().y);
						gLabels.back().x /= x_scale;
						gLabels.back().y /= y_scale;
						gLabels.back().text = iter->second.plane->pos.label;
					}

			glMatrixMode(GL_PROJECTION);
			glPopMatrix();
			glMatrixMode(GL_MODELVIEW);
			glPopMatrix();

		}
	}

	// Final hack - leave a note to ourselves for how many of Austin's planes we relocated to do TCAS.
	gEnableCount = (lastMultiRefUsed + 2); // +1 for counter, +1 for own aircraft
	// cleanup unused multiplayer datarefs
	if (gHasControlOfAIAircraft) {
		for (auto &ref : gMultiRefs) {
			if (!ref.isReserved) { ref.resetValues(); }
		}
	}

	gDumpOneRenderCycle = 0;

	// finally, cleanup textures.
	OBJ_MaintainTextures();
}

void XPMPDefaultLabelRenderer()
{
	if (gDrawLabels)
	{
		XPLMSetGraphicsState(0, 0, 0, 0, 1, 1, 0);
		float color[4] = { 1, 1, 0, 1 };

		for (const auto &label : gLabels)
		{
			XPLMDrawString(color, static_cast<int>(label.x), static_cast<int>(label.y) + 10, const_cast<char *>(label.text), nullptr, xplmFont_Basic);
		}
	}
}

void XPMPEnableAircraftLabels()
{
	gDrawLabels = true;
}

void XPMPDisableAircraftLabels()
{
	gDrawLabels = false;
}

bool XPMPDrawingAircraftLabels()
{
	return gDrawLabels;
}

