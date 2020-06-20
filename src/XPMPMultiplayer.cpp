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

#include "XPMPMultiplayer.h"
#include "XPMPMultiplayerVars.h"
#include "XPMPPlaneRenderer.h"
#include "XPMPMultiplayerCSL.h"
#include "XPLMUtilities.h"

#include <algorithm>
#include <cctype>
#include <vector>
#include <string>
#include <cstring>
#include <sstream>
#include <memory>

#include "XPLMProcessing.h"
#include "XPLMPlanes.h"
#include "XPLMDataAccess.h"
#include "XPLMDisplay.h"
#include "XPLMPlugin.h"
#include "XPLMUtilities.h"

#include "XOGLUtils.h"
#include "XUtils.h"
//#include "PlatformUtils.h"

#include <stdlib.h>
#include <stdio.h>
#include <set>
#include <cassert>
#include <chrono>
#include <sstream>
#include <iomanip>

// This prints debug info on our process of loading Austin's planes.
#define	DEBUG_MANUAL_LOADING	0


/******************************************************************************

	T H E   T C A S   H A C K
	
 The 1.0 SDK provides no way to add TCAS blips to a panel - we simply don't know
 where on the panel to draw.  The only way to get said blips is to manipulate
 Austin's "9 planes" plane objects, which he refers to when drawing the moving
 map.
 
 But how do we integrate this with our system, which relies on us doing
 the drawing (either by calling Austin's low level drawing routine or just
 doing it ourselves with OpenGL)?
 
 The answer is the TCAS hack.  Basically we set Austin's number of multiplayer
 planes to zero while 3-d drawing is happening so he doesn't draw.  Then during
 2-d drawing we pop this number back up to the number of planes that are
 visible on TCAS and set the datarefs to move them, so that they appear on TCAS.

 One note: since all TCAS blips are the same, we do no model matching for
 TCAS - we just place the first 9 planes at the right place.
 
 Our rendering loop records for us in gEnableCount how many TCAS planes should
 be visible.

 ******************************************************************************/


/* we can only manipulate the TCAS, etc, if we have control of the AI aircraft
 *
 * Non-static because this information is actually needed in the renderer, but
 * we don't want it known outside of libxplanemp and both this and the renderer's
 * headers are public!  argh!
*/
bool	gHasControlOfAIAircraft = false;

static	XPMPPlanePtr	XPMPPlaneFromID(
		XPMPPlaneID 		inID,
		XPMPPlaneVector::iterator * outIter = nullptr);

// This drawing hook is called once per frame to do the real drawing.
static	int				XPMPRenderMultiplayerPlanes(
		XPLMDrawingPhase     inPhase,
		int                  inIsBefore,
		void *               inRefcon);

// This drawing hook is called once per frame to draw the plane labels.
static	int				XPMPRenderPlaneLabels(
		XPLMDrawingPhase     inPhase,
		int                  inIsBefore,
		void *               inRefcon);

// This drawing hook is called when we want to enable Austin's planes.
static	int				XPMPEnablePlaneCount(
		XPLMDrawingPhase     inPhase,
		int                  inIsBefore,
		void *               inRefcon);

// This drawing hook is called when we want to disable Austin's planes.
static	int				XPMPDisablePlaneCount(
		XPLMDrawingPhase     inPhase,
		int                  inIsBefore,
		void *               inRefcon);

std::string XPMPTimestamp() {
	std::ostringstream ss;

	// https://en.cppreference.com/w/cpp/io/manip/put_time
	// https://stackoverflow.com/a/35157784/356726
	using namespace std::chrono;
	const auto clockNow = system_clock::now();
	const time_t now = system_clock::to_time_t(clockNow);

	// get number of milliseconds for the current second (remainder after division into seconds)
	const auto ms = duration_cast<milliseconds>(clockNow.time_since_epoch()) % 1000;
	struct tm tms;
#if defined (IBM)
	localtime_s(&tms, &now);
#else
	localtime_r(&now, &tms);
#endif
	ss	<< std::put_time(&tms, "%T")
		<< "." << std::setfill('0') << std::setw(3) << ms.count()
		<< " ";

	return ss.str();
}

#ifdef DEBUG_GL
static void xpmpKhrDebugProc(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar *message, const void *param)
{
	std::stringstream	msgOut;
	msgOut << XPMP_CLIENT_NAME << ": GL Debug: ";
	switch (type)
	{
	case GL_DEBUG_TYPE_ERROR:
		msgOut << "[ERR] ";
		break;
	case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
		msgOut << "[DEP] ";
		break;
	case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:
		msgOut << "[UND] ";
		break;
	case GL_DEBUG_TYPE_PORTABILITY:
		msgOut << "[PORT] ";
		break;
	case GL_DEBUG_TYPE_PERFORMANCE:
		msgOut << "[PERF] ";
		break;
	case GL_DEBUG_TYPE_MARKER:
		msgOut << "*** ";
		break;
	case GL_DEBUG_TYPE_PUSH_GROUP:
		msgOut << " -> ";
		break;
	case GL_DEBUG_TYPE_POP_GROUP:
		msgOut << " <- ";
		break;
	default:
		break;
	}
	msgOut << std::string(message, length) << std::endl;
	XPLMDebugString(msgOut.str().c_str());
}

void XPMPSetupGLDebug()
{
	glDebugMessageCallback(xpmpKhrDebugProc, nullptr);
	glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_FALSE);

	glDebugMessageControl(GL_DEBUG_SOURCE_API, GL_DEBUG_TYPE_ERROR, GL_DONT_CARE, 0, nullptr, GL_TRUE);
	glDebugMessageControl(GL_DEBUG_SOURCE_API, GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR, GL_DONT_CARE, 0, nullptr, GL_TRUE);

	glDebugMessageControl(GL_DEBUG_SOURCE_API, GL_DONT_CARE, GL_DEBUG_SEVERITY_HIGH, 0, nullptr, GL_TRUE);
	glDebugMessageControl(GL_DEBUG_SOURCE_API, GL_DONT_CARE, GL_DEBUG_SEVERITY_MEDIUM, 0, nullptr, GL_TRUE);
	glDebugMessageControl(GL_DEBUG_SOURCE_THIRD_PARTY, GL_DEBUG_TYPE_PUSH_GROUP, GL_DONT_CARE, 0, nullptr, GL_TRUE);
	glDebugMessageControl(GL_DEBUG_SOURCE_THIRD_PARTY, GL_DEBUG_TYPE_POP_GROUP, GL_DONT_CARE, 0, nullptr, GL_TRUE);
	glDebugMessageControl(GL_DEBUG_SOURCE_THIRD_PARTY, GL_DEBUG_TYPE_MARKER, GL_DONT_CARE, 0, nullptr, GL_TRUE);
	glDebugMessageControl(GL_DEBUG_SOURCE_THIRD_PARTY, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_TRUE);
	// eat errors.
	while (GL_NO_ERROR != glGetError())
		;
}
#endif

/********************************************************************************
 * SETUP
 ********************************************************************************/


const char * 	XPMPMultiplayerInitLegacyData(
		const char * inCSLFolder, const char * inRelatedPath,
		const char * inTexturePath, const char * inDoc8643,
		const char * inDefaultPlane,
		int (* inIntPrefsFunc)(const char *, const char *, int),
		float (* inFloatPrefsFunc)(const char *, const char *, float))
{
	gDefaultPlane = inDefaultPlane;
	gIntPrefsFunc = inIntPrefsFunc;
	gFloatPrefsFunc = inFloatPrefsFunc;

	// Set up OpenGL for our drawing callbacks
	OGL_UtilsInit();

#ifdef DEBUG_GL
	XPLMDebugString(XPMP_CLIENT_NAME ": WARNING: This build includes OpenGL Debugging\n");
	XPLMDebugString("    OpenGL Debugging induces a large overhead and produces large logfiles.\n");
	XPLMDebugString("    Please do not use this build other than as directed.\n");
#endif

	OGLDEBUG(XPLMDebugString(XPMP_CLIENT_NAME " - GL supports debugging\n"));
	OGLDEBUG(glEnable(GL_DEBUG_OUTPUT));
	OGLDEBUG(glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS));
	OGLDEBUG(XPMPSetupGLDebug());
	
	xpmp_tex_useAnisotropy = OGL_HasExtension("GL_EXT_texture_filter_anisotropic");
	if (xpmp_tex_useAnisotropy) {
		GLfloat maxAnisoLevel;

		XPLMDebugString(XPMP_CLIENT_NAME " - GL supports anisoptropic filtering.\n");

		glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &maxAnisoLevel);
		xpmp_tex_maxAnisotropy = maxAnisoLevel;
	}
	glGetIntegerv(GL_MAX_TEXTURE_SIZE, &xpmp_tex_maxSize);

	bool	problem = false;
	if (!CSL_LoadCSL(inCSLFolder, inRelatedPath, inDoc8643))
		problem = true;

	if (!CSL_Init(inTexturePath))
		problem = true;

	if (problem)		return "There were problems initializing " XPMP_CLIENT_LONGNAME ". Please examine X-Plane's Log.txt file for detailed information.";
	else 				return "";
}

const char *    XPMPMultiplayerOBJ7SupportEnable(const char * inTexturePath) {
	// Set up OpenGL for our drawing callbacks
	OGL_UtilsInit();
#ifdef DEBUG_GL
	XPLMDebugString(XPMP_CLIENT_NAME ": WARNING: This build includes OpenGL Debugging\n");
	XPLMDebugString("    OpenGL Debugging induces a large overhead and produces large logfiles.\n");
	XPLMDebugString("    Please do not use this build other than as directed.\n");
#endif

	OGLDEBUG(XPLMDebugString(XPMP_CLIENT_NAME " - GL supports debugging\n"));
	OGLDEBUG(glEnable(GL_DEBUG_OUTPUT));
	OGLDEBUG(glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS));
	OGLDEBUG(XPMPSetupGLDebug());
	
	xpmp_tex_useAnisotropy = OGL_HasExtension("GL_EXT_texture_filter_anisotropic");
	if (xpmp_tex_useAnisotropy) {
		GLfloat maxAnisoLevel;
		
		XPLMDebugString(XPMP_CLIENT_NAME " - GL supports anisoptropic filtering.\n");
		
		glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &maxAnisoLevel);
		xpmp_tex_maxAnisotropy = maxAnisoLevel;
	}
	glGetIntegerv(GL_MAX_TEXTURE_SIZE, &xpmp_tex_maxSize);

	bool problem = false;
	if (!CSL_Init(inTexturePath))
		problem = true;

	if (problem) return "There was a problem initializing " XPMP_CLIENT_LONGNAME ". Please examine X-Plane's Log.txt file for detailed information.";
	else         return "";
}

const char * 	XPMPMultiplayerInit(int (* inIntPrefsFunc)(const char *, const char *, int),
		float (* inFloatPrefsFunc)(const char *, const char *, float))
{
	gIntPrefsFunc = inIntPrefsFunc;
	gFloatPrefsFunc = inFloatPrefsFunc;
	//char	myPath[1024];
	//char	airPath[1024];
	//char	line[256];
	//char	sysPath[1024];
	//FILE *	fi;
	
	XPMPInitDefaultPlaneRenderer();

	return "";
}

void XPMPMultiplayerCleanup(void)
{
	XPMPDeinitDefaultPlaneRenderer();
	CSL_DeInit();
	OGLDEBUG(glDebugMessageCallback(nullptr, nullptr));
}


// We use this array to track Austin's planes, since we have to mess with them.
static	std::vector<std::string>	gPlanePaths;

const  char * XPMPMultiplayerEnable(void)
{
	// First build up a list of all of Austin's planes, and assign
	// their effective index numbers.
	gPlanePaths.clear();
	std::vector<char *>		ptrs;
	gPlanePaths.push_back("");
	
	for (size_t p = 0; p < gPackages.size(); ++p)
	{
		for (size_t pp = 0; pp < gPackages[p].planes.size(); ++pp)
		{
			if (gPackages[p].planes[pp].plane_type == plane_Austin)
			{
				gPackages[p].planes[pp].austin_idx = static_cast<int>(gPlanePaths.size());
				char	buf[1024];
				strcpy(buf, gPackages[p].planes[pp].file_path.c_str());
#if defined(APL)
				if (XPLMIsFeatureEnabled("XPLM_USE_NATIVE_PATHS") == 0)
				{
					Posix2HFSPath(buf, buf, 1024);
				}
#endif
				gPlanePaths.push_back(buf);
			}
		}
	}
	
	// Copy the list into something that's not permanent, but is needed by the XPLM.
	for (size_t n = 0; n < gPlanePaths.size(); ++n)
	{
#if DEBUG_MANUAL_LOADING
		char	strbuf[1024];
		sprintf(strbuf, "Plane %d = '%s'\n", static_cast<int>(n), gPlanePaths[n].c_str());
		XPLMDebugString(strbuf);
#endif	
		ptrs.push_back((char *) gPlanePaths[n].c_str());
	}
	ptrs.push_back(nullptr);
	
	
	// Attempt to grab multiplayer planes, then analyze.
	int	result = XPLMAcquirePlanes(&(*ptrs.begin()), nullptr, nullptr);
	if (result) {
		gHasControlOfAIAircraft = true;
		XPLMSetActiveAircraftCount(1);

		int	total, 		active;
		XPLMPluginID	who;

		XPLMCountAircraft(&total, &active, &who);

		if (gIntPrefsFunc("debug", "tcas_traffic", 1))
		{
			// Register the plane control calls.
			XPLMRegisterDrawCallback(XPMPDisablePlaneCount, xplm_Phase_Airplanes, 1 /* before */, nullptr);
			XPLMRegisterDrawCallback(XPMPEnablePlaneCount, xplm_Phase_Airplanes, 0 /* after */, nullptr);

			// Avoid a crash in swift.
			XPLMRegisterDrawCallback(XPMPDisablePlaneCount, xplm_Phase_Window, 0 /* after */, nullptr);
			XPLMRegisterDrawCallback(XPMPEnablePlaneCount, xplm_Phase_LastCockpit, 1 /* before */, nullptr);
		}
	} else {
		gHasControlOfAIAircraft = false;
		XPLMDebugString("WARNING: " XPMP_CLIENT_LONGNAME " did not acquire multiplayer planes!!\n");
		XPLMDebugString("    Without multiplayer plane control, we cannot fake TCAS or render ACF aircraft!\n");
		XPLMDebugString("    Make sure you remove any plugins that control multiplayer aircraft if you want these features to work\n");
	}

	// Register the actual drawing func.
	XPLMRegisterDrawCallback(XPMPRenderMultiplayerPlanes,
		xplm_Phase_Airplanes, 0, /* after*/ nullptr /* refcon */);

	// Register the label drawing func.
	XPLMRegisterDrawCallback(XPMPRenderPlaneLabels,
							 xplm_Phase_Window, 1 /* before */, nullptr /* refcon */);

	XPLMRegisterFlightLoopCallback(ThreadSynchronizer::flightLoopCallback, -1, &gThreadSynchronizer);

	if (result == 0)
	{
		return XPMP_CLIENT_LONGNAME " was not able to start up multiplayer visuals because another plugin is controlling aircraft.";
	}
	return "";
}

void XPMPMultiplayerDisable(void)
{
	if (gHasControlOfAIAircraft) {
		XPLMReleasePlanes();
		gHasControlOfAIAircraft = false;

		if (gIntPrefsFunc("debug", "tcas_traffic", 1))
		{
			XPLMUnregisterDrawCallback(XPMPDisablePlaneCount, xplm_Phase_Gauges, 0, nullptr);
			XPLMUnregisterDrawCallback(XPMPEnablePlaneCount, xplm_Phase_Gauges, 1, nullptr);
			XPLMUnregisterDrawCallback(XPMPDisablePlaneCount, xplm_Phase_Window, 0, nullptr);
			XPLMUnregisterDrawCallback(XPMPEnablePlaneCount, xplm_Phase_LastCockpit, 1, nullptr);
		}
	}

	XPLMUnregisterDrawCallback(XPMPRenderMultiplayerPlanes, xplm_Phase_Airplanes, 0, nullptr);
	XPLMUnregisterDrawCallback(XPMPRenderPlaneLabels, xplm_Phase_Window, 1, nullptr);
	XPLMUnregisterFlightLoopCallback(ThreadSynchronizer::flightLoopCallback, &gThreadSynchronizer);
}


const char * 	XPMPLoadCSLPackage(
		const char * inCSLFolder, const char * inRelatedPath, const char * inDoc8643)
{
	bool	problem = false;

	if (!CSL_LoadCSL(inCSLFolder, inRelatedPath, inDoc8643))
		problem = true;

	if (problem)		return "There were problems initializing " XPMP_CLIENT_LONGNAME ".  Please examine X-Plane's error.out file for detailed information.";
	else 				return "";
}

// This routine checks plane loading and grabs anyone we're missing.
void	XPMPLoadPlanesIfNecessary(void)
{
	int	active, models;
	XPLMPluginID	owner;
	XPLMCountAircraft(&models, &active, &owner);
	if (owner != XPLMGetMyID())
		return;

	if (models > static_cast<int>(gPlanePaths.size()))
		models = static_cast<int>(gPlanePaths.size());
	for (int n = 1; n < models; ++n)
	{
		if (!gPlanePaths[n].empty())
		{
			const char *	ourPath = gPlanePaths[n].c_str();
			char	realPath[512];
			char	fileName[256];
			XPLMGetNthAircraftModel(n, fileName, realPath);
			if (strcmp(ourPath, realPath))
			{
#if DEBUG_MANUAL_LOADING			
				XPLMDebugString("Manually Loading plane: ");
				XPLMDebugString(ourPath);
				XPLMDebugString("\n");
#endif				
				XPLMSetAircraftModel(n, ourPath);
			}
		}
	}

}

int XPMPGetNumberOfInstalledModels(void)
{
	size_t number = 0;
	for (const auto& package : gPackages)
	{
		number += package.planes.size();
	}
	return static_cast<int>(number);
}

void XPMPGetModelInfo(int inIndex, const char** outModelName, const char** outIcao, const char** outAirline, const char** outLivery)
{
	int counter = 0;
	for (const auto& package : gPackages)
	{

		if (counter + static_cast<int>(package.planes.size()) < inIndex + 1)
		{
			counter += static_cast<int>(package.planes.size());
			continue;
		}

		int positionInPackage =  inIndex - counter;
		*outModelName = package.planes[positionInPackage].getModelName().c_str();
		*outIcao = package.planes[positionInPackage].icao.c_str();
		*outAirline = package.planes[positionInPackage].airline.c_str();
		*outLivery = package.planes[positionInPackage].livery.c_str();
		break;
	}
}

/********************************************************************************
 * PLANE OBJECT SUPPORT
 ********************************************************************************/

XPMPPlaneID		XPMPCreatePlane(
		const char *			inICAOCode,
		const char *			inAirline,
		const char *			inLivery,
		XPMPPlaneData_f			inDataFunc,
		XPMPPlaneLoaded_f		inPlaneLoadedFunc,
		void *					inRefcon)
{
	auto plane = std::make_shared<XPMPPlane_t>();
	plane->icao = inICAOCode;
	plane->livery = inLivery;
	plane->airline = inAirline;
	plane->dataFunc = inDataFunc;
	plane->planeLoadedFunc = inPlaneLoadedFunc;
	plane->ref = inRefcon;
	plane->model = CSL_MatchPlane(inICAOCode, inAirline, inLivery, &plane->match_quality, true);

	if (! plane->model) { return nullptr; }

	plane->pos.size = sizeof(plane->pos);
	plane->surface.size = sizeof(plane->surface);
	plane->radar.size = sizeof(plane->radar);
	plane->posAge = plane->radarAge = plane->surfaceAge = -1;
	gPlanes.push_back(plane);
	
	XPMPPlanePtr planePtr = gPlanes.back().get();
	for (XPMPPlaneNotifierVector::iterator iter = gObservers.begin(); iter !=
		 gObservers.end(); ++iter)
	{
		iter->first.first(planePtr, xpmp_PlaneNotification_Created, iter->first.second);
	}

	if (planePtr->model->plane_type == plane_Obj)
	{
		OBJ_LoadModelAsync(plane);
	}
	else if (planePtr->model->plane_type == plane_Obj8)
	{
		OBJ_LoadObj8Async(plane);
	}

	return planePtr;
}

bool CompareCaseInsensitive(const std::string &a, const std::string &b)
{
	return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin(), [](char aa, char bb) { return toupper(aa) == toupper(bb); });
}

XPMPPlaneID     XPMPCreatePlaneWithModelName(const char *inModelName, const char *inICAOCode, const char *inAirline, const char *inLivery, const char *inNightTextureMode, XPMPPlaneData_f inDataFunc, XPMPPlaneLoaded_f inPlaneLoadedFunc, void *inRefcon)
{
	auto plane = std::make_shared<XPMPPlane_t>();
	plane->icao = inICAOCode;
	plane->livery = inLivery;
	plane->airline = inAirline;
	plane->dataFunc = inDataFunc;
	plane->planeLoadedFunc = inPlaneLoadedFunc;
	plane->ref = inRefcon;
	plane->useNightTexture = -1;

	if (inNightTextureMode && strncmp(inNightTextureMode, "d", 1) == 0) { plane->useNightTexture = 0; }
	else if (inNightTextureMode && strncmp(inNightTextureMode, "n", 1) == 0) { plane->useNightTexture = 1; }

	// Find the model
	for (auto &package : gPackages)
	{
		auto cslPlane = std::find_if(package.planes.begin(), package.planes.end(), [inModelName](auto &&p) { return CompareCaseInsensitive(p.getModelName(), inModelName); });
		if (cslPlane != package.planes.end())
		{
			plane->model = &(*cslPlane);
			break;
		}
	}

	if (!plane->model)
	{
		XPLMDebugString("Requested model ");
		XPLMDebugString(inModelName);
		XPLMDebugString(" is unknown! Falling back to own model matching.");
		if (inICAOCode) { XPLMDebugString(" Acft ");    XPLMDebugString(inICAOCode); }
		if (inAirline)  { XPLMDebugString(" Airline "); XPLMDebugString(inAirline);  }
		if (inLivery)   { XPLMDebugString(" Livery ");  XPLMDebugString(inLivery);   }
		XPLMDebugString("\n");
		return XPMPCreatePlane(inICAOCode, inAirline, inLivery, inDataFunc, inPlaneLoadedFunc, inRefcon);
	}

	plane->pos.size = sizeof(plane->pos);
	plane->surface.size = sizeof(plane->surface);
	plane->radar.size = sizeof(plane->radar);
	plane->posAge = plane->radarAge = plane->surfaceAge = -1;
	gPlanes.push_back(plane);

	XPMPPlanePtr planePtr = gPlanes.back().get();
	for (XPMPPlaneNotifierVector::iterator iter = gObservers.begin(); iter !=
		 gObservers.end(); ++iter)
	{
		iter->first.first(planePtr, xpmp_PlaneNotification_Created, iter->first.second);
	}

	if (planePtr->model->plane_type == plane_Obj)
	{
		XPLMDebugString(XPMPTimestamp().c_str());
		XPLMDebugString(XPMP_CLIENT_NAME ": Start loading OBJ7 ");
		XPLMDebugString("(");
		XPLMDebugString(inModelName);
		XPLMDebugString(")\n");

		OBJ_LoadModelAsync(plane);
	}
	else if (planePtr->model->plane_type == plane_Obj8)
	{
		XPLMDebugString(XPMPTimestamp().c_str());
		XPLMDebugString(XPMP_CLIENT_NAME ": Start loading OBJ8 ");
		XPLMDebugString("(");
		XPLMDebugString(inModelName);
		XPLMDebugString(")\n");

		OBJ_LoadObj8Async(plane);
	}
	else
	{
		XPLMDebugString(XPMPTimestamp().c_str());
		XPLMDebugString(XPMP_CLIENT_NAME ": Unknown model type ");
		XPLMDebugString("(");
		XPLMDebugString(inModelName);
		XPLMDebugString(")\n");
	}

	return planePtr;
}

void			XPMPDestroyPlane(XPMPPlaneID inID)
{
	XPMPPlaneVector::iterator iter;
	XPMPPlanePtr plane = XPMPPlaneFromID(inID, &iter);

	for (XPMPPlaneNotifierVector::iterator iter2 = gObservers.begin(); iter2 !=
		 gObservers.end(); ++iter2)
	{
		iter2->first.first(plane, xpmp_PlaneNotification_Destroyed, iter2->first.second);
	}
	gPlanes.erase(iter);
}

int	XPMPChangePlaneModel(
		XPMPPlaneID				inPlaneID,
		const char *			inICAOCode,
		const char *			inAirline,
		const char *			inLivery)
{
	XPMPPlanePtr plane = XPMPPlaneFromID(inPlaneID);
	plane->icao = inICAOCode;
	plane->airline = inAirline;
	plane->livery = inLivery;
	plane->model = CSL_MatchPlane(inICAOCode, inAirline, inLivery, &plane->match_quality, true);

	// we're changing model, we must flush the resource handles so they get reloaded.
	std::atomic_store(&plane->objHandle, OBJ7Handle{});
	std::atomic_store(&plane->texHandle, TextureHandle{});
	std::atomic_store(&plane->texLitHandle, TextureHandle{});

	for (XPMPPlaneNotifierVector::iterator iter2 = gObservers.begin(); iter2 !=
		 gObservers.end(); ++iter2)
	{
		iter2->first.first(plane, xpmp_PlaneNotification_ModelChanged, iter2->first.second);
	}

	return plane->match_quality;
}	

void	XPMPSetDefaultPlaneICAO(
		const char *			inICAO)
{
	gDefaultPlane = inICAO;
}						

long			XPMPCountPlanes(void)
{
	return static_cast<long>(gPlanes.size());
}

XPMPPlaneID		XPMPGetNthPlane(
		long 					index)
{
	if ((index < 0) || (index >= static_cast<long>(gPlanes.size())))
		return nullptr;

	return gPlanes[index].get();
}							


void XPMPGetPlaneICAOAndLivery(
		XPMPPlaneID				inPlane,
		char *					outICAOCode,	// Can be nullptr
		char *					outLivery)
{
	XPMPPlanePtr	plane = XPMPPlaneFromID(inPlane);

	if (outICAOCode)
		strcpy(outICAOCode,plane->icao.c_str());
	if (outLivery)
		strcpy(outLivery,plane->livery.c_str());
}	

void			XPMPRegisterPlaneNotifierFunc(
		XPMPPlaneNotifier_f		inFunc,
		void *					inRefcon)
{
	gObservers.push_back(XPMPPlaneNotifierTripple(XPMPPlaneNotifierPair(inFunc, inRefcon), XPLMGetMyID()));
}					

void			XPMPUnregisterPlaneNotifierFunc(
		XPMPPlaneNotifier_f		inFunc,
		void *					inRefcon)
{
	XPMPPlaneNotifierVector::iterator iter = std::find(
				gObservers.begin(), gObservers.end(), XPMPPlaneNotifierTripple(XPMPPlaneNotifierPair(inFunc, inRefcon), XPLMGetMyID()));
	if (iter != gObservers.end())
		gObservers.erase(iter);
}					

XPMPPlaneCallbackResult			XPMPGetPlaneData(
		XPMPPlaneID					inPlane,
		XPMPPlaneDataType			inDataType,
		void *						outData)
{
	XPMPPlanePtr	plane = XPMPPlaneFromID(inPlane);
	
	XPMPPlaneCallbackResult result = xpmpData_Unavailable;
	
	int now = XPLMGetCycleNumber();

	switch(inDataType) {
	case xpmpDataType_Position:
	{
		if (plane->posAge != now)
		{
			result = plane->dataFunc(plane, inDataType, &plane->pos, plane->ref);
			if (result == xpmpData_NewData)
				plane->posAge = now;
		}

		XPMPPlanePosition_t *	posD = (XPMPPlanePosition_t *) outData;
		memcpy(posD, &plane->pos, XPMP_TMIN(posD->size, plane->pos.size));
		result = xpmpData_Unchanged;

		break;
	}
	case xpmpDataType_Surfaces:
	{
		if (plane->surfaceAge != now)
		{
			result = plane->dataFunc(plane, inDataType, &plane->surface, plane->ref);
			if (result == xpmpData_NewData)
				plane->surfaceAge = now;
		}

		XPMPPlaneSurfaces_t *	surfD = (XPMPPlaneSurfaces_t *) outData;
		memcpy(surfD, &plane->surface, XPMP_TMIN(surfD->size, plane->surface.size));
		result = xpmpData_Unchanged;

		break;
	}
	case xpmpDataType_Radar:
	{
		if (plane->radarAge != now)
		{
			result = plane->dataFunc(plane, inDataType, &plane->radar, plane->ref);
			if (result == xpmpData_NewData)
				plane->radarAge = now;
		}

		XPMPPlaneRadar_t *	radD = (XPMPPlaneRadar_t *) outData;
		memcpy(radD, &plane->radar, XPMP_TMIN(radD->size, plane->radar.size));
		result = xpmpData_Unchanged;

		break;
	}
	}
	return result;
}

XPMPPlanePtr	XPMPPlaneFromID(XPMPPlaneID inID, XPMPPlaneVector::iterator * outIter)
{
	assert(inID);
	if (outIter)
	{
		*outIter = std::find_if(gPlanes.begin(), gPlanes.end(), [inID] (const auto &p)
		{
			return p.get() == inID;
		});
		assert(*outIter != gPlanes.end());
	}
	return static_cast<XPMPPlanePtr>(inID);
}

void		XPMPSetPlaneRenderer(
		XPMPRenderPlanes_f  		inRenderer,
		void * 						inRef)
{
	gRenderer = inRenderer;
	gRendererRef = inRef;
}					

/********************************************************************************
 * RENDERING
 ********************************************************************************/

// These callbacks ping-pong the multiplayer count up and back depending
// on whether we're drawing the TCAS gauges or not.
int	XPMPEnablePlaneCount(
		XPLMDrawingPhase     /*inPhase*/,
		int                  /*inIsBefore*/,
		void *               /*inRefcon*/)
{
	if (!gHasControlOfAIAircraft) {
		return 1;
	}
	XPLMSetActiveAircraftCount(gEnableCount);
	return 1;
}
int	XPMPDisablePlaneCount(
		XPLMDrawingPhase     /*inPhase*/,
		int                  /*inIsBefore*/,
		void *               /*inRefcon*/)
{
	if (!gHasControlOfAIAircraft) {
		return 1;
	}
	XPLMSetActiveAircraftCount(1);
	return 1;
}


// This routine draws the actual planes.
int	XPMPRenderMultiplayerPlanes(
		XPLMDrawingPhase     /*inPhase*/,
		int                  /*inIsBefore*/,
		void *               /*inRefcon*/)
{
	static int is_blend = 0;
	
	static XPLMDataRef wrt = XPLMFindDataRef("sim/graphics/view/world_render_type");
	static XPLMDataRef prt = XPLMFindDataRef("sim/graphics/view/plane_render_type");
	
	int is_shadow = wrt != nullptr && XPLMGetDatai(wrt) != 0;
	
	if(prt)
		is_blend = XPLMGetDatai(prt) == 2;

	if (gRenderer)
		gRenderer(is_shadow ? 0 : is_blend,gRendererRef);
	else
		XPMPDefaultPlaneRenderer(is_shadow ? 0 : is_blend);
	if(!is_shadow)
		is_blend = 1 - is_blend;
	return 1;
}

// This routine draws the plane labels.
int	XPMPRenderPlaneLabels(
		XPLMDrawingPhase     /*inPhase*/,
		int                  /*inIsBefore*/,
		void *               /*inRefcon*/)
{
	if (!gRenderer)
		XPMPDefaultLabelRenderer();
	return 1;
}

bool			XPMPIsICAOValid(
		const char *				inICAO)
{
	return CSL_MatchPlane(inICAO, "", "", nullptr, false) != nullptr;
}

int 		XPMPGetPlaneModelQuality(
		XPMPPlaneID 				inPlane)
{
	XPMPPlanePtr thisPlane = XPMPPlaneFromID(inPlane);

	return thisPlane->match_quality;
}

int			XPMPModelMatchQuality(
		const char *				inICAO,
		const char *				inAirline,
		const char *				inLivery)
{
	int 	matchQuality = -1;
	CSL_MatchPlane(inICAO, inAirline, inLivery, &matchQuality, false);

	return matchQuality;
}

void		XPMPDumpOneCycle(void)
{
	CSL_Dump();
	gDumpOneRenderCycle = true;
}

bool XPMPGetVerticalOffset(XPMPPlaneID inPlane, double *outOffset)
{
	*outOffset = 0.0;
	XPMPPlanePtr plane = XPMPPlaneFromID(inPlane);
	if (!plane->model) { return false; }

	if (plane->model->isXsbVertOffsetAvail)
	{
		*outOffset = plane->model->xsbVertOffset;
		return true;
	}

	if (plane->model->plane_type == plane_Obj)
	{
		auto handle = std::atomic_load(&plane->objHandle);
		if (! handle) { return false; }
		*outOffset = handle->calcVertOffset;
		return true;
	}

	if (plane->model->plane_type == plane_Obj8)
	{
		// OBJ8 model always have 0.0 offset per spec
		*outOffset = 0.0;
		return false;
	}
	return false;
}
