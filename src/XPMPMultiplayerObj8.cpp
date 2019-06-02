/* 
 * Copyright (c) 2013, Laminar Research.
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

#include "XPMPMultiplayerObj8.h"
#include "XPMPMultiplayerVars.h"
#include "XStringUtils.h"
#include "XPLMScenery.h"
#include "XPLMUtilities.h"
#include "XPLMDataAccess.h"
#include <stddef.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <memory>
#include <thread>

using namespace std;

bool cloneObj8WithDifferentTexture(const std::string &sourceFileName, const std::string &targetFileName, const std::string &textureFile, const std::string &litTextureFile);

static Obj8Manager gObj8Manager;

struct	one_inst {
	XPLMDrawInfo_t			location;
	xpmp_LightStatus		lights;
	XPLMPlaneDrawState_t *	state;
};

static one_inst *	s_cur_plane = NULL;

enum {
	gear_rat = 0,
	flap_rat,
	spoi_rat,
	sbrk_rat,
	slat_rat,
	swep_rat,
	thrs_rat,
	ptch_rat,
	head_rat,
	roll_rat,
	thrs_rev,

	tax_lite_on,
	lan_lite_on,
	bcn_lite_on,
	str_lite_on,
	nav_lite_on,
	
	dref_dim
};

const char * dref_names[dref_dim] = {
	"libxplanemp/controls/gear_ratio",
	"libxplanemp/controls/flap_ratio",
	"libxplanemp/controls/spoiler_ratio",
	"libxplanemp/controls/speed_brake_ratio",
	"libxplanemp/controls/slat_ratio",
	"libxplanemp/controls/wing_sweep_ratio",
	"libxplanemp/controls/thrust_ratio",
	"libxplanemp/controls/yoke_pitch_ratio",
	"libxplanemp/controls/yoke_heading_ratio",
	"libxplanemp/controls/yoke_roll_ratio",
	"libxplanemp/controls/thrust_revers",

	"libxplanemp/controls/taxi_lites_on",
	"libxplanemp/controls/landing_lites_on",
	"libxplanemp/controls/beacon_lites_on",
	"libxplanemp/controls/strobe_lites_on",
	"libxplanemp/controls/nav_lites_on"
};


static float obj_get_float(void * inRefcon)
{
	if(s_cur_plane == NULL) return 0.0f;
	
	intptr_t v = reinterpret_cast<intptr_t>(inRefcon);
	switch(v)
	{
	case gear_rat:			return s_cur_plane->state->gearPosition;		break;
	case flap_rat:			return s_cur_plane->state->flapRatio;			break;
	case spoi_rat:			return s_cur_plane->state->spoilerRatio;		break;
	case sbrk_rat:			return s_cur_plane->state->speedBrakeRatio;		break;
	case slat_rat:			return s_cur_plane->state->slatRatio;			break;
	case swep_rat:			return s_cur_plane->state->wingSweep;			break;
	case thrs_rat:			return s_cur_plane->state->thrust;				break;
	case ptch_rat:			return s_cur_plane->state->yokePitch;			break;
	case head_rat:			return s_cur_plane->state->yokeHeading;			break;
	case roll_rat:			return s_cur_plane->state->yokeRoll;			break;
	case thrs_rev:			return static_cast<float>((s_cur_plane->state->thrust < 0.0) ? 1.0 : 0.0); break; //if thrust less than zero, reverse is on

	case tax_lite_on:		return static_cast<float>(s_cur_plane->lights.taxiLights);			break;
	case lan_lite_on:		return static_cast<float>(s_cur_plane->lights.landLights);			break;
	case bcn_lite_on:		return static_cast<float>(s_cur_plane->lights.bcnLights);			break;
	case str_lite_on:		return static_cast<float>(s_cur_plane->lights.strbLights);			break;
	case nav_lite_on:		return static_cast<float>(s_cur_plane->lights.navLights);			break;

	default:
		return 0.0f;
	}
}

int obj_get_float_array(
		void *               inRefcon,
		float *              inValues,
		int                  /*inOffset*/,
		int                  inCount)
{
	if(inValues == NULL)
		return 1;
	float rv = obj_get_float(inRefcon);
	for(int i = 0; i < inCount; ++i)
		inValues[i] = rv;
	return inCount;
}

static bool obj8_load_async = true;

void Obj8Manager::loadAsync(obj_for_acf &objForAcf, const std::string &mtl, bool needsCloning, ResourceCallback callback)
{
	std::string fileNameToLoad = objForAcf.sourceFile;

	if (needsCloning && objForAcf.draw_type == draw_solid)
	{
		if (objForAcf.clonedFile.empty())
		{
			// generate the name for new object
			std::string destObjFile = objForAcf.sourceFile;
			string::size_type pos = destObjFile.find_last_of(".");
			std::string suffix = "_" + mtl;
			if (pos != string::npos)
			{
				destObjFile.insert(pos, suffix);
			}
			else
			{
				destObjFile += suffix;
			}
			objForAcf.clonedFile = destObjFile;
		}

		fileNameToLoad = objForAcf.clonedFile;
	}

	std::string sourceObjFile = objForAcf.sourceFile;
	std::string destObjFile = objForAcf.clonedFile;

	ResourceHandle resource;

	// Is the resource fully loaded already?
	// If yes, simply return the shared handle
	auto resourceIt = m_resourceCache.find(fileNameToLoad);
	if (resourceIt != m_resourceCache.end())
	{
		resource = resourceIt->second.lock();
	}

	if (resource)
	{
		callback(resource);
		return;
	}

	// Is the resource currently being loaded?
	// If yes, attach our callback to be called when finished.
	auto pendingCallbacksIt = m_pendingCallbacks.find(fileNameToLoad);
	if (pendingCallbacksIt != m_pendingCallbacks.end())
	{
		pendingCallbacksIt->second.push_back(callback);
		return;
	}

	std::thread loaderThread([=]
	{
		if (needsCloning && objForAcf.draw_type == draw_solid)
		{
			cloneObj8WithDifferentTexture(sourceObjFile, destObjFile, objForAcf.textureFile, objForAcf.litTextureFile);
			gThreadSynchronizer.queueCall([=]()
			{
				XPLMDebugString(XPMP_CLIENT_NAME ": Cloning finished ");
				XPLMDebugString("(");
				XPLMDebugString(fileNameToLoad.c_str());
				XPLMDebugString(")\n");
                xplmLoadAsync(fileNameToLoad, callback);
			});
		}
		else
		{
			gThreadSynchronizer.queueCall([=]()
			{
				XPLMDebugString(XPMP_CLIENT_NAME ": Started async loading ");
				XPLMDebugString("(");
				XPLMDebugString(fileNameToLoad.c_str());
				XPLMDebugString(")\n");
				xplmLoadAsync(fileNameToLoad, callback);
			});
		}
	});
	if (loaderThread.joinable())
	{
		loaderThread.detach();
	}
}

void Obj8Manager::xplmLoadAsync(const std::string &fileName, ResourceCallback callback)
{
	m_pendingCallbacks[fileName].push_back(callback);

	std::unique_ptr<XPLMCallbackRef> callbackRef = std::make_unique<XPLMCallbackRef>(this, fileName);
	XPLMLoadObjectAsync(fileName.c_str(), [](XPLMObjectRef objectRef, void *refcon)
	{
		std::unique_ptr<XPLMCallbackRef> callbackRef(static_cast<XPLMCallbackRef *>(refcon));
		callbackRef->m_manager->objectLoaded(callbackRef->m_filename, objectRef);
	}, callbackRef.get());
	callbackRef.release();
}

void Obj8Manager::objectLoaded(const std::string &fileName, XPLMObjectRef objectRef)
{
	Obj8Ref_t obj8Ref;
	obj8Ref.objectRef = objectRef;

	if (objectRef)
	{
		ResourceHandle handle(new Obj8Ref_t(obj8Ref), Obj8RefDeleter);
		m_resourceCache[fileName] = handle;

		XPLMDebugString(XPMP_CLIENT_NAME ": Async loading succeeded ");
		XPLMDebugString("(");
		XPLMDebugString(fileName.c_str());
		XPLMDebugString(")\n");

		auto callbacksIt = m_pendingCallbacks.find(fileName);
		if (callbacksIt != m_pendingCallbacks.end())
		{
			for (auto &callback : callbacksIt->second)
			{
				callback(handle);
			}
			m_pendingCallbacks.erase(callbacksIt);
		}
	}
	else
	{
		XPLMDebugString(XPMP_CLIENT_NAME ": Async loading failed ");
		XPLMDebugString("(");
		XPLMDebugString(fileName.c_str());
		XPLMDebugString(")\n");

		auto callbacksIt = m_pendingCallbacks.find(fileName);
		if (callbacksIt != m_pendingCallbacks.end())
		{
			for (auto &callback : callbacksIt->second)
			{
				callback(nullptr);
			}
			m_pendingCallbacks.erase(callbacksIt);
		}

		m_pendingCallbacks.erase(fileName);
	}
}

void Obj8Manager::Obj8RefDeleter(Obj8Ref_t *ref)
{
	XPLMUnloadObject(ref->objectRef);
	delete ref;
}

void	obj_init()
{
	int sim, xplm;
	XPLMHostApplicationID app;
	XPLMGetVersions(&sim,&xplm,&app);
	// Ben says: we need the 2.10 SDK (e.g. X-Plane 10) to have async load at all.  But we need 10.30 to pick up an SDK bug
	// fix where async load crashes if we queue a second load before the first completes.  So for users on 10.25, they get
	// pauses.
	if (1 == gIntPrefsFunc("debug", "allow_obj8_async_load", 0) && sim >= 10300) {
		obj8_load_async = true;	
	} else {
		obj8_load_async = false;
	}
	
	for(int i = 0; i < dref_dim; ++i)
	{
		XPLMRegisterDataAccessor(
					dref_names[i], xplmType_Float|xplmType_FloatArray, 0,
					NULL, NULL,
					obj_get_float, NULL,
					NULL, NULL,
					NULL, NULL,
					obj_get_float_array, NULL,
					NULL, NULL, reinterpret_cast<void *>(static_cast<intptr_t>(i)), NULL);
	}
}


void obj_deinit()
{
}

bool cloneObj8WithDifferentTexture(const std::string &sourceFileName, const std::string &targetFileName, const std::string &textureFile, const std::string &litTextureFile)
{
	// copy and edit new object
	if (!textureFile.empty() && !fileExists(targetFileName))
	{
		std::string srcObjContent = getFileContent(sourceFileName);
		if (srcObjContent.empty())
		{
			XPLMDebugString(std::string(XPMP_CLIENT_NAME" Warning: Could not open " + sourceFileName + " for cloning.\n").c_str());
			return false;
		}

		bool textureHasWritten = false;
		bool litTextureHasWritten = false;
		std::istringstream sin(srcObjContent);
		std::ostringstream sout;
		std::string line;
		while (std::getline(sin, line))
		{
			if (line.size() == 0 || line.at(0) == ';' || line.at(0) == '#' || line.at(0) == '/')
			{
				sout << line << std::endl;
				continue;
			}

			trim(line);
			std::vector<std::string> tokens = tokenize(line);
			if (tokens.size() >= 2)
			{
				if (tokens[0] == "TEXTURE")
				{
					if (!textureHasWritten) { sout << "TEXTURE " << textureFile << std::endl; }
					textureHasWritten = true;
					continue;
				}
				if (tokens[0] == "TEXTURE_LIT")
				{
					if (!litTextureHasWritten) { sout << "TEXTURE_LIT " << litTextureFile << std::endl; }
					litTextureHasWritten = true;
					continue;
				}
				if (tokens[0] == "VT"
					|| tokens[0] == "VLINE"
					|| tokens[0] == "VLIGHT"
					|| tokens[0] == "IDX"
					|| tokens[0] == "IDX10")
				{
					if (!textureHasWritten)
					{
						sout << "TEXTURE " << textureFile << std::endl;
						textureHasWritten = true;
					}
					if (!litTextureHasWritten)
					{
						sout << "TEXTURE_LIT " << litTextureFile << std::endl;
						litTextureHasWritten = true;
					}
				}
			}
			sout << line << std::endl;
		}
		writeFileContent(targetFileName, sout.str());
		XPLMDebugString(std::string(XPMP_CLIENT_NAME": Modified obj8 has been created at: " + targetFileName + "\n").c_str());
	}
	return true;
}

void OBJ_LoadObj8Async(const std::shared_ptr<XPMPPlane_t> &plane)
{
	for (auto &attachment : plane->model->attachments)
	{
		Obj8Info_t obj8Info;
		obj8Info.index = plane->obj8Handles.size();
		obj8Info.drawType = attachment.draw_type;
		plane->obj8Handles[obj8Info] = nullptr;
		std::string mtlCode = plane->model->getMtlCode();

		// If the model has an additional texture defined, we need to clone the OBJ8
		bool shouldClone = !plane->model->textureName.empty();

		gObj8Manager.loadAsync(attachment, mtlCode, shouldClone, [plane, obj8Info](const Obj8Manager::ResourceHandle &resourceHandle)
		{
			if (! resourceHandle)
			{
				gThreadSynchronizer.queueCall([=]()
				{
					plane->planeLoadedFunc(plane.get(), false, plane->ref);
				});
				return;
			}

			bool allObj8Loaded = true;
			for (auto &obj8handle : plane->obj8Handles)
			{
				if (obj8handle.first.index == obj8Info.index)
				{
					std::atomic_store(&obj8handle.second, resourceHandle);
				}

				if (! obj8handle.second) { allObj8Loaded = false; }
			}

			if (!plane->allObj8Loaded && allObj8Loaded)
			{
				plane->allObj8Loaded = true;
				gThreadSynchronizer.queueCall([=]()
				{
					XPLMDebugString(XPMP_CLIENT_NAME ": Plane fully loaded ");
					XPLMDebugString("(");
					XPLMDebugString(plane->pos.label);
					XPLMDebugString(")\n");
					plane->planeLoadedFunc(plane.get(), true, plane->ref);
				});
			}
		});
	}
}

void OBJ8_DrawModel(XPMPPlane_t *plane, double inX, double inY, double inZ, double inPitch, double inRoll, double inHeading, xpmp_LightStatus lights, XPLMPlaneDrawState_t *state, bool blend)
{
	for (auto &pair : plane->obj8Handles)
	{
		auto obj8Handle = std::atomic_load(&pair.second);
		if (!obj8Handle) { return; }
	}

	static XPLMDataRef night_lighting_ref = XPLMFindDataRef("sim/graphics/scenery/percent_lights_on");
	bool use_night = XPLMGetDataf(night_lighting_ref) > 0.25;

	XPLMDrawInfo_t drawInfo;

	drawInfo.structSize = sizeof(drawInfo);
	drawInfo.x = static_cast<float>(inX);
	drawInfo.y = static_cast<float>(inY);
	drawInfo.z = static_cast<float>(inZ);
	drawInfo.pitch = static_cast<float>(inPitch);
	drawInfo.roll = static_cast<float>(inRoll);
	drawInfo.heading = static_cast<float>(inHeading);

	one_inst i;
	i.location = drawInfo;
	i.lights = lights;
	i.state = state;

	s_cur_plane = &i;

	for (const auto &pair : plane->obj8Handles)
	{
		auto obj8Handle = std::atomic_load(&pair.second);
		if (pair.first.drawType == draw_glass && blend)
		{
			// Draw transluent
			XPLMDrawObjects(obj8Handle->objectRef, 1, &drawInfo, use_night, 0);
		}
		else
		{
			// Draw solid
			XPLMDrawObjects(obj8Handle->objectRef, 1, &drawInfo, use_night, 0);
		}
	}

	s_cur_plane = nullptr;
}
