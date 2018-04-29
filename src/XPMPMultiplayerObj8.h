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

#ifndef XPMPMultiplayerObj8_h
#define XPMPMultiplayerObj8_h

#include "XPLMScenery.h"
#include "XPLMPlanes.h"
#include "XPMPMultiplayer.h"
#include <string>
#include <memory>
#include <unordered_map>
#include <vector>
#include <functional>

/*

	OBJ8_AIRCRAFT
		OBJ8 LOW_LOD NO foo.obj
		OBJ8 GLASS YES bar.obj
	AIRLINE DAL
	ICAO B732 B733
	
 */

enum obj_draw_type {

	draw_lights = 0,
	draw_low_lod,
	draw_solid,
	draw_glass

};

enum obj_load_state {
	load_none = 0,		// not loaded, no attempt yet.
	load_loading,		// async load requested.
	load_loaded,		// (a)sync load complete
	load_failed			// (a)sync load failed
};

// Thin wrapper around XPLMObjectRef (which is just a void *)
struct Obj8Ref_t
{
    XPLMObjectRef objectRef;
};

struct	obj_for_acf {
	std::string			sourceFile;
	std::string			clonedFile;
	XPLMObjectRef		handle;
	obj_draw_type		draw_type;
	obj_load_state		load_state;
	bool				needs_animation;
	std::string			textureFile;
	std::string			litTextureFile;
};

class Obj8Manager
{
public:
	using ResourceHandle = std::shared_ptr<Obj8Ref_t>;
	using ResourceCache = std::unordered_map<std::string, std::weak_ptr<Obj8Ref_t>>;
	using ResourceCallback = std::function<void(const ResourceHandle &)>;

	Obj8Manager() = default;

	void loadAsync(obj_for_acf &objForAcf, const std::string &mtl, ResourceCallback callback);

private:
	struct XPLMCallbackRef
	{
		XPLMCallbackRef(Obj8Manager *manager, const std::string &filename)
			: m_manager(manager), m_filename(filename)
		{}

		Obj8Manager *m_manager = nullptr;
		std::string m_filename;
	};

	void xplmLoadAsync(const std::string &fileName, ResourceCallback callback);
	void objectLoaded(const std::string &fileName, XPLMObjectRef objectRef);
	static void Obj8RefDeleter(Obj8Ref_t *ref);

	ResourceCache m_resourceCache;
	std::unordered_map<std::string, std::vector<ResourceCallback>> m_pendingCallbacks;
};

using OBJ8Handle = Obj8Manager::ResourceHandle;

void	obj_init();

struct CSLPlane_t;
struct XPMPPlane_t;

void OBJ_LoadObj8Async(const std::shared_ptr<XPMPPlane_t> &plane);
OBJ8Handle OBJ_LoadObj8Model(const std::string &inFilePath);

void OBJ8_DrawModel(
    XPMPPlane_t *plane,
    double inX,
    double inY,
    double inZ,
    double inPitch,
    double inRoll,
    double inHeading,
    xpmp_LightStatus lights,
    XPLMPlaneDrawState_t *state,
    bool blend);


void	obj_deinit();

#endif
