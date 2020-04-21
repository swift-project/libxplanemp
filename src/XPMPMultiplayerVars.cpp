/*
 * Copyright (c) 2005, Ben Supnik and Chris Serio.
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
#include "XPMPMultiplayerVars.h"
#include <stddef.h>


int								(* gIntPrefsFunc)(const char *, const char *, int) = nullptr;
float							(* gFloatPrefsFunc)(const char *, const char *, float) = nullptr;

XPMPPlaneVector					gPlanes;
XPMPPlaneNotifierVector			gObservers;
XPMPRenderPlanes_f				gRenderer = nullptr;
void *							gRendererRef;
int								gDumpOneRenderCycle = 0;
int 							gEnableCount = 1;

std::vector<CSLPackage_t>		gPackages;
std::map<std::string, std::string>	gGroupings;

std::string						gDefaultPlane;
std::map<std::string, CSLAircraftCode_t>	gAircraftCodes;
ThreadSynchronizer				gThreadSynchronizer;

void ThreadSynchronizer::queueCall(std::function<void()> func)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_qeuedCalls.push_back(func);
}

void ThreadSynchronizer::executeQueuedCalls()
{
	if (m_mutex.try_lock())
	{
		while (m_qeuedCalls.size() > 0)
		{
			m_qeuedCalls.front()();
			m_qeuedCalls.pop_front();
		}
		m_mutex.unlock();
	}
}

float ThreadSynchronizer::flightLoopCallback(float, float, int, void *refcon)
{
	auto *obj = static_cast<ThreadSynchronizer *>(refcon);
	obj->executeQueuedCalls();
	return -1;
}
