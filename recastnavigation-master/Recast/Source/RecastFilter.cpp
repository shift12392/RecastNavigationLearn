//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include "Recast.h"
#include "RecastAssert.h"

/// @par
///
/// Allows the formation of walkable regions that will flow over low lying 
/// objects such as curbs, and up structures such as stairways. 
/// 
/// Two neighboring spans are walkable if: <tt>rcAbs(currentSpan.smax - neighborSpan.smax) < waklableClimb</tt>
/// 
/// @warning Will override the effect of #rcFilterLedgeSpans.  So if both filters are used, call
/// #rcFilterLedgeSpans after calling this filter. 
///
/// @see rcHeightfield, rcConfig
void rcFilterLowHangingWalkableObstacles(rcContext* ctx, const int walkableClimb, rcHeightfield& solid)
{
	rcAssert(ctx);

	rcScopedTimer timer(ctx, RC_TIMER_FILTER_LOW_OBSTACLES);
	
	const int w = solid.width;
	const int h = solid.height;
	
	//行扫描
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			// prev span 前一个span
			rcSpan* ps = 0;
			bool previousWalkable = false;
			unsigned char previousArea = RC_NULL_AREA;
			
			// 从低到高的遍历span串中的span。
			for (rcSpan* s = solid.spans[x + y*w]; s; ps = s, s = s->next)
			{
				const bool walkable = s->area != RC_NULL_AREA;
				// If current span is not walkable, but there is walkable
				// span just below it, mark the span above it walkable too.

				//  当前span不可走，前一个span可走，同时当前span的距离前一个span高度小于walkableClimb，则当前span被改为可走。
				//  然后，后一个span也不可走，则不改变后一个span的可走标志。
				if (!walkable && previousWalkable)
				{
					if (rcAbs((int)s->smax - (int)ps->smax) <= walkableClimb)
						s->area = previousArea;      //注意，这里可没有改变walkable的值。
				}

				// Copy walkable flag so that it cannot propagate
				// past multiple non-walkable objects.
				previousWalkable = walkable;
				previousArea = s->area;
			}
		}
	}
}

/// @par
///
/// A ledge is a span with one or more neighbors whose maximum is further away than @p walkableClimb
/// from the current span's maximum.
/// This method removes the impact of the overestimation of conservative voxelization 
/// so the resulting mesh will not have regions hanging in the air over ledges.
/// 
/// A span is a ledge if: <tt>rcAbs(currentSpan.smax - neighborSpan.smax) > walkableClimb</tt>
/// 
/// @see rcHeightfield, rcConfig
void rcFilterLedgeSpans(rcContext* ctx, const int walkableHeight, const int walkableClimb,
						rcHeightfield& solid)
{
	rcAssert(ctx);
	
	rcScopedTimer timer(ctx, RC_TIMER_FILTER_BORDER);

	const int w = solid.width;
	const int h = solid.height;
	const int MAX_HEIGHT = 0xffff;
	
	// Mark border spans.
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			for (rcSpan* s = solid.spans[x + y*w]; s; s = s->next)
			{
				// Skip non walkable spans.
				if (s->area == RC_NULL_AREA)
					continue;
				
				const int bot = (int)(s->smax);
				const int top = s->next ? (int)(s->next->smin) : MAX_HEIGHT;
				
				// Find neighbours minimum height.
				// 1、如果agent站在当前s上，要从当前s走到轴邻居span串上，必须不碰头，即满足( rcMin(top,ntop) - rcMax(bot,nbot) > walkableHeight)这个条件，walkableHeight是agent的高度。
				// 因为轴邻居span串上有好多span，所以minh表示，轴邻居span串上，满足条件1的，最小的（nbot - bot）。
				int minh = MAX_HEIGHT;

				// Min and max height of accessible neighbours.
				// 2、如果agent站在当前s上，要从当前s走到轴邻居span串上，必须能跨上去，即满足(rcAbs(nbot - bot) <= walkableClimb)这个条件。
				// 因为轴邻居span串上有好多span，所以asmin表示，轴邻居span串上，满足条件2的，最小的span的nbot。asmax表示，轴邻居span串上，满足条件2的，最大的span的nbot。
				int asmin = s->smax;
				int asmax = s->smax;

				// 遍历4个轴邻居
				for (int dir = 0; dir < 4; ++dir)
				{
					int dx = x + rcGetDirOffsetX(dir);
					int dy = y + rcGetDirOffsetY(dir);
					// Skip neighbours which are out of bounds.
				    // 处理在边框上的轴邻居
					if (dx < 0 || dy < 0 || dx >= w || dy >= h)
					{
						minh = rcMin(minh, -walkableClimb - bot);
						continue;
					}

					// From minus infinity to the first span.
					// 处理轴邻居span串上的第一个span。
					rcSpan* ns = solid.spans[dx + dy*w];
					int nbot = -walkableClimb;                    
					int ntop = ns ? (int)ns->smin : MAX_HEIGHT;
					// Skip neightbour if the gap between the spans is too small.
					if (rcMin(top,ntop) - rcMax(bot,nbot) > walkableHeight)
						minh = rcMin(minh, nbot - bot);
					
					// Rest of the spans.
					// 从低到高，遍历轴邻居span串上的所有span。
					for (ns = solid.spans[dx + dy*w]; ns; ns = ns->next)
					{
						nbot = (int)ns->smax;
						ntop = ns->next ? (int)ns->next->smin : MAX_HEIGHT;
						// Skip neightbour if the gap between the spans is too small.
						// 判断洞窟能走过，不碰头
						if (rcMin(top,ntop) - rcMax(bot,nbot) > walkableHeight)
						{
							minh = rcMin(minh, nbot - bot);
						
							// Find min/max accessible neighbour height. 
							if (rcAbs(nbot - bot) <= walkableClimb)
							{
								if (nbot < asmin) asmin = nbot;
								if (nbot > asmax) asmax = nbot;
							}
							
						}
					}
				}
				
				// The current span is close to a ledge if the drop to any
				// neighbour span is less than the walkableClimb.
				// 如果到任何相邻跨度的降落小于walkableClimb，则当前span是ledge，不可行走。
				// 4个轴邻居总得存在一个，可以从当前span走到这个轴邻居上
				if (minh < -walkableClimb)
				{
					s->area = RC_NULL_AREA;
				}
				// If the difference between all neighbours is too large,
				// we are at steep slope, mark the span as ledge.
				// 如果4个轴邻居中最高的和最低的相差太大，则当前span不可走
				else if ((asmax - asmin) > walkableClimb)
				{
					s->area = RC_NULL_AREA;
				}
			}
		}
	}
}

/// @par
///
/// For this filter, the clearance above the span is the distance from the span's 
/// maximum to the next higher span's minimum. (Same grid column.)
/// 
/// @see rcHeightfield, rcConfig
void rcFilterWalkableLowHeightSpans(rcContext* ctx, int walkableHeight, rcHeightfield& solid)
{
	rcAssert(ctx);
	
	rcScopedTimer timer(ctx, RC_TIMER_FILTER_WALKABLE);
	
	const int w = solid.width;
	const int h = solid.height;
	const int MAX_HEIGHT = 0xffff;
	
	// Remove walkable flag from spans which do not have enough
	// space above them for the agent to stand there.
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			//遍历当前span串上的所有span
			for (rcSpan* s = solid.spans[x + y*w]; s; s = s->next)
			{
				// 上下span的高度小于walkableHeight
				const int bot = (int)(s->smax);
				const int top = s->next ? (int)(s->next->smin) : MAX_HEIGHT;
				if ((top - bot) <= walkableHeight)
					s->area = RC_NULL_AREA;
			}
		}
	}
}
