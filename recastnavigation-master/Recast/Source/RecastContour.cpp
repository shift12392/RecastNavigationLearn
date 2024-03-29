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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"


static int getCornerHeight(int x, int y, int i, int dir,
						   const rcCompactHeightfield& chf,
						   bool& isBorderVertex)
{
	const rcCompactSpan& s = chf.spans[i];
	int ch = (int)s.y;
	int dirp = (dir+1) & 0x3;  //按顺序，dir的下一个方向
	
	unsigned int regs[4] = {0,0,0,0};
	
	// Combine region and area codes in order to prevent
	// border vertices which are in between two areas to be removed.
	regs[0] = chf.spans[i].reg | (chf.areas[i] << 16);         //高16位是area标记，低16位是region ID。
	
	// 得到，序号i的OpenSpan的y值与序号i的OpenSpan相邻的三个OpenSpan（右手坐标系的右手法则的旋转方向）的y值，中的最大值。
	// 比如：dir的方向为下，则三个相邻的OpenSpan为（下，右下，右）。
	if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
	{
		const int ax = x + rcGetDirOffsetX(dir);
		const int ay = y + rcGetDirOffsetY(dir);
		const int ai = (int)chf.cells[ax+ay*chf.width].index + rcGetCon(s, dir);
		const rcCompactSpan& as = chf.spans[ai];         //下
		ch = rcMax(ch, (int)as.y);
		regs[1] = chf.spans[ai].reg | (chf.areas[ai] << 16);
		if (rcGetCon(as, dirp) != RC_NOT_CONNECTED)
		{
			const int ax2 = ax + rcGetDirOffsetX(dirp);
			const int ay2 = ay + rcGetDirOffsetY(dirp);
			const int ai2 = (int)chf.cells[ax2+ay2*chf.width].index + rcGetCon(as, dirp);
			const rcCompactSpan& as2 = chf.spans[ai2];   //下的右
			ch = rcMax(ch, (int)as2.y);
			regs[2] = chf.spans[ai2].reg | (chf.areas[ai2] << 16);
		}
	}
	if (rcGetCon(s, dirp) != RC_NOT_CONNECTED)
	{
		const int ax = x + rcGetDirOffsetX(dirp);
		const int ay = y + rcGetDirOffsetY(dirp);
		const int ai = (int)chf.cells[ax+ay*chf.width].index + rcGetCon(s, dirp);
		const rcCompactSpan& as = chf.spans[ai];        //右
		ch = rcMax(ch, (int)as.y);
		regs[3] = chf.spans[ai].reg | (chf.areas[ai] << 16);
		if (rcGetCon(as, dir) != RC_NOT_CONNECTED)
		{
			const int ax2 = ax + rcGetDirOffsetX(dir);
			const int ay2 = ay + rcGetDirOffsetY(dir);
			const int ai2 = (int)chf.cells[ax2+ay2*chf.width].index + rcGetCon(as, dir);
			const rcCompactSpan& as2 = chf.spans[ai2];  //右的下（注：右的下 和 下的右 可能不是相同的OpenSpan）。
			ch = rcMax(ch, (int)as2.y);
			regs[2] = chf.spans[ai2].reg | (chf.areas[ai2] << 16);
		}
	}

	// Check if the vertex is special edge vertex, these vertices will be removed later.
	// 检查顶点是否是特殊的边缘顶点，这些顶点将在以后删除。
	for (int j = 0; j < 4; ++j)
	{
		// 0 1 2 3
		// 1 2 3 0
		// 2 3 0 1
		// 3 0 1 2
		const int a = j;
		const int b = (j+1) & 0x3;
		const int c = (j+2) & 0x3;
		const int d = (j+3) & 0x3;
		
		// The vertex is a border vertex there are two same exterior cells in a row,
		// followed by two interior cells and none of the regions are out of bounds.
		const bool twoSameExts = (regs[a] & regs[b] & RC_BORDER_REG) != 0 && regs[a] == regs[b];    //a 和 b都在同一个BORDER地区 
		const bool twoInts = ((regs[c] | regs[d]) & RC_BORDER_REG) == 0;                            //c和d都不是BORDER地区
		const bool intsSameArea = (regs[c]>>16) == (regs[d]>>16);                                   //c和d的Area标记相同
		const bool noZeros = regs[a] != 0 && regs[b] != 0 && regs[c] != 0 && regs[d] != 0;          //a b c d都在地区中
		if (twoSameExts && twoInts && intsSameArea && noZeros)
		{
			isBorderVertex = true;
			break;
		}
	}
	
	return ch;
}

/// 沿着轮廓走，找出所有的轮廓点
/// @param x,y  单元格坐标（单元格坐标系）
/// @param i    OpenSpan索引
/// @param chf  紧凑型高度场
/// @param flags OpenSpan的4个邻接点边界类型的数组。如果某个方向是地区边界，那么对应的bit就是0。
/// @param points 向外输出轮廓点
static void walkContour(int x, int y, int i,
						rcCompactHeightfield& chf,
						unsigned char* flags, rcIntArray& points)
{
	// Choose the first non-connected edge
	// 找到这个OpenSpan的第一个地区边界的方向。
	unsigned char dir = 0;
	while ((flags[i] & (1 << dir)) == 0)
		dir++;
	
	unsigned char startDir = dir;
	int starti = i;
	
	const unsigned char area = chf.areas[i];
	
	// 以当前OpenSpan的开始，逐渐展开。为什么是4000次遍历？
	int iter = 0;
	while (++iter < 40000)
	{
		if (flags[i] & (1 << dir))  //这个方向的边界是地区边界
		{
			// Choose the edge corner
			bool isBorderVertex = false;
			bool isAreaBorder = false;

			// 一，得到dir方向轮廓顶点（体素空间单位）
			int px = x;
			int py = getCornerHeight(x, y, i, dir, chf, isBorderVertex);
			int pz = y;
			switch(dir)
			{
				case 0: pz++; break;                //左边,使用左下角的点
				case 1: px++; pz++; break;          //下面,使用右下角的点
				case 2: px++; break;                //右边,使用右上角的点
			}

			// 二，dir方向的邻接OpenSpan的地区ID和状态。
			int r = 0;                                        //低16位是dir方向的邻接OpenSpan的地区ID。
			const rcCompactSpan& s = chf.spans[i];
			if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
			{
				const int ax = x + rcGetDirOffsetX(dir);
				const int ay = y + rcGetDirOffsetY(dir);
				const int ai = (int)chf.cells[ax+ay*chf.width].index + rcGetCon(s, dir);   //得到dir方向邻居索引
				r = (int)chf.spans[ai].reg;
				if (area != chf.areas[ai])                   //dir方向的邻接OpenSpan不可走。 
					isAreaBorder = true;
			}
			if (isBorderVertex)
				r |= RC_BORDER_VERTEX;                       //标记到第17位
			if (isAreaBorder)
				r |= RC_AREA_BORDER;                         //标记到第18位

			// 三，保存到数组中
			points.push(px);
			points.push(py);
			points.push(pz);
			points.push(r);
			
			// 
			flags[i] &= ~(1 << dir); // Remove visited edges
			dir = (dir+1) & 0x3;  // Rotate CW  按右手坐标系的右手法则来旋转。如果dir为3（上面的邻居），则新的dir为0（左边的邻居）。

			//继续循环判断新的方向的邻居的边界类型
		}
		else
		{
			//这个方向的边界是内部边界，则移动到这个方向的邻接OpenSpan

			int ni = -1;                                    //dir方向邻接OpenSpan的索引
			const int nx = x + rcGetDirOffsetX(dir);
			const int ny = y + rcGetDirOffsetY(dir);
			const rcCompactSpan& s = chf.spans[i];
			if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
			{
				const rcCompactCell& nc = chf.cells[nx+ny*chf.width];
				ni = (int)nc.index + rcGetCon(s, dir);                    
			}
			if (ni == -1)
			{
				// Should not happen.
				return;
			}

			//移动了单元格
			x = nx;
			y = ny;
			i = ni;
			dir = (dir+3) & 0x3;	// Rotate CCW
		}
		
		//回到了起始OpenSapn的起始方向。退出
		if (starti == i && startDir == dir)
		{
			break;
		}
	}
}

static float distancePtSeg(const int x, const int z,
						   const int px, const int pz,
						   const int qx, const int qz)
{
	float pqx = (float)(qx - px);
	float pqz = (float)(qz - pz);
	float dx = (float)(x - px);
	float dz = (float)(z - pz);
	float d = pqx*pqx + pqz*pqz;
	float t = pqx*dx + pqz*dz;
	if (d > 0)
		t /= d;
	if (t < 0)
		t = 0;
	else if (t > 1)
		t = 1;
	
	dx = px + t*pqx - x;
	dz = pz + t*pqz - z;
	
	return dx*dx + dz*dz;
}

// 简化轮廓
static void simplifyContour(rcIntArray& points, rcIntArray& simplified,
							const float maxError, const int maxEdgeLen, const int buildFlags)
{
	//注：有两种类型的轮廓。第一种是，一个地区和其他地区（指有地区ID或者Border地区）相邻，则这种轮廓称之为portals（入口）。
	//第二种，一个地区只和“空”地区相邻，则这个地区孤零零的。“空”地区指没有地区ID的那些OpenSpan组成的区域。

	// Add initial points.
	// 检查这条轮廓，是否有portals。
	bool hasConnections = false;
	for (int i = 0; i < points.size(); i += 4)
	{
		if ((points[i+3] & RC_CONTOUR_REG_MASK) != 0)    // 注:获取points[i+3]的低16位，低16位存放的是Regon ID。
		{
			hasConnections = true;
			break;
		}
	}
	
	// 第一种情况：如果这条地区轮廓有portals，则简化轮廓很简单，只保留一些关键点。
	if (hasConnections)
	{
		// The contour has some portals to other regions.
		// Add a new point to every location where the region changes.
		for (int i = 0, ni = points.size()/4; i < ni; ++i)
		{
			int ii = (i+1) % ni;
			const bool differentRegs = (points[i*4+3] & RC_CONTOUR_REG_MASK) != (points[ii*4+3] & RC_CONTOUR_REG_MASK);    // 判断邻接OpenSpan的Regon ID是否相同。
			const bool areaBorders = (points[i*4+3] & RC_AREA_BORDER) != (points[ii*4+3] & RC_AREA_BORDER);                // 这个轮廓点的两边的OpenSpan的可行走标记不同。
			if (differentRegs || areaBorders)        
			{
				// 1、两个邻接OpenSpan的Regon ID不一样的轮廓点留下。
				// 2、是同一Regon ID，但是两个轮廓点的可行走标记不一样的留下。

				simplified.push(points[i*4+0]);
				simplified.push(points[i*4+1]);
				simplified.push(points[i*4+2]);
				simplified.push(i);                          //注：这个是point在points数组的索引
			}
		}
	}
	
	// 如果没有得到简化点，则增加两个初始点。
	if (simplified.size() == 0)
	{
		// If there is no connections at all,
		// create some initial points for the simplification process.   如果根本没有任何连接，请为这个简化过程创建一些初始点。
		// Find lower-left and upper-right vertices of the contour.     查找轮廓的左上角和右下角的点。（注：右手坐标系，x-z平面坐标和windows桌面坐标一样。）
		int llx = points[0];
		int lly = points[1];
		int llz = points[2];
		int lli = 0;
		int urx = points[0];
		int ury = points[1];
		int urz = points[2];
		int uri = 0;
		for (int i = 0; i < points.size(); i += 4)
		{
			int x = points[i+0];
			int y = points[i+1];
			int z = points[i+2];
			if (x < llx || (x == llx && z < llz))    //找到最小的x和z的点
			{
				llx = x;
				lly = y;
				llz = z;
				lli = i/4;
			}
			if (x > urx || (x == urx && z > urz))   //找到最大的x和z的点
			{
				urx = x;
				ury = y;
				urz = z;
				uri = i/4;
			}
		}
		simplified.push(llx);
		simplified.push(lly);
		simplified.push(llz);
		simplified.push(lli);
		
		simplified.push(urx);
		simplified.push(ury);
		simplified.push(urz);
		simplified.push(uri);
	}
	
	// 再次简化顶点
	// 第一步：使用Douglas-Peucker算法，使用maxError参数决定将哪些顶点丢弃以获得简化的线段。

	// Add points until all raw points are within
	// error tolerance to the simplified shape.            添加点，直到所有原始点都在简化形状的误差范围内。
	const int pn = points.size()/4;
	for (int i = 0; i < simplified.size()/4; )
	{
		int ii = (i+1) % (simplified.size()/4);        //环形循环，i的下一个
		
		int ax = simplified[i*4+0];
		int az = simplified[i*4+2];
		int ai = simplified[i*4+3];

		int bx = simplified[ii*4+0];
		int bz = simplified[ii*4+2];
		int bi = simplified[ii*4+3];

		// Find maximum deviation from the segment.
		float maxd = 0;
		int maxi = -1;
		int ci, cinc, endi;

		// Traverse the segment in lexilogical order so that the
		// max deviation is calculated similarly when traversing
		// opposite segments.
		if (bx > ax || (bx == ax && bz > az))   
		{//下一个关键点在右边或者是上面。
			cinc = 1;
			ci = (ai+cinc) % pn;
			endi = bi;
		}
		else
		{
			cinc = pn-1;
			ci = (bi+cinc) % pn;
			endi = ai;
			rcSwap(ax, bx);
			rcSwap(az, bz);
		}
		
		// Tessellate only outer edges or edges between areas.   仅对外部边缘或区域之间的边缘进行镶嵌。
		if ((points[ci*4+3] & RC_CONTOUR_REG_MASK) == 0 ||
			(points[ci*4+3] & RC_AREA_BORDER))
		{
			//找到一个最远点
			while (ci != endi)
			{
				float d = distancePtSeg(points[ci*4+0], points[ci*4+2], ax, az, bx, bz);
				if (d > maxd)
				{
					maxd = d;
					maxi = ci;
				}
				ci = (ci+cinc) % pn;
			}
		}
		
		
		// If the max deviation is larger than accepted error,
		// add new point, else continue to next segment.           如果最大偏差大于可接受的误差，则添加新点，否则继续下一个分段。
		if (maxi != -1 && maxd > (maxError*maxError))
		{
			// Add space for the new point.  为新点添加空间。
			simplified.resize(simplified.size()+4);
			const int n = simplified.size()/4;
			for (int j = n-1; j > i; --j)    //向后移动
			{
				simplified[j*4+0] = simplified[(j-1)*4+0];
				simplified[j*4+1] = simplified[(j-1)*4+1];
				simplified[j*4+2] = simplified[(j-1)*4+2];
				simplified[j*4+3] = simplified[(j-1)*4+3];
			}
			// Add the point.
			simplified[(i+1)*4+0] = points[maxi*4+0];
			simplified[(i+1)*4+1] = points[maxi*4+1];
			simplified[(i+1)*4+2] = points[maxi*4+2];
			simplified[(i+1)*4+3] = maxi;

			//注意这里没有改变i，所以再次循环执行一次。
		}
		else
		{
			++i;
		}
	}
	
	// Split too long edges.
	if (maxEdgeLen > 0 && (buildFlags & (RC_CONTOUR_TESS_WALL_EDGES|RC_CONTOUR_TESS_AREA_EDGES)) != 0)
	{
		for (int i = 0; i < simplified.size()/4; )
		{
			const int ii = (i+1) % (simplified.size()/4);
			
			const int ax = simplified[i*4+0];
			const int az = simplified[i*4+2];
			const int ai = simplified[i*4+3];
			
			const int bx = simplified[ii*4+0];
			const int bz = simplified[ii*4+2];
			const int bi = simplified[ii*4+3];
			
			// Find maximum deviation from the segment.
			int maxi = -1;
			int ci = (ai+1) % pn;
			
			// Tessellate only outer edges or edges between areas.
			bool tess = false;
			// Wall edges.
			if ((buildFlags & RC_CONTOUR_TESS_WALL_EDGES) && (points[ci*4+3] & RC_CONTOUR_REG_MASK) == 0)
				tess = true;
			// Edges between areas.
			if ((buildFlags & RC_CONTOUR_TESS_AREA_EDGES) && (points[ci*4+3] & RC_AREA_BORDER))
				tess = true;
			
			if (tess)
			{
				int dx = bx - ax;
				int dz = bz - az;
				if (dx*dx + dz*dz > maxEdgeLen*maxEdgeLen)
				{
					// Round based on the segments in lexilogical order so that the
					// max tesselation is consistent regardles in which direction
					// segments are traversed.
					const int n = bi < ai ? (bi+pn - ai) : (bi - ai);
					if (n > 1)
					{
						if (bx > ax || (bx == ax && bz > az))
							maxi = (ai + n/2) % pn;
						else
							maxi = (ai + (n+1)/2) % pn;
					}
				}
			}
			
			// If the max deviation is larger than accepted error,
			// add new point, else continue to next segment.
			if (maxi != -1)
			{
				// Add space for the new point.
				simplified.resize(simplified.size()+4);
				const int n = simplified.size()/4;
				for (int j = n-1; j > i; --j)
				{
					simplified[j*4+0] = simplified[(j-1)*4+0];
					simplified[j*4+1] = simplified[(j-1)*4+1];
					simplified[j*4+2] = simplified[(j-1)*4+2];
					simplified[j*4+3] = simplified[(j-1)*4+3];
				}
				// Add the point.
				simplified[(i+1)*4+0] = points[maxi*4+0];
				simplified[(i+1)*4+1] = points[maxi*4+1];
				simplified[(i+1)*4+2] = points[maxi*4+2];
				simplified[(i+1)*4+3] = maxi;
			}
			else
			{
				++i;
			}
		}
	}
	
	for (int i = 0; i < simplified.size()/4; ++i)
	{
		// The edge vertex flag is take from the current raw point,
		// and the neighbour region is take from the next raw point.
		const int ai = (simplified[i*4+3]+1) % pn;
		const int bi = simplified[i*4+3];
		simplified[i*4+3] = (points[ai*4+3] & (RC_CONTOUR_REG_MASK|RC_AREA_BORDER)) | (points[bi*4+3] & RC_BORDER_VERTEX);
	}
	
}

static int calcAreaOfPolygon2D(const int* verts, const int nverts)
{
	int area = 0;
	for (int i = 0, j = nverts-1; i < nverts; j=i++)
	{
		const int* vi = &verts[i*4];
		const int* vj = &verts[j*4];
		area += vi[0] * vj[2] - vj[0] * vi[2];
	}
	return (area+1) / 2;
}

// TODO: these are the same as in RecastMesh.cpp, consider using the same.
// Last time I checked the if version got compiled using cmov, which was a lot faster than module (with idiv).
inline int prev(int i, int n) { return i-1 >= 0 ? i-1 : n-1; }
inline int next(int i, int n) { return i+1 < n ? i+1 : 0; }

inline int area2(const int* a, const int* b, const int* c)
{
	return (b[0] - a[0]) * (c[2] - a[2]) - (c[0] - a[0]) * (b[2] - a[2]);
}

//	Exclusive or: true iff exactly one argument is true.
//	The arguments are negated to ensure that they are 0/1
//	values.  Then the bitwise Xor operator may apply.
//	(This idea is due to Michael Baldwin.)
inline bool xorb(bool x, bool y)
{
	return !x ^ !y;
}

// Returns true iff c is strictly to the left of the directed
// line through a to b.
inline bool left(const int* a, const int* b, const int* c)
{
	return area2(a, b, c) < 0;
}

inline bool leftOn(const int* a, const int* b, const int* c)
{
	return area2(a, b, c) <= 0;
}

inline bool collinear(const int* a, const int* b, const int* c)
{
	return area2(a, b, c) == 0;
}

//	Returns true iff ab properly intersects cd: they share
//	a point interior to both segments.  The properness of the
//	intersection is ensured by using strict leftness.
static bool intersectProp(const int* a, const int* b, const int* c, const int* d)
{
	// Eliminate improper cases.
	if (collinear(a,b,c) || collinear(a,b,d) ||
		collinear(c,d,a) || collinear(c,d,b))
		return false;
	
	return xorb(left(a,b,c), left(a,b,d)) && xorb(left(c,d,a), left(c,d,b));
}

// Returns T iff (a,b,c) are collinear and point c lies
// on the closed segement ab.
static bool between(const int* a, const int* b, const int* c)
{
	if (!collinear(a, b, c))
		return false;
	// If ab not vertical, check betweenness on x; else on y.
	if (a[0] != b[0])
		return	((a[0] <= c[0]) && (c[0] <= b[0])) || ((a[0] >= c[0]) && (c[0] >= b[0]));
	else
		return	((a[2] <= c[2]) && (c[2] <= b[2])) || ((a[2] >= c[2]) && (c[2] >= b[2]));
}

// Returns true iff segments ab and cd intersect, properly or improperly.
static bool intersect(const int* a, const int* b, const int* c, const int* d)
{
	if (intersectProp(a, b, c, d))
		return true;
	else if (between(a, b, c) || between(a, b, d) ||
			 between(c, d, a) || between(c, d, b))
		return true;
	else
		return false;
}

static bool vequal(const int* a, const int* b)
{
	return a[0] == b[0] && a[2] == b[2];
}

static bool intersectSegCountour(const int* d0, const int* d1, int i, int n, const int* verts)
{
	// For each edge (k,k+1) of P
	for (int k = 0; k < n; k++)
	{
		int k1 = next(k, n);
		// Skip edges incident to i.
		if (i == k || i == k1)
			continue;
		const int* p0 = &verts[k * 4];
		const int* p1 = &verts[k1 * 4];
		if (vequal(d0, p0) || vequal(d1, p0) || vequal(d0, p1) || vequal(d1, p1))
			continue;
		
		if (intersect(d0, d1, p0, p1))
			return true;
	}
	return false;
}

static bool	inCone(int i, int n, const int* verts, const int* pj)
{
	const int* pi = &verts[i * 4];
	const int* pi1 = &verts[next(i, n) * 4];
	const int* pin1 = &verts[prev(i, n) * 4];
	
	// If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
	if (leftOn(pin1, pi, pi1))
		return left(pi, pj, pin1) && left(pj, pi, pi1);
	// Assume (i-1,i,i+1) not collinear.
	// else P[i] is reflex.
	return !(leftOn(pi, pj, pi1) && leftOn(pj, pi, pin1));
}


static void removeDegenerateSegments(rcIntArray& simplified)
{
	// Remove adjacent vertices which are equal on xz-plane,
	// or else the triangulator will get confused.
	//删除在xz平面上相等的相邻顶点，否则三角形将变得混乱。
	int npts = simplified.size()/4;
	for (int i = 0; i < npts; ++i)
	{
		int ni = next(i, npts);
		
		if (vequal(&simplified[i*4], &simplified[ni*4]))
		{
			// Degenerate segment, remove.
			// 向前移动数组，去除邻近的相等的点
			for (int j = i; j < simplified.size()/4-1; ++j)
			{
				simplified[j*4+0] = simplified[(j+1)*4+0];
				simplified[j*4+1] = simplified[(j+1)*4+1];
				simplified[j*4+2] = simplified[(j+1)*4+2];
				simplified[j*4+3] = simplified[(j+1)*4+3];
			}
			simplified.resize(simplified.size()-4);
			npts--;
		}
	}
}


static bool mergeContours(rcContour& ca, rcContour& cb, int ia, int ib)
{
	const int maxVerts = ca.nverts + cb.nverts + 2;
	int* verts = (int*)rcAlloc(sizeof(int)*maxVerts*4, RC_ALLOC_PERM);
	if (!verts)
		return false;
	
	int nv = 0;
	
	// Copy contour A.
	for (int i = 0; i <= ca.nverts; ++i)
	{
		int* dst = &verts[nv*4];
		const int* src = &ca.verts[((ia+i)%ca.nverts)*4];
		dst[0] = src[0];
		dst[1] = src[1];
		dst[2] = src[2];
		dst[3] = src[3];
		nv++;
	}

	// Copy contour B
	for (int i = 0; i <= cb.nverts; ++i)
	{
		int* dst = &verts[nv*4];
		const int* src = &cb.verts[((ib+i)%cb.nverts)*4];
		dst[0] = src[0];
		dst[1] = src[1];
		dst[2] = src[2];
		dst[3] = src[3];
		nv++;
	}
	
	rcFree(ca.verts);
	ca.verts = verts;
	ca.nverts = nv;
	
	rcFree(cb.verts);
	cb.verts = 0;
	cb.nverts = 0;
	
	return true;
}

struct rcContourHole
{
	rcContour* contour;
	int minx, minz, leftmost;
};

struct rcContourRegion
{
	rcContour* outline;
	rcContourHole* holes;
	int nholes;
};

struct rcPotentialDiagonal
{
	int vert;
	int dist;
};

// Finds the lowest leftmost vertex of a contour.
static void findLeftMostVertex(rcContour* contour, int* minx, int* minz, int* leftmost)
{
	*minx = contour->verts[0];
	*minz = contour->verts[2];
	*leftmost = 0;
	for (int i = 1; i < contour->nverts; i++)
	{
		const int x = contour->verts[i*4+0];
		const int z = contour->verts[i*4+2];
		if (x < *minx || (x == *minx && z < *minz))
		{
			*minx = x;
			*minz = z;
			*leftmost = i;
		}
	}
}

static int compareHoles(const void* va, const void* vb)
{
	const rcContourHole* a = (const rcContourHole*)va;
	const rcContourHole* b = (const rcContourHole*)vb;
	if (a->minx == b->minx)
	{
		if (a->minz < b->minz)
			return -1;
		if (a->minz > b->minz)
			return 1;
	}
	else
	{
		if (a->minx < b->minx)
			return -1;
		if (a->minx > b->minx)
			return 1;
	}
	return 0;
}


static int compareDiagDist(const void* va, const void* vb)
{
	const rcPotentialDiagonal* a = (const rcPotentialDiagonal*)va;
	const rcPotentialDiagonal* b = (const rcPotentialDiagonal*)vb;
	if (a->dist < b->dist)
		return -1;
	if (a->dist > b->dist)
		return 1;
	return 0;
}


static void mergeRegionHoles(rcContext* ctx, rcContourRegion& region)
{
	// Sort holes from left to right.
	for (int i = 0; i < region.nholes; i++)
		findLeftMostVertex(region.holes[i].contour, &region.holes[i].minx, &region.holes[i].minz, &region.holes[i].leftmost);
	
	qsort(region.holes, region.nholes, sizeof(rcContourHole), compareHoles);
	
	int maxVerts = region.outline->nverts;
	for (int i = 0; i < region.nholes; i++)
		maxVerts += region.holes[i].contour->nverts;
	
	rcScopedDelete<rcPotentialDiagonal> diags((rcPotentialDiagonal*)rcAlloc(sizeof(rcPotentialDiagonal)*maxVerts, RC_ALLOC_TEMP));
	if (!diags)
	{
		ctx->log(RC_LOG_WARNING, "mergeRegionHoles: Failed to allocated diags %d.", maxVerts);
		return;
	}
	
	rcContour* outline = region.outline;
	
	// Merge holes into the outline one by one.
	for (int i = 0; i < region.nholes; i++)
	{
		rcContour* hole = region.holes[i].contour;
		
		int index = -1;
		int bestVertex = region.holes[i].leftmost;
		for (int iter = 0; iter < hole->nverts; iter++)
		{
			// Find potential diagonals.
			// The 'best' vertex must be in the cone described by 3 cosequtive vertices of the outline.
			// ..o j-1
			//   |
			//   |   * best
			//   |
			// j o-----o j+1
			//         :
			int ndiags = 0;
			const int* corner = &hole->verts[bestVertex*4];
			for (int j = 0; j < outline->nverts; j++)
			{
				if (inCone(j, outline->nverts, outline->verts, corner))
				{
					int dx = outline->verts[j*4+0] - corner[0];
					int dz = outline->verts[j*4+2] - corner[2];
					diags[ndiags].vert = j;
					diags[ndiags].dist = dx*dx + dz*dz;
					ndiags++;
				}
			}
			// Sort potential diagonals by distance, we want to make the connection as short as possible.
			qsort(diags, ndiags, sizeof(rcPotentialDiagonal), compareDiagDist);
			
			// Find a diagonal that is not intersecting the outline not the remaining holes.
			index = -1;
			for (int j = 0; j < ndiags; j++)
			{
				const int* pt = &outline->verts[diags[j].vert*4];
				bool intersect = intersectSegCountour(pt, corner, diags[i].vert, outline->nverts, outline->verts);
				for (int k = i; k < region.nholes && !intersect; k++)
					intersect |= intersectSegCountour(pt, corner, -1, region.holes[k].contour->nverts, region.holes[k].contour->verts);
				if (!intersect)
				{
					index = diags[j].vert;
					break;
				}
			}
			// If found non-intersecting diagonal, stop looking.
			if (index != -1)
				break;
			// All the potential diagonals for the current vertex were intersecting, try next vertex.
			bestVertex = (bestVertex + 1) % hole->nverts;
		}
		
		if (index == -1)
		{
			ctx->log(RC_LOG_WARNING, "mergeHoles: Failed to find merge points for %p and %p.", region.outline, hole);
			continue;
		}
		if (!mergeContours(*region.outline, *hole, index, bestVertex))
		{
			ctx->log(RC_LOG_WARNING, "mergeHoles: Failed to merge contours %p and %p.", region.outline, hole);
			continue;
		}
	}
}


/// @par
///
/// The raw contours will match the region outlines exactly. The @p maxError and @p maxEdgeLen
/// parameters control how closely the simplified contours will match the raw contours.
///
/// Simplified contours are generated such that the vertices for portals between areas match up.
/// (They are considered mandatory vertices.)
///
/// Setting @p maxEdgeLength to zero will disabled the edge length feature.
///
/// See the #rcConfig documentation for more information on the configuration parameters.
///
/// @see rcAllocContourSet, rcCompactHeightfield, rcContourSet, rcConfig
/// 构建轮廓。原始的轮廓会恰好覆盖地区的轮廓线。maxError和maxEdgeLen参数控制了简化的轮廓如何接近原始的轮廓。
bool rcBuildContours(rcContext* ctx, rcCompactHeightfield& chf,
					 const float maxError, const int maxEdgeLen,
					 rcContourSet& cset, const int buildFlags)
{
	// 注：如果这个OpenSpan的一个方向没有邻接OpenSpan，或者这个OpenSpan的地区ID和其这个方向的邻接OpenSpan的地区ID不同（或者这个方向的邻接OpenSpan没有地区ID），则这个OpenSpan的这个方向的边界叫做地区边界。
	//     同一地区的OpenSpan之间的边界叫做内部边界。

	rcAssert(ctx);
	
	const int w = chf.width;
	const int h = chf.height;
	const int borderSize = chf.borderSize;
	
	rcScopedTimer timer(ctx, RC_TIMER_BUILD_CONTOURS);
	
	rcVcopy(cset.bmin, chf.bmin);
	rcVcopy(cset.bmax, chf.bmax);
	if (borderSize > 0)
	{
		// If the heightfield was build with bordersize, remove the offset.
		// 如果设置了边界，就将包围盒缩小一些
		const float pad = borderSize*chf.cs;
		cset.bmin[0] += pad;
		cset.bmin[2] += pad;
		cset.bmax[0] -= pad;
		cset.bmax[2] -= pad;
	}
	cset.cs = chf.cs;
	cset.ch = chf.ch;
	cset.width = chf.width - chf.borderSize*2;
	cset.height = chf.height - chf.borderSize*2;
	cset.borderSize = chf.borderSize;
	cset.maxError = maxError;
	
	int maxContours = rcMax((int)chf.maxRegions, 8);
	cset.conts = (rcContour*)rcAlloc(sizeof(rcContour)*maxContours, RC_ALLOC_PERM);
	if (!cset.conts)
		return false;
	cset.nconts = 0;
	
	rcScopedDelete<unsigned char> flags((unsigned char*)rcAlloc(sizeof(unsigned char)*chf.spanCount, RC_ALLOC_TEMP));
	if (!flags)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'flags' (%d).", chf.spanCount);
		return false;
	}
	
	ctx->startTimer(RC_TIMER_BUILD_CONTOURS_TRACE);
	
	// Mark boundaries.
	// 记录有地区ID的OpenSpan的4个边的边界类型，保存到flags数组中。
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				unsigned char res = 0;
				const rcCompactSpan& s = chf.spans[i];

				//没有地区ID或者是BORDER地区，略过
				if (!chf.spans[i].reg || (chf.spans[i].reg & RC_BORDER_REG))
				{
					flags[i] = 0;
					continue;
				}

				//得到当前OpenSpan与四个轴邻接OpenSpan的边界类型。
				for (int dir = 0; dir < 4; ++dir)
				{
					unsigned short r = 0;    //邻接OpenSpan的区域id
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, dir);
						r = chf.spans[ai].reg;
					}

					// 注：如果4个边界都是内部边界，则res为0x0f，二进制为0000 1111
					if (r == chf.spans[i].reg)
						res |= (1 << dir);
				}

				//异或，得到地区边界标记
				//比如：如果只有2方向的邻接OpenSpan是不同的地区，则flags为，0000 0100（二进制）
				//如果flags为0000 0000（二进制），则表示4个都是内部边界
				//如果flags为0000 1111（二进制），4个都是地区边界，可能吗？因为前面生成地区时，已经处理小的地区了。
				flags[i] = res ^ 0xf; // Inverse, mark non connected edges.
			}
		}
	}
	
	ctx->stopTimer(RC_TIMER_BUILD_CONTOURS_TRACE);
	
	rcIntArray verts(256);
	rcIntArray simplified(64);
	
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				//没有地区边界（或者都是内部边界）略过。
				if (flags[i] == 0 || flags[i] == 0xf)
				{
					flags[i] = 0;
					continue;
				}

				//没有地区ID或者是BORDER地区，略过
				const unsigned short reg = chf.spans[i].reg;
				if (!reg || (reg & RC_BORDER_REG))
					continue;
				const unsigned char area = chf.areas[i];
				
				verts.resize(0);
				simplified.resize(0);
				
				// 找到轮廓上的点
				ctx->startTimer(RC_TIMER_BUILD_CONTOURS_TRACE);
				walkContour(x, y, i, chf, flags, verts);
				ctx->stopTimer(RC_TIMER_BUILD_CONTOURS_TRACE);
				
				// 简化轮廓
				ctx->startTimer(RC_TIMER_BUILD_CONTOURS_SIMPLIFY);
				simplifyContour(verts, simplified, maxError, maxEdgeLen, buildFlags);

				// 删除在xz平面上相等的相邻顶点，否则三角形将变得混乱。
				removeDegenerateSegments(simplified);
				ctx->stopTimer(RC_TIMER_BUILD_CONTOURS_SIMPLIFY);
				
				
				// Store region->contour remap info.
				// Create contour.
				if (simplified.size()/4 >= 3)
				{
					if (cset.nconts >= maxContours)
					{
						// Allocate more contours.
						// This happens when a region has holes.
						const int oldMax = maxContours;
						maxContours *= 2;
						rcContour* newConts = (rcContour*)rcAlloc(sizeof(rcContour)*maxContours, RC_ALLOC_PERM);
						for (int j = 0; j < cset.nconts; ++j)
						{
							newConts[j] = cset.conts[j];
							// Reset source pointers to prevent data deletion.
							cset.conts[j].verts = 0;
							cset.conts[j].rverts = 0;
						}
						rcFree(cset.conts);
						cset.conts = newConts;
						
						ctx->log(RC_LOG_WARNING, "rcBuildContours: Expanding max contours from %d to %d.", oldMax, maxContours);
					}
					
					rcContour* cont = &cset.conts[cset.nconts++];
					
					cont->nverts = simplified.size()/4;
					cont->verts = (int*)rcAlloc(sizeof(int)*cont->nverts*4, RC_ALLOC_PERM);
					if (!cont->verts)
					{
						ctx->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'verts' (%d).", cont->nverts);
						return false;
					}
					memcpy(cont->verts, &simplified[0], sizeof(int)*cont->nverts*4);
					if (borderSize > 0)
					{
						// If the heightfield was build with bordersize, remove the offset.
						for (int j = 0; j < cont->nverts; ++j)
						{
							int* v = &cont->verts[j*4];
							v[0] -= borderSize;
							v[2] -= borderSize;
						}
					}
					
					cont->nrverts = verts.size()/4;
					cont->rverts = (int*)rcAlloc(sizeof(int)*cont->nrverts*4, RC_ALLOC_PERM);
					if (!cont->rverts)
					{
						ctx->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'rverts' (%d).", cont->nrverts);
						return false;
					}
					memcpy(cont->rverts, &verts[0], sizeof(int)*cont->nrverts*4);
					if (borderSize > 0)
					{
						// If the heightfield was build with bordersize, remove the offset.
						for (int j = 0; j < cont->nrverts; ++j)
						{
							int* v = &cont->rverts[j*4];
							v[0] -= borderSize;
							v[2] -= borderSize;
						}
					}
					
					cont->reg = reg;
					cont->area = area;
				}
			}
		}
	}
	
	// Merge holes if needed.
	// 如果需要，合并孔。
	if (cset.nconts > 0)
	{
		// Calculate winding of all polygons.
		// 计算所有多边形的缠绕。
		rcScopedDelete<char> winding((char*)rcAlloc(sizeof(char)*cset.nconts, RC_ALLOC_TEMP));
		if (!winding)
		{
			ctx->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'hole' (%d).", cset.nconts);
			return false;
		}
		int nholes = 0;
		for (int i = 0; i < cset.nconts; ++i)
		{
			rcContour& cont = cset.conts[i];
			// If the contour is wound backwards, it is a hole.
			// 如果轮廓向后缠绕，则它是一个孔。
			winding[i] = calcAreaOfPolygon2D(cont.verts, cont.nverts) < 0 ? -1 : 1;
			if (winding[i] < 0)
				nholes++;
		}
		
		if (nholes > 0)
		{
			// Collect outline contour and holes contours per region.
			// We assume that there is one outline and multiple holes.
			const int nregions = chf.maxRegions+1;
			rcScopedDelete<rcContourRegion> regions((rcContourRegion*)rcAlloc(sizeof(rcContourRegion)*nregions, RC_ALLOC_TEMP));
			if (!regions)
			{
				ctx->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'regions' (%d).", nregions);
				return false;
			}
			memset(regions, 0, sizeof(rcContourRegion)*nregions);
			
			rcScopedDelete<rcContourHole> holes((rcContourHole*)rcAlloc(sizeof(rcContourHole)*cset.nconts, RC_ALLOC_TEMP));
			if (!holes)
			{
				ctx->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'holes' (%d).", cset.nconts);
				return false;
			}
			memset(holes, 0, sizeof(rcContourHole)*cset.nconts);
			
			for (int i = 0; i < cset.nconts; ++i)
			{
				rcContour& cont = cset.conts[i];
				// Positively would contours are outlines, negative holes.
				if (winding[i] > 0)
				{
					if (regions[cont.reg].outline)
						ctx->log(RC_LOG_ERROR, "rcBuildContours: Multiple outlines for region %d.", cont.reg);
					regions[cont.reg].outline = &cont;
				}
				else
				{
					regions[cont.reg].nholes++;
				}
			}
			int index = 0;
			for (int i = 0; i < nregions; i++)
			{
				if (regions[i].nholes > 0)
				{
					regions[i].holes = &holes[index];
					index += regions[i].nholes;
					regions[i].nholes = 0;
				}
			}
			for (int i = 0; i < cset.nconts; ++i)
			{
				rcContour& cont = cset.conts[i];
				rcContourRegion& reg = regions[cont.reg];
				if (winding[i] < 0)
					reg.holes[reg.nholes++].contour = &cont;
			}
			
			// Finally merge each regions holes into the outline.
			for (int i = 0; i < nregions; i++)
			{
				rcContourRegion& reg = regions[i];
				if (!reg.nholes) continue;
				
				if (reg.outline)
				{
					mergeRegionHoles(ctx, reg);
				}
				else
				{
					// The region does not have an outline.
					// This can happen if the contour becaomes selfoverlapping because of
					// too aggressive simplification settings.
					ctx->log(RC_LOG_ERROR, "rcBuildContours: Bad outline for region %d, contour simplification is likely too aggressive.", i);
				}
			}
		}
		
	}
	
	return true;
}
