#include "DetectionGrouping.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace tracking {

bool velocitiesAgree(const Detection & a, const Detection & b, const GroupingParams & p){
	const bool aDir = a.hasVelocity && a.speed() >= p.minSpeed;
	const bool bDir = b.hasVelocity && b.speed() >= p.minSpeed;
	if(!aDir || !bDir) return true; // a directionless blob can join anything
	const float dot = a.vx * b.vx + a.vy * b.vy;
	const float cosang = dot / (a.speed() * b.speed());
	const float ang = std::acos(std::clamp(cosang, -1.0f, 1.0f)) * 180.0f / static_cast<float>(M_PI);
	if(ang > p.angleDeg) return false;
	const float ratio = std::max(a.speed(), b.speed()) / std::max(1e-6f, std::min(a.speed(), b.speed()));
	return ratio <= p.speedRatio;
}

float gapX(float ax0, float ax1, float bx0, float bx1){
	return std::max(0.0f, std::max(bx0 - ax1, ax0 - bx1));
}

float lateralOverlapFrac(float ay0, float ay1, float by0, float by1){
	const float inter = std::min(ay1, by1) - std::max(ay0, by0);
	const float minH = std::max(1e-6f, std::min(ay1 - ay0, by1 - by0));
	return inter <= 0 ? 0.0f : inter / minH;
}

Detection unionDetections(const Detection & a, const Detection & b){
	Detection u;
	u.x0 = std::min(a.x0, b.x0);
	u.x1 = std::max(a.x1, b.x1);
	u.y0 = std::min(a.y0, b.y0);
	u.y1 = std::max(a.y1, b.y1);
	const float wa = std::max(a.area, 1e-6f), wb = std::max(b.area, 1e-6f);
	const bool aV = a.hasVelocity, bV = b.hasVelocity;
	if(aV && bV){
		u.vx = (a.vx * wa + b.vx * wb) / (wa + wb);
		u.vy = (a.vy * wa + b.vy * wb) / (wa + wb);
		u.coherence = (a.coherence * wa + b.coherence * wb) / (wa + wb);
		u.hasVelocity = true;
	}else if(aV || bV){
		const Detection & v = aV ? a : b;
		u.vx = v.vx;
		u.vy = v.vy;
		u.coherence = v.coherence;
		u.hasVelocity = true;
	}
	u.area = a.area + b.area;
	u.confidence = std::max(a.confidence, b.confidence);
	u.touchesLeft = a.touchesLeft || b.touchesLeft;
	u.touchesRight = a.touchesRight || b.touchesRight;
	u.lane = a.lane == b.lane ? a.lane : -1;
	u.label = a.label.empty() ? b.label : a.label;
	return u;
}

std::vector<Detection> groupDetections(const std::vector<Detection> & in, const GroupingParams & p){
	const size_t n = in.size();
	if(n < 2) return in;

	std::vector<int> parent(n);
	std::iota(parent.begin(), parent.end(), 0);
	const auto find = [&](int i){
		while(parent[i] != i){
			parent[i] = parent[parent[i]];
			i = parent[i];
		}
		return i;
	};
	const auto unite = [&](int a, int b){
		a = find(a);
		b = find(b);
		if(a != b) parent[b] = a;
	};

	// Pairwise test on the original blobs; transitive closure through the
	// union-find gives chains (wagon -> wagon -> wagon).
	for(size_t i = 0; i < n; i++){
		for(size_t j = i + 1; j < n; j++){
			const Detection & a = in[i];
			const Detection & b = in[j];
			// y is lane-local: two rectified lanes of the same corridor
			// place the object at unrelated heights, so the lateral test
			// only applies within one lane
			const bool sameLane = a.lane < 0 || b.lane < 0 || a.lane == b.lane;
			if(sameLane && lateralOverlapFrac(a.y0, a.y1, b.y0, b.y1) < p.lateralOverlap) continue;
			if(gapX(a.x0, a.x1, b.x0, b.x1) > p.gapFrac) continue;
			if(!velocitiesAgree(a, b, p)) continue;
			unite(static_cast<int>(i), static_cast<int>(j));
		}
	}

	std::vector<Detection> out;
	std::vector<int> rootIndex(n, -1);
	for(size_t i = 0; i < n; i++){
		const int r = find(static_cast<int>(i));
		if(rootIndex[r] < 0){
			rootIndex[r] = static_cast<int>(out.size());
			out.push_back(in[i]);
		}else{
			out[rootIndex[r]] = unionDetections(out[rootIndex[r]], in[i]);
		}
	}
	std::sort(out.begin(), out.end(), [](const Detection & a, const Detection & b){
		return a.area > b.area;
	});
	return out;
}

} // namespace tracking
