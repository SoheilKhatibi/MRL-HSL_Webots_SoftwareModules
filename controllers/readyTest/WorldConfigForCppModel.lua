-- Walk Parameters
walk = {};
----------------------------------------------
-- Stance and velocity limit values
----------------------------------------------
walk.stanceLimitX={-0.16,0.16};
walk.stanceLimitY={0.07,0.20};
walk.stanceLimitA={-10*math.pi/180,30*math.pi/180};
walk.velLimitX={-.04,.08};
walk.velLimitY={-.03,.03};
walk.velLimitA={-.3,.3};
walk.velDelta={0.02,0.02,0.15} 

walk.velXHigh = 0.10;
walk.velDeltaXHigh = 0.005;

walk.vaFactor = 0.6;


walk.footSizeX = {-0.05, 0.05};
walk.stanceLimitMarginY = 0.015;

return walk
