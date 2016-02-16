 function flag = sat (rectA, rectB)
 % Implementation of Separating Axis theorem to check for intersection of two oriented quadrilaterals
 % Author: Vicky
 %rectA -> 4 rows of 2 columns each... eg. rectA = [2 1; 7 1; 7 5; 2 5]
 %rectB -> 4 rows of 2 columns each... eg. rectB = [7 1; 10 1; 10 5; 7 5]
 % The below implementation of SAT is based on the XY co-ord system, and not based on the row*column matrix indexing system of Matlab. So make sure to convert the vertices to XY co-ord system before invoking this function from MATLAB.
 % For eg. if you are using this in image processing, the pixels of the imgae would be named by (a,b) where a is the row number and b is the column number. Convert this to XY co-ordinate system by shifting the origin.
 % Contact me (vrkpt.india@gmail.com) for help/suggestions/feedback 
 
 overlapflag = zeros (181,1);
 flag = 0;
 
 for angle = 0:180
 
 % projrectA and projrectB hold the projections of each of the vertices of rectA and rectB onto the axis y=mx
 % y = mx is an axis which pases through the origin
 % (x1,y1) is the projection of the point (x0,y0) on the axis y=mx
 % x1 = (m.y0 + x0) / (m^2 + 1)
 % y1 = (m^2.y0 + m.x0) / (m^2 + 1)
  
	m = tan(degtorad(angle));	% Calculating the slope
	
	% Calculating projrectA
	for i = 1:4
		% x1
		projrectA (i,1) = (m * rectA (i,2) + rectA (i,1)) / (m^2 + 1);
		
		%y1
		projrectA (i,2) = ((m^2) * rectA (i,2) + m * rectA (i,1)) / (m^2 + 1);
	end
		
	% Calculating projrectB
	for i = 1:4
		% x1
		projrectB (i,1) = (m * rectB (i,2) + rectB (i,1)) / (m^2 + 1);
		
		%y1
		projrectB (i,2) = ((m^2) * rectB (i,2) + m * rectB (i,1)) / (m^2 + 1);
	end
	
	%Calculating the maximum and minimum X co-ord for the two rectangles
	xMinRectA = min (projrectA(:,1));
	xMaxRectA = max (projrectA(:,1));
	
	xMinRectB = min (projrectB(:,1));
	xMaxRectB = max (projrectB(:,1));
 
	if ((xMaxRectB < xMinRectA) || (xMaxRectA < xMinRectB ))
		%no overlap
		overlapflag(angle+1) = 0;
	
	else	
		overlapflag(angle+1) = 1;
	end
	
	%disp ('flag ='); overlapflag(angle+1)	
		
 end
 
 if overlapflag(:,1) == 1
	flag = 1;
 end