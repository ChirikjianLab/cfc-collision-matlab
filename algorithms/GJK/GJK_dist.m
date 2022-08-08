%This function is based on the book Real-Time Collision Detection
% (http://realtimecollisiondetection.net/)
%
%It computes the distance, the points of closest proximity points and also
%returns the points last contained in the simplex.
%[dist,pts,G,H] = GJK_dist_7_point_poly( shape1, shape2 )
%
% INPUTS:
%
% shape1:   a patch object representing the first convex solid
% shape2:   a patch object representing the second convex solid
%
% OUTPUTS:
%
% dist:     an absolute float number representing the minimum distance between
%           the shapes.
% pts:      The points of the minkowsky diference where the alogrithm
%           stoped.
% G:        The point on shape1 closest to shape2
% H:        The point on shape2 closest to shape1
% updated 05/01/17. Thanks to Amin Ramezanifar for pointing out an error.
function [dist,pts,G,H] = GJK_dist( shape1, shape2 )
%pick a random direction
d=[1 1 1];
%pick a point
[a, point1_a, point2_a]=support_2(shape1,shape2,d);
%pick a second point to creat a line
[b, point1_b, point2_b]=support_2(shape1,shape2,-d);
%pick a third point to make a tirangle
ab=b-a;
ao=-a;
%perpendicular direction
d=cross(cross(ab,ao),ab);
%pick a third point
[c, point1_c, point2_c]=support_2(shape1,shape2,d);
%the simplex is now complete
pts=[a b c];
pts_1_2=[point1_a point1_b point1_c;point2_a point2_b point2_c];
itt=0;
pts_old=pts;
norm_old=norm(pts_old);
d_old=d;
dist_old=1000;
dist=1000;
flag=0;
PP0 = 'V';
%This loop is there for the very rare case where the simplex would contain
%duplicate points.
while true
    if isequal(c,b) || isequal(c,a)
        [c, point1_c, point2_c]=support_2(shape1,shape2,-d);
    else
        pts=[a b c];
        pts_1_2=[point1_a point1_b point1_c;point2_a point2_b point2_c];
        break
    end
end
while true
    
    %we compute the norm of vector going from the origin to each member
    %of the simplex
    Norm_pts = (sqrt(sum(abs(pts').^2,2)));
    %Computing and removing the farthest point of the simplex from the origin
    [max_norm,index]=max(Norm_pts);
    %new search direction
    d=cross(pts(:,2)-pts(:,1),pts(:,3)-pts(:,1));
    origin_projected=(pts(:,1)'*d)*d;
    if d'*origin_projected>0
        d=-d;
    end
    d=d/norm(d);
    
    
    %Pciking a new point to add to the simplex.
    [c, point1_c, point2_c]=support_2(shape1,shape2,d);
    %we project the origin on the simplex's plane.
    origin_projected=(pts(:,1)'*d)*d;
    % We verify if the point is allready contained in the simplex.
    % The (itt>5 && flag == 5) condition to enter this loop is one of the main
    % problems of the algorithm. Sometime, when using only the normal to
    % the triangle, the solution does not converge to the absolute minimum
    % distance. This other way of computing the new direction allows a
    % higher rate of succesful convergeance. If one would want to upgrade the
    % algorithm he would certainly need to work arround here.
     if any(abs(Norm_pts-norm(c))<0.000001) || (itt>5 && flag == 5)
         % computing the distance of each traingle lines from the origin
        d1 = distLinSeg(pts(:,1)',pts(:,2)',origin_projected',origin_projected');
        d2 = distLinSeg(pts(:,1)',pts(:,3)',origin_projected',origin_projected');
        d3 = distLinSeg(pts(:,2)',pts(:,3)',origin_projected',origin_projected');
%         d1=(pts(:,2)-pts(:,1))'/norm(pts(:,2)-pts(:,1))*(pts(:,1)-origin_projected)/norm((pts(:,1)-origin_projected));
%         d2=(pts(:,3)-pts(:,1))'/norm(pts(:,3)-pts(:,1))*(pts(:,1)-origin_projected)/norm((pts(:,1)-origin_projected));
%         d3=(pts(:,3)-pts(:,2))'/norm(pts(:,3)-pts(:,2))*(pts(:,2)-origin_projected)/norm((pts(:,2)-origin_projected));
        dist_seg_min=min(abs([d1 d2 d3]));
        %we want to pick the two points creating the line closest to the
        %origin.
        %We also need to deal with the case where the origin is closest to
        %a point, resulting in two distances being equal.
        if dist_seg_min<dist
            while true
                if d1==dist_seg_min
                    %pick a third point to make a tirangle
                    ab=pts(:,2)-pts(:,1);
                    ao=-pts(:,1);
                    %perpendicular direction
                    d=cross(cross(ab,ao),ab);
                    if d1~=d2 && d1~=d3
                        [c, point1_c, point2_c]=support_2(shape1,shape2,d);
                        index=3;
                        break
                    else
                        [c_1, point1_c_1, point2_c_1]=support_2(shape1,shape2,d);
                        index_1=3;
                    end
                end
                if d2==dist_seg_min
                    %pick a third point to make a tirangle
                    ab=pts(:,3)-pts(:,1);
                    ao=-pts(:,1);
                    %perpendicular direction
                    d=cross(cross(ab,ao),ab);
                    if d2~=d1 && d2~=d3
                        [c, point1_c, point2_c]=support_2(shape1,shape2,d);
                        index=2;
                        break
                    else
                        [c_2, point1_c_2, point2_c_2]=support_2(shape1,shape2,d);
                        index_2=2;
                    end
                end
                if d3==dist_seg_min
                    %pick a third point to make a tirangle
                    ab=pts(:,3)-pts(:,2);
                    ao=-pts(:,2);
                    %perpendicular direction
                    d=cross(cross(ab,ao),ab);
                    if d3~=d1 && d3~=d2
                        [c, point1_c, point2_c]=support_2(shape1,shape2,d);
                        index=1;
                        break
                    else
                        [c_3, point1_c_3, point2_c_3]=support_2(shape1,shape2,d);
                        index_3=1;
                    end
                    
                end
                if d1==d2
                    L1=(pts(:,2)-pts(:,1))'/norm(pts(:,2)-pts(:,1))*(pts(:,1)-origin_projected)/norm((pts(:,1)-origin_projected));
                    L2=(pts(:,3)-pts(:,1))'/norm(pts(:,3)-pts(:,1))*(pts(:,1)-origin_projected)/norm((pts(:,1)-origin_projected));
                    if L1>L2
                        c=c_1;
                        point1_c=point1_c_1;
                        point2_c=point2_c_1;
                    else
                        c=c_2;
                        point1_c=point1_c_2;
                        point2_c=point2_c_2;
                    end
                elseif d1==d3
                    L1=(pts(:,2)-pts(:,1))'/norm(pts(:,2)-pts(:,1))*(pts(:,1)-origin_projected)/norm((pts(:,1)-origin_projected));
                    L3=(pts(:,3)-pts(:,2))'/norm(pts(:,3)-pts(:,2))*(pts(:,2)-origin_projected)/norm((pts(:,2)-origin_projected));
                    if L1>L3
                        c=c_1;
                        point1_c=point1_c_1;
                        point2_c=point2_c_1;
                    else
                        c=c_3;
                        point1_c=point1_c_3;
                        point2_c=point2_c_3;
                    end
                elseif d2==d3
                    L2=(pts(:,3)-pts(:,1))'/norm(pts(:,3)-pts(:,1))*(pts(:,1)-origin_projected)/norm((pts(:,1)-origin_projected));
                    L3=(pts(:,3)-pts(:,2))'/norm(pts(:,3)-pts(:,2))*(pts(:,2)-origin_projected)/norm((pts(:,2)-origin_projected));
                    if L2>L3
                        c=c_2;
                        point1_c=point1_c_2;
                        point2_c=point2_c_2;
                    else
                        c=c_3;
                        point1_c=point1_c_3;
                        point2_c=point2_c_3;
                    end
                end
                break
            end
        end
        % if even after picking a new point with the other method, if the
        % simplex already has it, we assume we converged.
        if any(abs(Norm_pts-norm(c))<0.000001)
            if PP0 == 'V'
                % converged on first itteration, dist and PP0 must be
                % computed.
                [dist, PP0] = pointTriangleDistance(pts',[0 0 0]);
            end
            Norm_pts=0;
            break
        end
     end
    
    pts(:,index)=[];
    pts_1_2(:,index)=[];
    
    pts=[pts c];
    pts_1_2=[pts_1_2 [point1_c;point2_c]];
    [dist, PP0] = pointTriangleDistance(pts',[0 0 0]);
    %we need to do the following manipulation because sometimes, the
    %simplex will oscillate arround the closest solution, creating an
    %infinite loop. We need to backup the solution that resulted in the
    %minimal distance and use it if the loop times out (10 itteration max)
    if dist<dist_old
        pts_prox=pts;
        dist_prox=dist;
        PP0_prox=PP0;
        pts_1_2_prox=pts_1_2;
    end
    
    %The time out is set here.
    if itt>6
        flag=5;
        if itt>10 && flag==5
            pts=pts_prox;
            dist=dist_prox;
            PP0=PP0_prox;
            pts_1_2=pts_1_2_prox;
            break
        end
        
    end
    dist_old=dist;
    d_old=d;
    pts_old=pts;
    if abs(norm_old-norm(pts))<0.01
        norm_old=norm(pts);
    end
    itt=itt+1;
    
%         pause(0.1)
end
% Calculation of the barycentric coordinates of the origin in respect to
% the simplex.
 T = [1	 4  3;
      1  4  2;
      3	 4  2;
      1  2  3];
  pts_t=[pts';PP0];
  TR=triangulation(T,pts_t);
  B=cartesianToBarycentric(TR,4,PP0);
  G=0;
  H=0;
  %calculation of the points G and H of proximity on,respectively, shape1
  %and shape2.
  for i=1:3
      G=G+B(i)*pts_1_2(1:3,i);
      H=H+B(i)*pts_1_2(4:6,i);
  end
end

function point = getFarthestInDir(shape, v)
%Find the furthest point in a given direction for a shape
XData = get(shape,'XData'); % Making it more compatible with previous MATLAB releases.
YData = get(shape,'YData');
ZData = get(shape,'ZData');
dotted = XData*v(1) + YData*v(2) + ZData*v(3);
[maxInCol,rowIdxSet] = max(dotted);
[maxInRow,colIdx] = max(maxInCol);
rowIdx = rowIdxSet(colIdx);
point = [shape.XData(rowIdx,colIdx), shape.YData(rowIdx,colIdx), shape.ZData(rowIdx,colIdx)];
end

function [point,point1 ,point2] = support_2(shape1,shape2,v)
%Support function to get the Minkowski difference.
point1 = getFarthestInDir(shape1, v)';
point2 = getFarthestInDir(shape2, -v)';
point = point1 - point2;
end
