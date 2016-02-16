function min_d = calcMinDistance(L1,L2)

% surface = @(p1,p2,p3) 0.5*abs( p1(1)*(p3(2) - p2(2)) + ...
%                                p2(1)*(p1(2) - p3(2)) + ...
%                                p3(1)*(p2(2) - p1(2)) );

p1 = L1(1,:)
p2 = L1(2,:)
p3 = L2(1,:)
p4 = L2(2,:)

d1 = min_distance_point_to_link(p1,L2)
d2 = min_distance_point_to_link(p2,L2)
d3 = min_distance_point_to_link(p3,L1)
d4 = min_distance_point_to_link(p4,L1)

min_d = min([d1,d2,d3,d4]);

    function d = min_distance_point_to_link(p,L)
        % check p1 with p3 and p4
        ang1 = acos(dot(p-L(1,:),L(2,:)-L(1,:))/(norm(p-L(1,:))*norm(L(2,:)-L(1,:))));
        ang2 = acos(dot(p-L(2,:),L(1,:)-L(2,:))/(norm(p-L(2,:))*norm(L(1,:)-L(2,:))));
        
        if (ang1>pi/2)
            d = norm(p-L(1,:));
        elseif ang2>pi/2
            d = norm(p-L(2,:));
        else
            d = abs( p(1)*(L(2,2) - L(1,2)) + ...
                         L(1,1)*(p(2) - L(2,2)) + ...
                         L(2,1)*(L(1,2) - p(2)) )...
                /norm(L(1,:)-L(2,:));
        end
    end

end