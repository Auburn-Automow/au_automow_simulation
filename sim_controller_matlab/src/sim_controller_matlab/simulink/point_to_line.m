function [q] = point_to_line(A,B,p)
  % returns q the closest point to p on the line segment from A to B 

  % vector from A to B
  AB = (B-A);
  % squared distance from A to B
  AB_squared = dot(AB,AB);
  if(AB_squared == 0)
    % A and B are the same point
    q = A;
  else
    % vector from A to p
    Ap = (p-A);
    % from http://stackoverflow.com/questions/849211/
    % Consider the line extending the segment, parameterized as A + t (B - A)
    % We find projection of point p onto the line.
    % It falls where t = [(p-A) . (B-A)] / |B-A|^2
    t = dot(Ap,AB)/AB_squared;
    q = A + t * AB;
    end
end
