% FLOATCURVEGEN Generate a polynomial identical to Unity's floatCurves using the keys arranged into an input matrix.
%
%   The polynomial produced is a cubic hermite spline.
%
%   Input is a matrix consisting of 5 column vectors.
%   The first and second columns are the x,y coordinates of the points.
%   The third and fourth columns are the first derivatives of the cubic
%   polynomials at the left and right side of the points.
%   
%   In cases where the first derivatives are specified in the part's .cfg
%   file, the fifth column should be set to 1.  In cases where the first
%   derivatives are left unspecified (the provided keys only consist of x
%   and y coordinates), the fifth column should be set to zero.  
%
%   If the first derivative is unspecified, it is assumed to be continuous
%   and equal to the average of the secants between the point and the
%   preceding point and the point and the next point.
%
%   Author: Charlie_Zulu
%   2016-07-16

%horrendously un-optimized, sorry about that.
%since the first derivatives are specified, we can create a cubic function
%one interval at a time and combine to form the overall spline.

function [pp] = floatCurveGen(input)

%find the number of rows (points)
s=size(input);
n=s(1);

%find tangents for all points; incrementing inc.
%in cases where input(inc,5)is true, the input(inc,3) and input(inc,4) entries can differ; otherwise, they're the same.
for inc=1:n
    if input(inc,5) == 1 %if tangent is defined, nothing changes.
    elseif inc==1 %left endpoint special case
        input(inc,3)=(input((inc+1),2)-input(inc,2))/(input((inc+1),1)-input(inc,1));
        input(inc,4)=input(inc,3);
    elseif inc==n %right endpoint special case
        input(inc,3)=(input(inc,2)-input((inc-1),2))/(input(inc,1)-input((inc-1),1));
        input(inc,4)=input(inc,3);
    else % tangent is the average of the slopes to either side
        input(inc,3)=1/2*((input(inc,2)-input((inc-1),2))/(input(inc,1)-input((inc-1),1))+(input((inc+1),2)-input(inc,2))/(input((inc+1),1)-input(inc,1)));
        input(inc,4)=input(inc,3);
    end
end

%create initial values of pp outside of the loop
ppcoefs=zeros((n-1),4); %n-1-by-4 matrix, since it has 4 coefficients per interval.
ppbreaks=zeros(1,n);    %n breaks

%create splines for each interval, combine with pp
for inc=1:(n-1)
    %left endpoint = inc, right endpoint = inc+1
    %([lx,rx],[ly,ry],[ltan,rtan])
    ppinc = pwch(input([inc,(inc+1)],1)',input([inc,(inc+1)],2)',[input(inc,4),input((inc+1),3)]);  %there's a better way to calculate the coefs, but I'm too lazy to work it out.
    %combining with pp
    ppcoefs(inc,:)=ppinc.coefs;
    ppbreaks([inc,(inc+1)])=ppinc.breaks;
end
%creating pp
pp=mkpp(ppbreaks,ppcoefs);
end