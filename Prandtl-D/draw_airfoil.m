function draw_airfoil()
% save airfoil centerlinex centerliney wingtipx wingtipy twistx twisty
close all;
clear all;
load airfoil;
global X;
X = [];
scalef = 0.5;
wingSpan_mm = 3750;
centerCord = 400;
tipcord = 100;
ribCnt = 21;
rootCord_mm = 400;
wingtipCord_mm = 100;
X.LeadingEdgeBeamThickness_mm = 2; % square edge size 
X.LeadingEdgeBeamWidth_mm = 5; % square edge size 
X.trailingEdgeWidth_mm = 10;

X.curvefitmethod = 'pchip';
% rootBeamGammaWidthmm = 6;
% BaseBeamWidthmm = 1; % at tip, lift is zero, we cannot have 0 beam width for misc struct weight 
assert(mod(ribCnt,2)==1); % we have one rib in the center, and be symmetrical 
rotateFromLE = 0.3; %rotation occurs at the center of [0,1] from the leading edge, so that we have the desired AOA.  
fineX = [0:0.0005:0.05 0.06:0.01:0.95 0.9505:0.0005:1]';

%%
Len_fineX = length(fineX);
X.OneSideRibCnt = (ribCnt+1)/2;
CoeffsRoot2Tip = (0:X.OneSideRibCnt-1)/(X.OneSideRibCnt-1);

%%
centerlinex = centerlinex(:);
centerliney = centerliney(:);
[centerlinex,centerliney] = normalizexy(centerlinex,centerliney);
wingtipx = wingtipx(:);
wingtipy = wingtipy(:);
[wingtipx,wingtipy] = normalizexy(wingtipx,wingtipy);
figure;
hold on;
plot(centerlinex,centerliney,'.');
axis equal;
title('centerline');grid on;
xlabel('unit');
ylabel('unit');
%%
figure;
hold on;
end_of_center_top_curve = (length(centerlinex)+1)/2;
top_centerline_x = centerlinex(1:end_of_center_top_curve);
top_centerline_y = centerliney(1:end_of_center_top_curve);
top_centerline_fine_y = interp1(top_centerline_x,top_centerline_y,fineX,X.curvefitmethod);
plot(fineX,top_centerline_fine_y,'.');
btm_centerline_x = centerlinex(end_of_center_top_curve:end);
btm_centerline_y = centerliney(end_of_center_top_curve:end);
btm_centerline_fine_y = interp1(btm_centerline_x,btm_centerline_y,fineX,X.curvefitmethod);
plot(fineX,btm_centerline_fine_y,'.');
axis equal;
legend('top','btm');
title('centerline');grid on;
xlabel('unit');
ylabel('unit');
%%
figure;
hold on;
end_of_wingtip_top_curve = (length(wingtipx)+1)/2;
top_wingtip_x = wingtipx(1:end_of_wingtip_top_curve);
top_wingtip_y = wingtipy(1:end_of_wingtip_top_curve);
top_wingtip_mid_y = interp1(top_wingtip_x,top_wingtip_y,fineX,X.curvefitmethod);
plot(fineX,top_wingtip_mid_y,'.');
btm_wingtip_x = wingtipx(end_of_wingtip_top_curve:end);
btm_wingtip_y = wingtipy(end_of_wingtip_top_curve:end);
btm_wingtip_mid_y = interp1(btm_wingtip_x,btm_wingtip_y,fineX,X.curvefitmethod);
plot(fineX,btm_wingtip_mid_y,'.');
axis equal;
legend('top','btm');
title('wingtip');
grid on;
xlabel('unit');
ylabel('unit');
%%
top_ribs = interpolate2Airfoil(top_centerline_fine_y, top_wingtip_mid_y, CoeffsRoot2Tip);
btm_ribs = interpolate2Airfoil(btm_centerline_fine_y, btm_wingtip_mid_y, CoeffsRoot2Tip);
figure;
axis equal;
hold on;
for ii = 1:X.OneSideRibCnt
    topCurve = top_ribs(:,ii);
    btmCurve = btm_ribs(:,ii);
    plot([fineX fineX],[topCurve btmCurve],'.');
end
title('Ribs:same length');
grid on;
xlabel('unit');
ylabel('unit');
%%
xindexRotate = find(fineX==rotateFromLE);
assert(length(xindexRotate)==1);
centered_interpolatedX = fineX - rotateFromLE;
Ymean = (top_ribs(xindexRotate,:) + btm_ribs(xindexRotate,:))/2;
centered_top_mid_ribs = zeros(Len_fineX,X.OneSideRibCnt);
centered_btm_mid_ribs = zeros(Len_fineX,X.OneSideRibCnt);
figure;
axis equal;
hold on;
for ii = 1:X.OneSideRibCnt
    centered_top_mid_ribs(:,ii) = top_ribs(:,ii) - Ymean(ii);
    centered_btm_mid_ribs(:,ii) = btm_ribs(:,ii) - Ymean(ii);
    plot([centered_interpolatedX; centered_interpolatedX],[centered_top_mid_ribs(:,ii); centered_btm_mid_ribs(:,ii)],'.');
end
title('Ribs:same length,shifted');
grid on;
xlabel('unit');
ylabel('unit');
%%
figure;
plot(twistx/twistx(end),twisty,'.');
title('twist');
ylabel('deg');
grid on;
xlabel('half wingspan');
%%
twist_ribs = interp1(twistx/twistx(end),twisty,CoeffsRoot2Tip,X.curvefitmethod);

rotated_ribs_top_x = zeros(Len_fineX,X.OneSideRibCnt);
rotated_ribs_top_y = zeros(Len_fineX,X.OneSideRibCnt);
rotated_ribs_btm_x = zeros(Len_fineX,X.OneSideRibCnt);
rotated_ribs_btm_y = zeros(Len_fineX,X.OneSideRibCnt);
figure;
axis equal;
hold on;
for ii = 1:X.OneSideRibCnt
    thetaDeg = twist_ribs(ii);
    R = [cosd(thetaDeg) -sind(thetaDeg); sind(thetaDeg) cosd(thetaDeg)];
    top_beforeRotate = [centered_interpolatedX centered_top_mid_ribs(:,ii)];
    top_afterRotate = top_beforeRotate*R;
    rotated_ribs_top_x(:,ii) = top_afterRotate(:,1);
    rotated_ribs_top_y(:,ii) = top_afterRotate(:,2);
    btm_beforeRotate = [centered_interpolatedX centered_btm_mid_ribs(:,ii)];
    btm_afterRotate = btm_beforeRotate*R;
    rotated_ribs_btm_x(:,ii) = btm_afterRotate(:,1);
    rotated_ribs_btm_y(:,ii) = btm_afterRotate(:,2);
    plot([rotated_ribs_top_x(:,ii); rotated_ribs_btm_x(:,ii)],[rotated_ribs_top_y(:,ii); rotated_ribs_btm_y(:,ii)],'.');
end
title('Ribs:same length,shifted,rotated');
grid on;
xlabel('unit');
ylabel('unit');
%%
scaled_ribs_top_x = rotated_ribs_top_x;
scaled_ribs_top_y = rotated_ribs_top_y;
scaled_ribs_btm_x = rotated_ribs_btm_x;
scaled_ribs_btm_y = rotated_ribs_btm_y;
scale_ribs = interp1([0 1],[rootCord_mm wingtipCord_mm],CoeffsRoot2Tip,'linear');

figure;
axis equal;
hold on;
for ii = 1:X.OneSideRibCnt
    scaled_ribs_top_x(:,ii) = scaled_ribs_top_x(:,ii) * scale_ribs(ii);
    scaled_ribs_top_y(:,ii) = scaled_ribs_top_y(:,ii) * scale_ribs(ii);
    scaled_ribs_btm_x(:,ii) = scaled_ribs_btm_x(:,ii) * scale_ribs(ii);
    scaled_ribs_btm_y(:,ii) = scaled_ribs_btm_y(:,ii) * scale_ribs(ii);
    plot([scaled_ribs_top_x(:,ii); scaled_ribs_btm_x(:,ii)],[scaled_ribs_top_y(:,ii); scaled_ribs_btm_y(:,ii)],'.');
end
title('Ribs:shifted,rotated,scaled');
grid on;
xlabel('mm');
ylabel('mm');
%%
Gamma = (1-CoeffsRoot2Tip.^2).^1.5;
figure;
plot(CoeffsRoot2Tip,Gamma,'.-');
title('Gamma, airflow circulation');
grid on;
%%
figure;
plot(CoeffsRoot2Tip,1.5*(CoeffsRoot2Tip.^2-0.5),'.-');
title('Downwash');
grid on;

%% when cutting slot, the length of x/y data may not be same any more for all ribs
% use cell to store
slotcut_ribs_top_x = {};
slotcut_ribs_top_y = {};
slotcut_ribs_btm_x = {};
slotcut_ribs_btm_y = {};
for ii = 1:X.OneSideRibCnt
    slotcut_ribs_top_x{ii} = scaled_ribs_top_x(:,ii);
    slotcut_ribs_top_y{ii} = scaled_ribs_top_y(:,ii);
    slotcut_ribs_btm_x{ii} = scaled_ribs_btm_x(:,ii);
    slotcut_ribs_btm_y{ii} = scaled_ribs_btm_y(:,ii);
end
%% cut slot for force bearing beam
[slotcut_ribs_top_x,slotcut_ribs_top_y,slotcut_ribs_btm_x,slotcut_ribs_btm_y] = cutslot...
(slotcut_ribs_top_x,slotcut_ribs_top_y,slotcut_ribs_btm_x,slotcut_ribs_btm_y,8,-10,'front');
[slotcut_ribs_top_x,slotcut_ribs_top_y,slotcut_ribs_btm_x,slotcut_ribs_btm_y] = cutslot...
(slotcut_ribs_top_x,slotcut_ribs_top_y,slotcut_ribs_btm_x,slotcut_ribs_btm_y,8,0,'rear');

%% add leading edge beam
cut_leadingedge_top_x = {};
cut_leadingedge_top_y = {};
cut_leadingedge_btm_x = {};
cut_leadingedge_btm_y = {};

figure;
axis equal;
hold on;
for ii = 1:X.OneSideRibCnt
    LE_x = min([slotcut_ribs_top_x{ii};slotcut_ribs_top_y{ii}]);
    LeadingEdgeBeamPosDetected = false;
    for xposition = LE_x:0.1:LE_x+30
        if getThickness(...
            slotcut_ribs_top_x{ii},slotcut_ribs_top_y{ii},...
            slotcut_ribs_btm_x{ii},slotcut_ribs_btm_y{ii},...
            xposition) > X.LeadingEdgeBeamThickness_mm*sqrt(2)
            LeadingEdgeBeamPosDetected = true;
            break;
        end
    end
    assert(LeadingEdgeBeamPosDetected);
    [cut_leadingedge_top_x{ii},cut_leadingedge_top_y{ii}] = removeLeadingSector(slotcut_ribs_top_x{ii},slotcut_ribs_top_y{ii},xposition);
    [cut_leadingedge_btm_x{ii},cut_leadingedge_btm_y{ii}] = removeLeadingSector(slotcut_ribs_btm_x{ii},slotcut_ribs_btm_y{ii},xposition);
    
    [cut_leadingedge_top_x{ii},cut_leadingedge_top_y{ii}] = cutLeadingEdgeSlot(cut_leadingedge_top_x{ii},cut_leadingedge_top_y{ii});
    [cut_leadingedge_btm_x{ii},cut_leadingedge_btm_y{ii}] = cutLeadingEdgeSlot(cut_leadingedge_btm_x{ii},cut_leadingedge_btm_y{ii});
    
    plot([cut_leadingedge_top_x{ii}; cut_leadingedge_btm_x{ii}],[cut_leadingedge_top_y{ii}; cut_leadingedge_btm_y{ii}],'.');
end
title('Ribs:shifted,rotated,scaled,LEcut');
grid on;
xlabel('mm');
ylabel('mm');
%% cut trailing edge
cut_trailingedge_top_x = {};
cut_trailingedge_top_y = {};
cut_trailingedge_btm_x = {};
cut_trailingedge_btm_y = {};

figure;
axis equal;
hold on;
for ii = 1:X.OneSideRibCnt
    [cut_trailingedge_top_x{ii},cut_trailingedge_top_y{ii}] = removeTrailingSector(cut_leadingedge_top_x{ii},cut_leadingedge_top_y{ii},X.trailingEdgeWidth_mm);
    [cut_trailingedge_btm_x{ii},cut_trailingedge_btm_y{ii}] = removeTrailingSector(cut_leadingedge_btm_x{ii},cut_leadingedge_btm_y{ii},X.trailingEdgeWidth_mm);
    
    plot([cut_trailingedge_top_x{ii}; cut_trailingedge_btm_x{ii}],[cut_trailingedge_top_y{ii}; cut_trailingedge_btm_y{ii}],'.');
end
title('Ribs:shifted,rotated,scaled,LEcut,TEcut');
grid on;
xlabel('mm');
ylabel('mm');
end

function [x,y] = cutLeadingEdgeSlot(x,y)
global X;
LE_x = x(1);
LE_y = y(1);
x = [LE_x+X.LeadingEdgeBeamWidth_mm;x];
y = [LE_y;y];
end

function [x,y] = removeTrailingSector(x,y,cutoffx)
    index = find(x<=max(x)-cutoffx);
    x = x(index);
    y = y(index);
end

function [x,y] = removeLeadingSector(x,y,cutoffx)
    index = find(x>=cutoffx);
    x = x(index);
    y = y(index);
end

function [x,y] = normalizexy(x,y)
    scaleF = abs(max(x)-min(x));
    x = x/scaleF;
    y = y/scaleF;
    
end

function [top_x,top_y,btm_x,btm_y] = cutslot(top_x,top_y,btm_x,btm_y,width,xposition,slotName)
global X;
figure;
axis equal;
hold on;
for ii = 1:X.OneSideRibCnt
    thickness = getThickness(...
        top_x{ii},top_y{ii},...
        btm_x{ii},btm_y{ii},...
        xposition);
    [top_x{ii},top_y{ii}] = cutnothing(top_x{ii},top_y{ii});
    [btm_x{ii},btm_y{ii}] = ...
        cutslotIn1curve(btm_x{ii},btm_y{ii},-thickness/2,width,xposition);
    plot([top_x{ii}; btm_x{ii}],[top_y{ii}; btm_y{ii}],'.');
end
title(['Ribs:shifted,rotated,scaled,slotcut' slotName]);
grid on;
end

function val = interp1nearby(x,y,newx,nearbyCnt,type)
[~,ii] = min(abs(x-newx));
if ii-nearbyCnt > 0
    leftii = ii-nearbyCnt;
else
    leftii = 1;
end
if ii+nearbyCnt <= length(x)
    rightii = ii+nearbyCnt;
else
    rightii = length(x); 
end
val = interp1(x(leftii:rightii),y(leftii:rightii),newx,type);
end

function thickness = getThickness(x1,y1,x2,y2,x)
global X;
y_value1 = interp1nearby(x1,y1,x,2,X.curvefitmethod);
y_value2 = interp1nearby(x2,y2,x,2,X.curvefitmethod);
thickness = abs(y_value1-y_value2);
end

function [cutx,cuty] = cutnothing(x,y)
cutx = x;
cuty = y;
end

function [cutx,cuty] = cutslotIn1curve(x,y,depth,width,xposition)
% x,y form the curve
% depth: depth to be cut. 
%   +: cut to Y-
%   -: cut to Y+
% width: width to be cut
% xposition: relative to xposition, cut width/2 to X+ and X-
global X;
cutx = [];
cuty = [];

LE_beamEdgeX = xposition-width/2;
TE_beamEdgeX = xposition+width/2;

indexBeforeLEcut = find(x<=LE_beamEdgeX);
cutx = x(indexBeforeLEcut);
cuty = y(indexBeforeLEcut);
y_valueLE = interp1nearby(x,y,LE_beamEdgeX,1,X.curvefitmethod);
if ismember(LE_beamEdgeX,x)
    % we don't need to insert the point and begin slot cut
else
    cutx = [cutx;LE_beamEdgeX];
    cuty = [cuty;y_valueLE];
end
cutx = [cutx;LE_beamEdgeX];
cuty = [cuty;y_valueLE - depth];
cutx = [cutx;TE_beamEdgeX];
cuty = [cuty;y_valueLE - depth];
indexAfterTEcut = find(x>=TE_beamEdgeX);
y_valueTE = interp1nearby(x,y,TE_beamEdgeX,1,X.curvefitmethod);
if ismember(TE_beamEdgeX,x)
    % we don't need to insert the point and begin slot cut
else
    cutx = [cutx;TE_beamEdgeX];
    cuty = [cuty;y_valueTE];
end
cutx = [cutx;x(indexAfterTEcut)];
cuty = [cuty;y(indexAfterTEcut)];
end

function result = interpolate2Airfoil(center0Y, tip1Y, coeff0to1)
assert(length(center0Y)==length(tip1Y));
center0Y = center0Y(:);
tip1Y = tip1Y(:);

num = length(tip1Y);
result = zeros(num,length(coeff0to1));
for ii = 1:length(tip1Y)
    result(ii,:) =  interp1([0 1],[center0Y(ii) tip1Y(ii)],coeff0to1,'linear');
end
end