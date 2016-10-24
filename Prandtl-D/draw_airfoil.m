function draw_airfoil()
% save airfoil centerlinex centerliney wingtipx wingtipy twistx twisty
close all;
load airfoil centerlinex centerliney wingtipx wingtipy twistx twisty;
global X;
X = [];
scalingfactor = 0.5;
NASA_wingSpan_mm = 3750;
NASA_centerCord_mm = 400;
NASA_tipcord_mm = 100;
ribCnt = 39;
X.wingSpan_mm = NASA_wingSpan_mm*scalingfactor;
X.rootCord_mm = NASA_centerCord_mm*scalingfactor;
X.wingtipCord_mm = NASA_tipcord_mm*scalingfactor;
X.LeadingEdgeBeamThickness_mm = 2; % square edge size 
X.LeadingEdgeBeamWidth_mm = 3; % square edge size 
X.trailingEdgeWidth_mm = 10;

X.frontMainBeamWidth_mm = 5;
X.frontMainBeamXmm = -5;
X.rearMainBeamWidth_mm = 5;
X.rearMainBeamXmm = 5;

X.RibThickness_mm = 2;

X.PointType = [];
X.PointType.fundamental = 1;
X.PointType.addedPoints = 2;

X.ForceBearingBeamCnt = 0;

X.curvefitmethod = 'pchip';
% rootBeamGammaWidthmm = 6;
% BaseBeamWidthmm = 1; % at tip, lift is zero, we cannot have 0 beam width for misc struct weight 
assert(mod(ribCnt,2)==1); % we have one rib in the center, and be symmetrical 
rotateFromLE = 0.3; %rotation occurs at the center of [0,1] from the leading edge, so that we have the desired AOA.  
fineX = [0:0.001:0.05 0.06:0.01:0.90 0.901:0.001:1]';
% fineX = [0:0.0005:1]';

%%
Len_fineX = length(fineX);
X.OneSideRibCnt = (ribCnt+1)/2;
RibCoeffsRoot2Tip = (0:X.OneSideRibCnt-1)/(X.OneSideRibCnt-1);

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
top_ribs = interpolate2Airfoil(top_centerline_fine_y, top_wingtip_mid_y, RibCoeffsRoot2Tip);
btm_ribs = interpolate2Airfoil(btm_centerline_fine_y, btm_wingtip_mid_y, RibCoeffsRoot2Tip);
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
twist_ribs = interp1(twistx/twistx(end),twisty,RibCoeffsRoot2Tip,X.curvefitmethod);

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
scale_ribs = interp1([0 1],[X.rootCord_mm X.wingtipCord_mm],RibCoeffsRoot2Tip,'linear');

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
Gamma = (1-RibCoeffsRoot2Tip.^2).^1.5;
figure;
plot(RibCoeffsRoot2Tip,Gamma,'.-');
title('Gamma, airflow circulation');
grid on;
%%
figure;
plot(RibCoeffsRoot2Tip,1.5*(RibCoeffsRoot2Tip.^2-0.5),'.-');
title('Downwash');
grid on;

%% when cutting slot, the length of x/y data may not be same any more for all ribs
% use cell to store
slotcut_ribs = cell(1,X.OneSideRibCnt);
for ii = 1:X.OneSideRibCnt
    slotcut_ribs{ii}.top.x = scaled_ribs_top_x(:,ii);
    slotcut_ribs{ii}.top.y = scaled_ribs_top_y(:,ii);
    slotcut_ribs{ii}.top.PointType = ones(length(scaled_ribs_top_x(:,ii)),1)*X.PointType.fundamental;
    slotcut_ribs{ii}.btm.x = scaled_ribs_btm_x(:,ii);
    slotcut_ribs{ii}.btm.y = scaled_ribs_btm_y(:,ii);
    slotcut_ribs{ii}.btm.PointType = ones(length(scaled_ribs_btm_x(:,ii)),1)*X.PointType.fundamental;
    slotcut_ribs{ii}.ForceBearingBeamCut = [];
    slotcut_ribs{ii}.ForceBearingBeamRibThickness = [];
end
%% cut slot for force bearing beam
slotcut_ribs = cutslot(slotcut_ribs,X.frontMainBeamWidth_mm,X.frontMainBeamXmm,'front');

slotcut_ribs = cutslot(slotcut_ribs,X.rearMainBeamWidth_mm,X.rearMainBeamXmm,'rear');

%% add leading edge beam
cut_leadingedge = slotcut_ribs;

figure;
axis equal;
hold on;
for ii = 1:X.OneSideRibCnt
    LE_x = min([slotcut_ribs{ii}.top.x;slotcut_ribs{ii}.btm.x]);
    LeadingEdgeBeamPosDetected = false;
    for xposition = LE_x:0.001:LE_x+30
        Thickness_mm = getThickness(slotcut_ribs{ii},xposition);
        if Thickness_mm > X.LeadingEdgeBeamThickness_mm
            LeadingEdgeBeamPosDetected = true;
            break;
        end
    end
    assert(LeadingEdgeBeamPosDetected);
    cut_leadingedge{ii}.top = removeLeadingSector(slotcut_ribs{ii}.top,xposition);
    cut_leadingedge{ii}.btm = removeLeadingSector(slotcut_ribs{ii}.btm,xposition);
    
    afterCutThickness_mm = abs(cut_leadingedge{ii}.top.y(1) - cut_leadingedge{ii}.btm.y(1));
    cut_leadingedge{ii}.top = cutLeadingEdgeSlot(cut_leadingedge{ii}.top,'top',afterCutThickness_mm);
    cut_leadingedge{ii}.btm = cutLeadingEdgeSlot(cut_leadingedge{ii}.btm,'btm',afterCutThickness_mm);
    
    plot([cut_leadingedge{ii}.top.x; cut_leadingedge{ii}.btm.x],[cut_leadingedge{ii}.top.y; cut_leadingedge{ii}.btm.y],'.');
end
title('Ribs:shifted,rotated,scaled,LEcut');
grid on;
xlabel('mm');
ylabel('mm');

%% cut trailing edge
cut_trailingedge = cut_leadingedge;
figure;
axis equal;
hold on;
for ii = 1:X.OneSideRibCnt
    cut_trailingedge{ii}.top = removeTrailingSector(cut_trailingedge{ii}.top,X.trailingEdgeWidth_mm);
    cut_trailingedge{ii}.btm = removeTrailingSector(cut_trailingedge{ii}.btm,X.trailingEdgeWidth_mm);
    plot([cut_trailingedge{ii}.top.x; cut_trailingedge{ii}.btm.x],[cut_trailingedge{ii}.top.y; cut_trailingedge{ii}.btm.y],'.');
end
title('Ribs:shifted,rotated,scaled,LEcut,TEcut');
grid on;
xlabel('mm');
ylabel('mm');

%% produce rib cut cad file
for ii = 1:X.OneSideRibCnt
    cut_trailingedge{ii}.btm = reverseSeqOrder(cut_trailingedge{ii}.btm);
%     save_finalResults(ii,[cut_trailingedge_top_x{ii}; cut_trailingedge_btm_x{ii}],[cut_trailingedge_top_y{ii}; cut_trailingedge_btm_y{ii}]);
end

fid = fopen('airfoil_all.scr','w');
fprintf(fid,'pline\n');
for ii = 1:X.OneSideRibCnt
    save_finalResults2onefile(fid,ii,0,50,[cut_trailingedge{ii}.top.x; cut_trailingedge{ii}.btm.x],[cut_trailingedge{ii}.top.y; cut_trailingedge{ii}.btm.y]);
end
fclose(fid);

%% produce force bearing beam cut cad file
RibLocationXmm = RibCoeffsRoot2Tip*X.wingSpan_mm/2;
FineSegmentCnt = 1000;

for iiBeam = 1:X.ForceBearingBeamCnt
    BeamShape = cell(1,X.ForceBearingBeamCnt);
    BeamShapeAddBtmEdge = cell(1,X.ForceBearingBeamCnt);
    fid = fopen(sprintf('beam_%d.scr',iiBeam),'w');
    fprintf(fid,'pline\n');
    ThicknessVec = zeros(1,X.OneSideRibCnt);
    CutDepthVec = zeros(1,X.OneSideRibCnt);
    for iiRib = 1:X.OneSideRibCnt
        ThicknessVec(iiRib) = cut_trailingedge{iiRib}.ForceBearingBeamRibThickness(iiBeam);
        CutDepthVec(iiRib) =  cut_trailingedge{iiRib}.ForceBearingBeamCut(iiBeam);
    end
    
    figure;
    subplot(2,1,1);
    plot(RibLocationXmm,ThicknessVec,'.-');
    axis equal;
    xlabel('single side wingspan(mm)');
    ylabel('mm');
    title(sprintf('Rib Thickness for Beam No.%d',iiBeam));
    subplot(2,1,2);
    plot(RibLocationXmm,CutDepthVec,'.-');
    axis equal;
    xlabel('single side wingspan(mm)');
    ylabel('mm');
    title(sprintf('Rib Cut Depth for Beam No.%d',iiBeam));
    
    fineRibLocationXmm = ((0:FineSegmentCnt)/FineSegmentCnt)*(X.wingSpan_mm/2);
    BeamShape{iiBeam}.x = fineRibLocationXmm';
    fineThicknessVec = interp1(RibLocationXmm,ThicknessVec,fineRibLocationXmm,X.curvefitmethod);
    BeamShape{iiBeam}.y = fineThicknessVec';
    BeamShape{iiBeam}.PointType = ones(length(fineRibLocationXmm),1)*X.PointType.fundamental;
    
    BeamShapeCutRibSlot = BeamShape;
    for CutSlotForRib_ii = 1:X.OneSideRibCnt
        switch CutSlotForRib_ii
            case 1
               StyleCutRibSlot = 'RootRib';
            case X.OneSideRibCnt
               StyleCutRibSlot = 'TipRib';
            otherwise
               StyleCutRibSlot = 'MiddleRib';
        end
        BeamShapeCutRibSlot{iiBeam} = cutBeamSlot(BeamShapeCutRibSlot{iiBeam},CutDepthVec(CutSlotForRib_ii),X.RibThickness_mm,RibLocationXmm(CutSlotForRib_ii),StyleCutRibSlot);
    end
    BeamShapeAddBtmEdge{iiBeam} = AddBtmBeamEdge( BeamShapeCutRibSlot{iiBeam} );
    figure;
    plot(BeamShapeAddBtmEdge{iiBeam}.x,BeamShapeAddBtmEdge{iiBeam}.y,'.-');
    axis equal;
    xlabel('single side wingspan(mm)');
    ylabel('mm');
    title(sprintf('Beam%d',iiBeam));
    save_finalResults2onefile(fid,iiBeam,0,100,BeamShapeAddBtmEdge{iiBeam}.x,BeamShapeAddBtmEdge{iiBeam}.y);
    fclose(fid);
end
end


function NewCurve = AddBtmBeamEdge(Curve)
global X;
NewCurve = [];
NewCurve.x = [min(Curve.x);Curve.x;max(Curve.x)];
NewCurve.y = [0;Curve.y;0];
NewCurve.PointType = [X.PointType.addedPoints;Curve.PointType;X.PointType.addedPoints];

end

function newcurve = reverseSeqOrder(curve)
newcurve = [];
newcurve.x = curve.x(end:-1:1);
newcurve.y = curve.y(end:-1:1);
newcurve.PointType = curve.PointType(end:-1:1);
end

function save_finalResults2onefile(fid,idx, xoffset, yoffset, xx, yy)
assert(length(xx)==length(yy));
xoffset = (idx-1)*xoffset;
yoffset = (idx-1)*yoffset;
for ii = 1:length(xx)
    fprintf(fid,'%f,%f\n',xx(ii)+xoffset,yy(ii)+yoffset);
end
fprintf(fid,'%f,%f\n',xx(1)+xoffset,yy(1)+yoffset);
end

% function save_finalResults(idx, xx, yy)
% fid = fopen(sprintf('airfoil%d.scr',idx),'w');
% fprintf(fid,'pline\n');
% assert(length(xx)==length(yy));
% for ii = 1:length(xx)
%     fprintf(fid,'%f,%f\n',xx(ii),yy(ii));
% end
% fprintf(fid,'%f,%f\n',xx(1),yy(1));
% fclose(fid);
% end,

function newcurve = cutLeadingEdgeSlot(curve,edgetype,Thickness_mm)
global X;
newcurve = [];
assert( Thickness_mm > X.LeadingEdgeBeamThickness_mm);

LE_x = curve.x(1);
LE_y = curve.y(1);

yshift = ( Thickness_mm - X.LeadingEdgeBeamThickness_mm)/2;

switch edgetype
    case 'top'
        precise_y = LE_y-yshift;
    case 'btm'
        precise_y = LE_y+-yshift;
    otherwise
        assert(false);
end
newcurve.x = [LE_x;curve.x];
newcurve.y = [precise_y;curve.y];
newcurve.PointType = [X.PointType.addedPoints;curve.PointType];

newcurve.x = [LE_x+X.LeadingEdgeBeamWidth_mm;newcurve.x];
newcurve.y = [precise_y;newcurve.y];
newcurve.PointType = [X.PointType.addedPoints;newcurve.PointType];
end

function newcurve = removeTrailingSector(curve,cutoffx)
global X;
newcurve = [];
x = curve.x;
y = curve.y;
PointType = curve.PointType;
cutoff_x = max(x)-cutoffx;
index = find(x<=cutoff_x);
newcurve.x = x(index);
newcurve.y = y(index);
newcurve.PointType = PointType(index);
if ~ismember( cutoff_x, x )
    newcurve.x = [newcurve.x;cutoff_x];
    val = interp1nearby(x,y,PointType,cutoff_x,2,X.curvefitmethod);
    newcurve.y = [newcurve.y;val];
    newcurve.PointType = [newcurve.PointType;X.PointType.fundamental];
end
end

function newCurve = removeLeadingSector(curve,cutoffx)
global X;
newCurve = [];
assert(~ismember(cutoffx,curve.x));

newCurve.x = cutoffx;
newCurve.y = interp1nearby(curve.x,curve.y,curve.PointType,cutoffx,2);
newCurve.PointType = X.PointType.addedPoints;

indexaftercutoff = find(curve.x>=cutoffx);
newCurve.x = [newCurve.x;curve.x(indexaftercutoff)];
newCurve.y = [newCurve.y;curve.y(indexaftercutoff)];
newCurve.PointType = [newCurve.PointType;curve.PointType(indexaftercutoff)];
end

function [x,y] = normalizexy(x,y)
    scaleF = abs(max(x)-min(x));
    x = x/scaleF;
    y = y/scaleF;
    
end

function ribs = cutslot(ribs,width,xposition,slotName)
global X;
X.ForceBearingBeamCnt = X.ForceBearingBeamCnt + 1;
figure;
axis equal;
hold on;
for ii = 1:X.OneSideRibCnt
    thickness = getThickness(ribs{ii},xposition);
    [ribs{ii}.top.x,ribs{ii}.top.y] = cutnothing(ribs{ii}.top.x,ribs{ii}.top.y);
    cutDepth = thickness/2;
    ribs{ii}.btm = cutslotIn1curve(ribs{ii}.btm,-cutDepth,width,xposition,'cutBoth');
    ribs{ii}.ForceBearingBeamCut = [ribs{ii}.ForceBearingBeamCut;cutDepth];
    ribs{ii}.ForceBearingBeamRibThickness  = [ribs{ii}.ForceBearingBeamRibThickness;thickness];
    plot([ribs{ii}.top.x; ribs{ii}.btm.x],[ribs{ii}.top.y; ribs{ii}.btm.y],'.');
end
title(['Ribs:shifted,rotated,scaled,slotcut' slotName]);
grid on;
end

function val = interp1nearby(all_x,all_y,PointType,newx,nearbyCnt,type)
global X;
if ~exist('type','var')
    type = X.curvefitmethod;
end
% all_x contains points in the fundamental rib contour and points added for slots.
% basic_x contains points in the fundamental rib contour only
basic_x = all_x(PointType==X.PointType.fundamental);
basic_y = all_y(PointType==X.PointType.fundamental);
[~,ii] = min(abs(basic_x-newx));
if ii-nearbyCnt > 0
    leftii = ii-nearbyCnt;
else
    leftii = 1;
end
if ii+nearbyCnt <= length(basic_x)
    rightii = ii+nearbyCnt;
else
    rightii = length(basic_x); 
end
val = interp1(basic_x(leftii:rightii),basic_y(leftii:rightii),newx,type);
end

function thickness = getThickness(rib,x)
global X;
x1 = rib.top.x;
y1 = rib.top.y;
PointType1 = rib.top.PointType;
x2 = rib.btm.x;
y2 = rib.btm.y;
PointType2 = rib.btm.PointType;
y_value1 = interp1nearby(x1,y1,PointType1,x,2,X.curvefitmethod);
y_value2 = interp1nearby(x2,y2,PointType2,x,2,X.curvefitmethod);
thickness = abs(y_value1-y_value2);
end

function [cutx,cuty] = cutnothing(x,y)
cutx = x;
cuty = y;
end

function NewCurve = cutBeamSlot(Curve,depth,width,xposition,style)
switch style
    case 'RootRib'
        NewCurve = cutslotIn1curve(Curve,depth,width,xposition,'cutLeft');
    case 'TipRib'
        NewCurve = cutslotIn1curve(Curve,depth,width,xposition,'cutRight');
    case 'MiddleRib'
        NewCurve = cutslotIn1curve(Curve,depth,width,xposition,'cutBoth');
end
end

%function [cutx,cuty,NewPointType] = cutslotIn1curve(x,y,PointType,depth,width,xposition)
function NewCurve = cutslotIn1curve(Curve,depth,width,xposition,cutmethod)
% x,y form the curve
% depth: depth to be cut. 
%   +: cut to Y-
%   -: cut to Y+
% width: width to be cut
% xposition: relative to xposition, cut width/2 to X+ and X-
% cutmethod:
%   cutLeft: on the left side, don't rise to original height after the cut 
%   cutRight: on the Right side, don't rise to original height after the cut 
%   cutBoth: both sides, rise to original height after the cut 
if ~exist('cutmethod','var')
    cutmethod = 'cutBoth';
end

global X;
NewCurve = [];
NewCurve.x = [];
NewCurve.y = [];
NewCurve.PointType = [];
LE_beamEdgeX = xposition-width/2;
TE_beamEdgeX = xposition+width/2;

switch cutmethod
    case 'cutLeft'
        % do nothing
    case {'cutRight','cutBoth'}
        indexBeforeLEcut = find(Curve.x<=LE_beamEdgeX);
        NewCurve.x = Curve.x(indexBeforeLEcut);
        NewCurve.y = Curve.y(indexBeforeLEcut);
        NewCurve.PointType = Curve.PointType(indexBeforeLEcut);
        y_valueLE = interp1nearby(Curve.x,Curve.y,Curve.PointType,LE_beamEdgeX,2,X.curvefitmethod);
        if ismember(LE_beamEdgeX,Curve.x)
            % we don't need to insert the point and begin slot cut
        else
            NewCurve.x = [NewCurve.x;LE_beamEdgeX];
            NewCurve.y = [NewCurve.y;y_valueLE];
            NewCurve.PointType = [NewCurve.PointType;X.PointType.fundamental];
        end
    otherwise
        assert(false);
end
y_valueSlotCenterPosition = interp1nearby(Curve.x,Curve.y,Curve.PointType,xposition,2,X.curvefitmethod);

NewCurve.x = [NewCurve.x;LE_beamEdgeX];
NewCurve.y = [NewCurve.y;y_valueSlotCenterPosition - depth];
NewCurve.PointType = [NewCurve.PointType;X.PointType.addedPoints];

NewCurve.x = [NewCurve.x;TE_beamEdgeX];
NewCurve.y = [NewCurve.y;y_valueSlotCenterPosition - depth];
NewCurve.PointType = [NewCurve.PointType;X.PointType.addedPoints];

switch cutmethod
    case 'cutRight'
        % do nothing
    case {'cutLeft','cutBoth'}
        y_valueTE = interp1nearby(Curve.x,Curve.y,Curve.PointType,TE_beamEdgeX,2,X.curvefitmethod);
        if ismember(TE_beamEdgeX,Curve.x)
            % we don't need to insert the point and begin slot cut
        else
            NewCurve.x = [NewCurve.x;TE_beamEdgeX];
            NewCurve.y = [NewCurve.y;y_valueTE];
            NewCurve.PointType = [NewCurve.PointType;X.PointType.fundamental];
        end
    otherwise
        assert(false);
end
indexAfterTEcut = find(Curve.x>=TE_beamEdgeX);

NewCurve.x = [NewCurve.x;Curve.x(indexAfterTEcut)];
NewCurve.y = [NewCurve.y;Curve.y(indexAfterTEcut)];
NewCurve.PointType = [NewCurve.PointType;ones(length(indexAfterTEcut),1)*X.PointType.fundamental];
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