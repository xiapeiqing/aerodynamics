function draw_airfoil(scalingfactor,LE_sweepbackDeg,dihedralDeg,ribCnt,RibCntNeedCarbon)
% save airfoil centerlinex centerliney wingtipx wingtipy twistx twisty
close all;
load airfoil centerlinex centerliney wingtipx wingtipy twistx twisty;

NASA_wingSpan_mm = 3750;
NASA_centerCord_mm = 400;
NASA_tipcord_mm = 100;
% length of aileron measured in the right direction, not along the sweepback direction 
NASA_aileron_yAxislength_mm = 22.125*25.4;
NASA_aileron_innerwidth_mm = 1.875*25.4;
NASA_aileron_outerwidth_mm = 1*25.4;
%%
global X;
X = [];
X.setting.geometry.LE_sweepbackDeg = LE_sweepbackDeg;
% Dihedral is an angle raising the centerline of the wing tip above the centerline of the wing root.
X.setting.geometry.dihedralDeg = dihedralDeg;
X.setting.geometry.wingSpan_mm = NASA_wingSpan_mm*scalingfactor;
X.setting.geometry.rootCord_mm = NASA_centerCord_mm*scalingfactor;
X.setting.geometry.wingtipCord_mm = NASA_tipcord_mm*scalingfactor;
X.setting.aileron.yAxislength_mm = NASA_aileron_yAxislength_mm*scalingfactor;
X.setting.aileron.innerwidth_mm = NASA_aileron_innerwidth_mm*scalingfactor;
X.setting.aileron.outerwidth_mm = NASA_aileron_outerwidth_mm*scalingfactor;
fprintf(1,'Summary Report\n');
fprintf(1,'aileron\n\tY axis length=%2.1fmm\n\taileron inner width=%2.1fmm\n\taileron outer width=%2.1fmm\n',X.setting.aileron.yAxislength_mm,X.setting.aileron.innerwidth_mm,X.setting.aileron.outerwidth_mm);

X.setting.manufacture.ribCnt = ribCnt;
X.setting.manufacture.LeadingEdgeBeamThickness_mm = 2; % square edge size 
X.setting.manufacture.LeadingEdgeBeamWidth_mm = 6; % square edge size 
X.setting.manufacture.trailingEdgeWidth_mm = 10;

% support structure height, not part of the plane, only used in construction process as the base support structure for each rib 
X.setting.manufacture.RibSupportStructureHeight_mm = 15;

X.setting.manufacture.MainBeamWidth_mm = 6;
X.setting.manufacture.MainBeamXmm = 0;
assert(X.setting.manufacture.MainBeamXmm == 0); % we want the beam deployed at rotation center to have flat btm edge 

X.RibCntNeedCarbon = RibCntNeedCarbon;
assert(mod(X.RibCntNeedCarbon,2)==1 || X.RibCntNeedCarbon == 0,'rib with added carbon beam should be symmetrical btw the fuselage');
% we may want to add a square carbon tube to strengthen the beam
X.beamSquareCarbonTubeSize_mm = 5;

X.RibThickness_mm = 2;

% the width of laser trace cannot be ignored
X.TraceCompensation.beamSlot_mm = 0.5;
X.TraceCompensation.beamEdge_mm = 0.8;
X.TraceCompensation.RibSlot_mm = 0.05;
X.TraceCompensation.LeadingEdgeBeam_mm = 0.2;

% four quadrants formed by main beam and rib are supported by 4 pieces of wedge. 
X.BeamRibSupport_mm = 10;
X.PointType = [];
X.PointType.fundamental = 1;
X.PointType.addedPoints = 2;

% increased by 1 after each beam slot cut
X.ForceBearingBeamCnt = 0;

X.curvefitmethod = 'pchip';
% rootBeamGammaWidthmm = 6;
% BaseBeamWidthmm = 1; % at tip, lift is zero, we cannot have 0 beam width for misc struct weight 
assert(mod(X.setting.manufacture.ribCnt,2)==1); % we have one rib in the center, and be symmetrical 
% beam deployment location
X.rotateFromLE = 0.3; %rotation occurs at the center of [0,1] from the leading edge, so that we have the desired AOA.  
fineX = [0:0.001:0.05 0.06:0.01:0.90 0.901:0.001:1]';
% fineX = [0:0.0005:1]';
%%
Len_fineX = length(fineX);
X.OneSideRibCnt = (X.setting.manufacture.ribCnt+1)/2;
aileronLocation = (X.setting.geometry.wingSpan_mm/2-X.setting.aileron.yAxislength_mm)/(X.setting.geometry.wingSpan_mm/2);
ribLocation = (0:X.OneSideRibCnt-1)/(X.OneSideRibCnt-1);

figure;
plot(ribLocation,zeros(1,length(ribLocation)),'o');
hold on;
plot(aileronLocation,0,'rx');
hold off;
legend('rib','aileron');
ButtonName = questdlg('added aileron location acceptable?', ...
    'if there exists a rib in aileron location, we don''t need to design an extra rib', ...
    'accept','ignore Aileron','exit',...
    'accept');
switch ButtonName
    case 'accept'
        X.setting.manufacture.RibCoeffsRoot2Tip = sort([aileronLocation ribLocation]);
    case 'ignore Aileron'
        X.setting.manufacture.RibCoeffsRoot2Tip = ribLocation;
    otherwise
        assert(false);
end
X.OneSideRibCnt = length(X.setting.manufacture.RibCoeffsRoot2Tip);
%%
saveAirfoilProfile2file(centerlinex, centerliney, wingtipx, wingtipy);

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
[top_centerline_x,top_centerline_y,btm_centerline_x,btm_centerline_y] = airfoilProfile2twoPieces(centerlinex,centerliney);
top_centerline_fine_y = interp1(top_centerline_x,top_centerline_y,fineX,X.curvefitmethod);
plot(fineX,top_centerline_fine_y,'.');
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
[top_wingtip_x,top_wingtip_y,btm_wingtip_x,btm_wingtip_y] = airfoilProfile2twoPieces(wingtipx,wingtipy);
top_wingtip_mid_y = interp1(top_wingtip_x,top_wingtip_y,fineX,X.curvefitmethod);
plot(fineX,top_wingtip_mid_y,'.');
btm_wingtip_mid_y = interp1(btm_wingtip_x,btm_wingtip_y,fineX,X.curvefitmethod);
plot(fineX,btm_wingtip_mid_y,'.');
axis equal;
legend('top','btm');
title('wingtip');
grid on;
xlabel('unit');
ylabel('unit');
%%
top_ribs = interpolate2Airfoil(top_centerline_fine_y, top_wingtip_mid_y, X.setting.manufacture.RibCoeffsRoot2Tip);
btm_ribs = interpolate2Airfoil(btm_centerline_fine_y, btm_wingtip_mid_y, X.setting.manufacture.RibCoeffsRoot2Tip);
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
xindexRotate = find(fineX==X.rotateFromLE);
assert(length(xindexRotate)==1);
centered_interpolatedX = fineX - X.rotateFromLE;
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
twist_ribsDeg = interp1(twistx/twistx(end),twisty,X.setting.manufacture.RibCoeffsRoot2Tip,X.curvefitmethod);

rotated_ribs_top_x = zeros(Len_fineX,X.OneSideRibCnt);
rotated_ribs_top_y = zeros(Len_fineX,X.OneSideRibCnt);
rotated_ribs_btm_x = zeros(Len_fineX,X.OneSideRibCnt);
rotated_ribs_btm_y = zeros(Len_fineX,X.OneSideRibCnt);
figure;
axis equal;
hold on;
for ii = 1:X.OneSideRibCnt
    thetaDeg = twist_ribsDeg(ii);
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
%% scale rib to the desired cord length
scaled_ribs_top_x = rotated_ribs_top_x;
scaled_ribs_top_y = rotated_ribs_top_y;
scaled_ribs_btm_x = rotated_ribs_btm_x;
scaled_ribs_btm_y = rotated_ribs_btm_y;
ribs_lengthmm = interp1([0 1],[X.setting.geometry.rootCord_mm X.setting.geometry.wingtipCord_mm],X.setting.manufacture.RibCoeffsRoot2Tip,'linear');

figure;
axis equal;
hold on;
for ii = 1:X.OneSideRibCnt
    scaled_ribs_top_x(:,ii) = scaled_ribs_top_x(:,ii) * ribs_lengthmm(ii);
    scaled_ribs_top_y(:,ii) = scaled_ribs_top_y(:,ii) * ribs_lengthmm(ii);
    scaled_ribs_btm_x(:,ii) = scaled_ribs_btm_x(:,ii) * ribs_lengthmm(ii);
    scaled_ribs_btm_y(:,ii) = scaled_ribs_btm_y(:,ii) * ribs_lengthmm(ii);
    plot([scaled_ribs_top_x(:,ii); scaled_ribs_btm_x(:,ii)],[scaled_ribs_top_y(:,ii); scaled_ribs_btm_y(:,ii)],'.');
end
title('Ribs:shifted,rotated,scaled');
grid on;
xlabel('mm');
ylabel('mm');

%% shift in y-axis to align the bottom edge of main beam
btmaligned_ribs_top_x = scaled_ribs_top_x;
btmaligned_ribs_top_y = scaled_ribs_top_y;
btmaligned_ribs_btm_x = scaled_ribs_btm_x;
btmaligned_ribs_btm_y = scaled_ribs_btm_y;

figure;
axis equal;
hold on;
for ii = 1:X.OneSideRibCnt
    YatZeroX = interp1(btmaligned_ribs_btm_x(:,ii),btmaligned_ribs_btm_y(:,ii),0,X.curvefitmethod);
    btmaligned_ribs_top_y(:,ii) = btmaligned_ribs_top_y(:,ii) - YatZeroX;
    btmaligned_ribs_btm_y(:,ii) = btmaligned_ribs_btm_y(:,ii) - YatZeroX;
    plot([btmaligned_ribs_top_x(:,ii); btmaligned_ribs_btm_x(:,ii)],[btmaligned_ribs_top_y(:,ii); btmaligned_ribs_btm_y(:,ii)],'.');
end
title('Ribs:btmEdgeAlign,shifted,rotated,scaled');
grid on;
xlabel('mm');
ylabel('mm');

%% compute angle between beam and rib
CalcObtuseAngleBtwBeamAndRibDeg(btmaligned_ribs_top_x);
space = 10;
fid = fopen('BeamRibSupport.scr','w');
fprintf(fid,'pline\n');
% quadrant 1
fprintf(fid,'%f,%f\n',space,space);
fprintf(fid,'%f,%f\n',space,space+X.BeamRibSupport_mm);
fprintf(fid,'%f,%f\n',space+X.BeamRibSupport_mm*sind(X.derived.AngleBtwBeamAndRibDeg),space-X.BeamRibSupport_mm*cosd(X.derived.AngleBtwBeamAndRibDeg));
fprintf(fid,'%f,%f\n',space,space);

% quadrant 2
fprintf(fid,'%f,%f\n',-space,space);
fprintf(fid,'%f,%f\n',-space,space+X.BeamRibSupport_mm);
fprintf(fid,'%f,%f\n',-space-X.BeamRibSupport_mm*sind(X.derived.AngleBtwBeamAndRibDeg),space+X.BeamRibSupport_mm*cosd(X.derived.AngleBtwBeamAndRibDeg));
fprintf(fid,'%f,%f\n',-space,space);

% quadrant 3
fprintf(fid,'%f,%f\n',-space,-space);
fprintf(fid,'%f,%f\n',-space,-space-X.BeamRibSupport_mm);
fprintf(fid,'%f,%f\n',-space-X.BeamRibSupport_mm*sind(X.derived.AngleBtwBeamAndRibDeg),-space+X.BeamRibSupport_mm*cosd(X.derived.AngleBtwBeamAndRibDeg));
fprintf(fid,'%f,%f\n',-space,-space);

% quadrant 4
fprintf(fid,'%f,%f\n',space,-space);
fprintf(fid,'%f,%f\n',space,-space-X.BeamRibSupport_mm);
fprintf(fid,'%f,%f\n',space+X.BeamRibSupport_mm*sind(X.derived.AngleBtwBeamAndRibDeg),-space-X.BeamRibSupport_mm*cosd(X.derived.AngleBtwBeamAndRibDeg));
fprintf(fid,'%f,%f\n',space,-space);

fclose(fid);
%%
Gamma = (1-X.setting.manufacture.RibCoeffsRoot2Tip.^2).^1.5;
figure;
plot(X.setting.manufacture.RibCoeffsRoot2Tip,Gamma,'.-');
title('Gamma, airflow circulation');
grid on;
%%
figure;
plot(X.setting.manufacture.RibCoeffsRoot2Tip,1.5*(X.setting.manufacture.RibCoeffsRoot2Tip.^2-0.5),'.-');
title('Downwash');
grid on;

%% when cutting slot, the length of x/y data may not be same any more for all ribs
% use cell to store
slotcut_ribs = cell(1,X.OneSideRibCnt);
for ii = 1:X.OneSideRibCnt
    slotcut_ribs{ii}.top.x = btmaligned_ribs_top_x(:,ii);
    slotcut_ribs{ii}.top.y = btmaligned_ribs_top_y(:,ii);
    slotcut_ribs{ii}.top.PointType = ones(length(btmaligned_ribs_top_x(:,ii)),1)*X.PointType.fundamental;
    slotcut_ribs{ii}.btm.x = btmaligned_ribs_btm_x(:,ii);
    slotcut_ribs{ii}.btm.y = btmaligned_ribs_btm_y(:,ii);
    slotcut_ribs{ii}.btm.PointType = ones(length(btmaligned_ribs_btm_x(:,ii)),1)*X.PointType.fundamental;
    slotcut_ribs{ii}.ForceBearingBeamCutDepth = [];
    slotcut_ribs{ii}.ForceBearingBeamRibThickness = [];
    slotcut_ribs{ii}.ForceBearingBeamRibYoffset = [];
end
[X.BeamBtmEdgeRotDegDueToDihedral,OneSideWingTipRise_mm] = getDihedralRotateDeg(slotcut_ribs);
fprintf(1,'due to %2.1fDeg dihedral(dihedral is measured at center line):\n',X.setting.geometry.dihedralDeg);
fprintf(1,'\tthe center line of one side wing tip rises %2.1fmm at beam\n',OneSideWingTipRise_mm);
fprintf(1,'\tBeam Bottom Edge Rotate Degree: %3.2fDeg\n',X.BeamBtmEdgeRotDegDueToDihedral);

%%
X.derived.RibLocInBeam_mm = X.setting.manufacture.RibCoeffsRoot2Tip*(1/sind(X.derived.AngleBtwBeamAndRibDeg))*X.setting.geometry.wingSpan_mm/2;

%% cut slot for force bearing beam
[slotcut_ribs,ribMirror] = cutRibSlot(slotcut_ribs,X.setting.manufacture.MainBeamWidth_mm-2*X.TraceCompensation.RibSlot_mm,X.setting.manufacture.MainBeamXmm,'main beam');
fid = fopen(sprintf('ribMirror.scr'),'w');
fprintf(fid,'pline\n');
for ii = 1:X.OneSideRibCnt
    saveTopBtmCell2scrFile(fid,ribMirror{ii},0,ii*100);
end
fclose(fid);

%% add leading edge beam
cut_leadingedge = slotcut_ribs;

figure;
axis equal;
hold on;
for ii = 1:X.OneSideRibCnt
    LE_x = min([slotcut_ribs{ii}.top.x;slotcut_ribs{ii}.btm.x]);
    LeadingEdgeBeamPosDetected = false;
    for xposition = LE_x:0.001:LE_x+30
        [Thickness_mm,~,~] = getThickness(slotcut_ribs{ii},xposition);
        if Thickness_mm > X.setting.manufacture.LeadingEdgeBeamThickness_mm - 2*X.TraceCompensation.LeadingEdgeBeam_mm
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
    cut_trailingedge{ii}.top = removeTrailingSector(cut_trailingedge{ii}.top,X.setting.manufacture.trailingEdgeWidth_mm);
    cut_trailingedge{ii}.btm = removeTrailingSector(cut_trailingedge{ii}.btm,X.setting.manufacture.trailingEdgeWidth_mm);
    plot([cut_trailingedge{ii}.top.x; cut_trailingedge{ii}.btm.x],[cut_trailingedge{ii}.top.y; cut_trailingedge{ii}.btm.y],'.');
end
title('Ribs:shifted,rotated,scaled,LEcut,TEcut');
grid on;
xlabel('mm');
ylabel('mm');

% rotate rib to align camber line with wood growth direction in order to enhance the strength
rib_aligned = cell(1,X.OneSideRibCnt);
figure;
axis equal;
hold on;
for ii = 1:X.OneSideRibCnt
    rib_aligned{ii}.top = rotateXY(cut_trailingedge{ii}.top,-twist_ribsDeg(ii));
    rib_aligned{ii}.btm = rotateXY(cut_trailingedge{ii}.btm,-twist_ribsDeg(ii));
    plot([rib_aligned{ii}.top.x; rib_aligned{ii}.btm.x],[rib_aligned{ii}.top.y; rib_aligned{ii}.btm.y],'.');
end
title('Ribs:shifted,rotated,scaled,LEcut,TEcut,Aligned');
grid on;
xlabel('mm');
ylabel('mm');

%% produce rib cut cad file
fid = fopen('airfoil_all.scr','w');
fprintf(fid,'pline\n');
X.derived.RibSweepBackAlongRib_mm = X.setting.manufacture.RibCoeffsRoot2Tip*X.setting.geometry.wingSpan_mm/2/abs(tand(X.derived.AngleBtwBeamAndRibDeg));
X.derived.RibSweepBackAlongBeam_mm = X.setting.manufacture.RibCoeffsRoot2Tip*X.setting.geometry.wingSpan_mm/2;
for ii = 1:X.OneSideRibCnt
    xoffset = X.derived.RibSweepBackAlongRib_mm(ii);
    yoffset = X.derived.RibSweepBackAlongBeam_mm(ii);
    saveTopBtmCell2scrFile(fid,rib_aligned{ii},xoffset,yoffset);
end
fclose(fid);

%% produce force bearing beam cut cad file
FineSegmentCnt = 1000;

BeamShape = cell(1,X.ForceBearingBeamCnt);
BeamDihedralApplied = cell(1,X.ForceBearingBeamCnt);
BeamAlignedForStrength = cell(1,X.ForceBearingBeamCnt);
BeamLaserTraceCompensated = cell(1,X.ForceBearingBeamCnt);
for iiBeam = 1:X.ForceBearingBeamCnt
%     BeamShapeAddBtmEdge = cell(1,X.ForceBearingBeamCnt);
    ThicknessVec = zeros(1,X.OneSideRibCnt);
    CutDepthVec = zeros(1,X.OneSideRibCnt);
    Yoffset = zeros(1,X.OneSideRibCnt);
    for iiRib = 1:X.OneSideRibCnt
        ThicknessVec(iiRib) = cut_trailingedge{iiRib}.ForceBearingBeamRibThickness(iiBeam);
        CutDepthVec(iiRib) =  cut_trailingedge{iiRib}.ForceBearingBeamCutDepth(iiBeam);
        Yoffset(iiRib) = cut_trailingedge{iiRib}.ForceBearingBeamRibYoffset(iiBeam);
    end
    figure;
    subplot(3,1,1);
    hold on;
    plot(X.derived.RibLocInBeam_mm,ThicknessVec,'.-');
    plot(X.derived.RibLocInBeam_mm,ThicknessVec+Yoffset,'.-');
    legend('Thickness','Thickness+yoffset');
    axis equal;
    xlabel('single side wingspan(mm)');
    ylabel('mm');
    title(sprintf('Top Edge Beam No.%d',iiBeam));
    subplot(3,1,2);
    plot(X.derived.RibLocInBeam_mm,Yoffset,'.-');
    axis equal;
    xlabel('single side wingspan(mm)');
    ylabel('mm');
    title(sprintf('Btm Edge Beam No.%d',iiBeam));
    subplot(3,1,3);
    hold on;
    plot(X.derived.RibLocInBeam_mm,CutDepthVec,'.-');
    plot(X.derived.RibLocInBeam_mm,CutDepthVec+Yoffset,'.-');
    legend('pure cut','cut+yoffset');
    axis equal;
    xlabel('single side wingspan(mm)');
    ylabel('mm');
    title(sprintf('Rib Cut Depth for Beam No.%d',iiBeam));
    
    fineRibLocationXmm = ((0:FineSegmentCnt)/FineSegmentCnt)*(X.setting.geometry.wingSpan_mm/2)*(1/sind(X.derived.AngleBtwBeamAndRibDeg));
    BeamShape{iiBeam}.top = [];
    BeamShape{iiBeam}.top.x = fineRibLocationXmm';
    BeamShape{iiBeam}.top.y = interp1(X.derived.RibLocInBeam_mm,ThicknessVec+Yoffset,fineRibLocationXmm,X.curvefitmethod)';
    BeamShape{iiBeam}.top.PointType = ones(length(fineRibLocationXmm),1)*X.PointType.fundamental;

    % if beam location is at the rotation center, btm edge is a straight line 
    BeamShape{iiBeam}.btm = [];
    BeamShape{iiBeam}.btm.x = fineRibLocationXmm';
    BeamShape{iiBeam}.btm.y = interp1(X.derived.RibLocInBeam_mm,Yoffset,fineRibLocationXmm,X.curvefitmethod)';
    BeamShape{iiBeam}.btm.PointType = ones(length(fineRibLocationXmm),1)*X.PointType.fundamental;
    
    BeamDihedralApplied{iiBeam} = ApplyBeamDihedral(BeamShape{iiBeam},-X.BeamBtmEdgeRotDegDueToDihedral);

    BeamShapeCutRibSlot = BeamDihedralApplied;
    for CutSlotForRib_ii = 1:X.OneSideRibCnt
        switch CutSlotForRib_ii
            case 1
               StyleCutRibSlot = 'RootRib';
            case X.OneSideRibCnt
               StyleCutRibSlot = 'TipRib';
            otherwise
               StyleCutRibSlot = 'MiddleRib';
        end
        BeamShapeCutRibSlot{iiBeam}.top = cutBeamSlot(BeamShapeCutRibSlot{iiBeam}.top,CutDepthVec(CutSlotForRib_ii),X.RibThickness_mm-2*X.TraceCompensation.beamSlot_mm,X.derived.RibLocInBeam_mm(CutSlotForRib_ii),StyleCutRibSlot);
    end
    BeamShapeCutRibSlot{iiBeam}.btm = matchCurveSpan(BeamDihedralApplied{iiBeam}.btm,BeamShapeCutRibSlot{iiBeam}.top);
    draw_beam(BeamShapeCutRibSlot{iiBeam},sprintf('front view Beam%d',iiBeam));
    
    BeamAlignedForStrength{iiBeam} = AlignBeamForStrength(BeamShapeCutRibSlot{iiBeam},-X.BeamBtmEdgeRotDegDueToDihedral);
    draw_beam(BeamAlignedForStrength{iiBeam},sprintf('Aligned Beam%d',iiBeam));

    BeamLaserTraceCompensated{iiBeam} = BeamAlignedForStrength{iiBeam};
    BeamLaserTraceCompensated{iiBeam}.top.y = BeamLaserTraceCompensated{iiBeam}.top.y + X.TraceCompensation.beamEdge_mm;
    BeamLaserTraceCompensated{iiBeam}.btm.y = BeamLaserTraceCompensated{iiBeam}.btm.y - X.TraceCompensation.beamEdge_mm;
    draw_beam(BeamLaserTraceCompensated{iiBeam},sprintf('LaserTrace Compensated Beam%d',iiBeam));
    
    fid = fopen(sprintf('beam_%d.scr',iiBeam),'w');
    fprintf(fid,'pline\n');
    saveTopBtmCell2scrFile(fid,BeamLaserTraceCompensated{iiBeam},0,100);
    fclose(fid);
    
end

printXFLR5info(ribs_lengthmm,twist_ribsDeg);

end

function newCurve = matchCurveSpan(Curve,RefCurve)
global X;
assert(Curve.x(1) < Curve.x(end));
assert(RefCurve.x(1) < RefCurve.x(end));
newCurve = [];
if (Curve.x(1) == RefCurve.x(1))
    % do nothing
elseif (Curve.x(1) > RefCurve.x(1))
    val = interp1nearby(Curve.x,Curve.y,Curve.PointType,RefCurve.x(1),2,X.curvefitmethod);
    newCurve.x = [RefCurve.x(1);Curve.x];
    newCurve.y = [val;Curve.y];
    newCurve.PointType = [X.PointType.fundamental; Curve.PointType];
else
    % trim data
    % code to be filled here
end

if (Curve.x(end) == RefCurve.x(end))
    % do nothing
elseif (Curve.x(end) < RefCurve.x(end))
    val = interp1nearby(Curve.x,Curve.y,Curve.PointType,RefCurve.x(end),2,X.curvefitmethod);
    newCurve.x = [newCurve.x;RefCurve.x(end)];
    newCurve.y = [newCurve.y;val];
    newCurve.PointType = [Curve.PointType;X.PointType.fundamental];
else
    % trim data
    % code to be filled here
end

end

function BeamAlignedForStrength = AlignBeamForStrength(BeamShape,RotateDeg)
BeamAlignedForStrength = [];
% R = [cosd(RotateDeg) -sind(RotateDeg); sind(RotateDeg) cosd(RotateDeg)];
% top_beforeRotate = [BeamShape.top.x BeamShape.top.y];
% top_afterRotate = top_beforeRotate*R;
% BeamAlignedForStrength.top.x = top_afterRotate(:,1);
% BeamAlignedForStrength.top.y = top_afterRotate(:,2);
BeamAlignedForStrength.top = rotateXY(BeamShape.top,-RotateDeg);
BeamAlignedForStrength.btm = rotateXY(BeamShape.btm,-RotateDeg);

offsetx = BeamAlignedForStrength.btm.x(1);
offsety = BeamAlignedForStrength.btm.y(1);

BeamAlignedForStrength.top.x = BeamAlignedForStrength.top.x - offsetx;
BeamAlignedForStrength.btm.x = BeamAlignedForStrength.btm.x - offsetx;

BeamAlignedForStrength.top.y = BeamAlignedForStrength.top.y - offsety;
BeamAlignedForStrength.btm.y = BeamAlignedForStrength.btm.y - offsety;

end

function BeamDihedralApplied = ApplyBeamDihedral(BeamShape,RotateDeg)
global X;
BeamDihedralApplied = [];
% R = [cosd(RotateDeg) -sind(RotateDeg); sind(RotateDeg) cosd(RotateDeg)];
% top_beforeRotate = [BeamShape.top.x BeamShape.top.y];
% top_afterRotate = top_beforeRotate*R;
% BeamDihedralApplied.top.x = top_afterRotate(:,1);
% BeamDihedralApplied.top.y = top_afterRotate(:,2);
BeamDihedralApplied.top = rotateXY(BeamShape.top,RotateDeg);
BeamDihedralApplied.btm = rotateXY(BeamShape.btm,RotateDeg);

% align wingtip 
if (BeamDihedralApplied.top.x(end) == BeamDihedralApplied.btm.x(end))
    % do nothing
elseif (BeamDihedralApplied.top.x(end) > BeamDihedralApplied.btm.x(end))
    val = interp1nearby(BeamDihedralApplied.btm.x,BeamDihedralApplied.btm.y,BeamDihedralApplied.btm.PointType,BeamDihedralApplied.top.x(end),2,X.curvefitmethod);
    BeamDihedralApplied.btm.x = [BeamDihedralApplied.btm.x;BeamDihedralApplied.top.x(end)];
    BeamDihedralApplied.btm.y = [BeamDihedralApplied.btm.y;val];
    BeamDihedralApplied.btm.PointType = [BeamDihedralApplied.btm.PointType;X.PointType.fundamental];
else
    val = interp1nearby(BeamDihedralApplied.top.x,BeamDihedralApplied.top.y,BeamDihedralApplied.top.PointType,BeamDihedralApplied.btm.x(end),2,X.curvefitmethod);
    BeamDihedralApplied.top.x = [BeamDihedralApplied.top.x;BeamDihedralApplied.btm.x(end)];
    BeamDihedralApplied.top.y = [BeamDihedralApplied.top.y;val];
    BeamDihedralApplied.top.PointType = [BeamDihedralApplied.top.PointType;X.PointType.fundamental];
end

% align root 
if (BeamDihedralApplied.top.x(1) == BeamDihedralApplied.btm.x(1))
    % do nothing
elseif (BeamDihedralApplied.top.x(1) > BeamDihedralApplied.btm.x(1))
    val = interp1nearby(BeamDihedralApplied.btm.x,BeamDihedralApplied.btm.y,BeamDihedralApplied.btm.PointType,BeamDihedralApplied.top.x(1),2,X.curvefitmethod);
    BeamDihedralApplied.btm.x = [BeamDihedralApplied.top.x(1);BeamDihedralApplied.btm.x];
    BeamDihedralApplied.btm.y = [val;BeamDihedralApplied.btm.y];
    BeamDihedralApplied.btm.PointType = [X.PointType.fundamental;BeamDihedralApplied.btm.PointType];
else
    val = interp1nearby(BeamDihedralApplied.top.x,BeamDihedralApplied.top.y,BeamDihedralApplied.top.PointType,BeamDihedralApplied.btm.x(1),2,X.curvefitmethod);
    BeamDihedralApplied.top.x = [BeamDihedralApplied.btm.x(1);BeamDihedralApplied.top.x];
    BeamDihedralApplied.top.y = [val;BeamDihedralApplied.top.y];
    BeamDihedralApplied.top.PointType = [X.PointType.fundamental;BeamDihedralApplied.top.PointType];
end

offsetx = BeamDihedralApplied.btm.x(1);
offsety = BeamDihedralApplied.btm.y(1);

BeamDihedralApplied.top.x = BeamDihedralApplied.top.x - offsetx;
BeamDihedralApplied.btm.x = BeamDihedralApplied.btm.x - offsetx;

BeamDihedralApplied.top.y = BeamDihedralApplied.top.y - offsety;
BeamDihedralApplied.btm.y = BeamDihedralApplied.btm.y - offsety;

end

function newxy = rotateXY(xy,RotateDeg)
R = [cosd(RotateDeg) -sind(RotateDeg); sind(RotateDeg) cosd(RotateDeg)];
beforeRotate = [xy.x xy.y];
afterRotate = beforeRotate*R;
newxy = [];
newxy.x = afterRotate(:,1);
newxy.y = afterRotate(:,2);
newxy.PointType = xy.PointType;
end

function draw_beam(TopBtmCell,titlestr)
[xx,yy] = OrderDataForPlot(TopBtmCell);
figure;
plot(xx,yy,'.-');
axis equal;
xlabel('single side wingspan(mm)');
ylabel('mm');
title(titlestr);
end

function [BeamBtmEdgeRotDegDueToDihedral,OneSideWingTipRise_mm] = getDihedralRotateDeg(ribs)
global X;
[RootThickness_mm,~,~] = getThickness(ribs{1},0);
[TipThickness_mm,~,~] = getThickness(ribs{X.OneSideRibCnt},0);
OneSideWingTipRise_mm = ((X.setting.geometry.wingSpan_mm/2)*tand(X.setting.geometry.dihedralDeg)+(RootThickness_mm-TipThickness_mm)/2);
BeamBtmEdgeRotDegDueToDihedral = atand(OneSideWingTipRise_mm/(X.setting.geometry.wingSpan_mm/2)); 
end

function CalcObtuseAngleBtwBeamAndRibDeg(ribs_top_x)
% base on the input arguments:
%   1) Leading edge sweep back angle in degree
%   2) distance from beam center(x position = 0) to the Leading and Trailing edge  
% calculate the obtuse angle Btw
%   1) Beam and Rib in degree
%   2) trailing edge and rib in degree
global X;
root_LE_x = -ribs_top_x(1,1);
tip_LE_x = -ribs_top_x(1,X.OneSideRibCnt);
LE_DeltaX = tan(X.setting.geometry.LE_sweepbackDeg*pi/180)*X.setting.geometry.wingSpan_mm/2;
beam_DeltaX = LE_DeltaX + tip_LE_x - root_LE_x;
X.derived.AngleBtwBeamAndRibDeg = atan(beam_DeltaX/(X.setting.geometry.wingSpan_mm/2))*180/pi+90;
X.derived.AngleBtwTEandRibDeg = atan((LE_DeltaX+X.setting.geometry.wingtipCord_mm-X.setting.geometry.rootCord_mm)/(X.setting.geometry.wingSpan_mm/2))*180/pi+90;
fprintf(1,'Given Leading edge sweep back angle = %2.1fdeg,the obtuse angle btw\n\tBeam & Rib = %2.1fdeg\n\tbtw Trailing edge & Rib = %2.1fdeg\n',...
    X.setting.geometry.LE_sweepbackDeg,...
    X.derived.AngleBtwBeamAndRibDeg,...
    X.derived.AngleBtwTEandRibDeg);
end

function NewCurve = AddStraightLineBtmBeamEdge(Curve)
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

function [xx,yy] = OrderDataForPlot(TopBtmCell)
TopBtmCell.btm = reverseSeqOrder(TopBtmCell.btm);
xx = [TopBtmCell.top.x; TopBtmCell.btm.x];
yy = [TopBtmCell.top.y; TopBtmCell.btm.y];
assert(length(xx)==length(yy));
end

function saveTopBtmCell2scrFile(fid,TopBtmCell,xoffset,yoffset)
[xx,yy] = OrderDataForPlot(TopBtmCell);

for ii = 1:length(xx)
    fprintf(fid,'%f,%f\n',xx(ii)+xoffset,yy(ii)+yoffset);
end

% now go back to the first point so as to close the contour
fprintf(fid,'%f,%f\n',xx(1)+xoffset,yy(1)+yoffset);
end

function NewAirfoilY = interpolateAirfoil(x,y,newx)
global X;
[top_x,top_y,btm_x,btm_y] = airfoilProfile2twoPieces(x,y);
end_of_top_curve = (length(newx)+1)/2;
newtopx = newx(1:end_of_top_curve);
newbtmx = newx(end_of_top_curve:end);
newtopy = [];
newbtmy = [];
for ii = 1:length(newtopx)
    newtopy(ii) = interp1nearby(top_x,top_y,ones(1,length(top_x))*X.PointType.fundamental,newtopx(ii),2);
    newbtmy(ii) = interp1nearby(btm_x,btm_y,ones(1,length(btm_x))*X.PointType.fundamental,newbtmx(ii),2);
end
[~,NewAirfoilY] = twoPieces2airfoilProfile(newx,newtopy',newx,newbtmy');
end

function saveAirfoilProfile2file(centerlinex, centerliney, wingtipx, wingtipy)
global X;
if length(centerlinex) > length(wingtipx)
    x_coordinate = centerlinex;
    centerline_Y = centerliney;
    wingtip_Y = interpolateAirfoil(wingtipx,wingtipy,x_coordinate);
else
    x_coordinate = wingtipx;
    centerline_Y = interpolateAirfoil(centerlinex, centerliney,x_coordinate);
    wingtip_Y = wingtipy;
end
outputfolder = [pwd '\output\'];
if ~exist(outputfolder,'dir')
    mkdir(outputfolder);
end
for ribii = 1:length(X.setting.manufacture.RibCoeffsRoot2Tip)
    result = interpolate2Airfoil(centerline_Y, wingtip_Y, X.setting.manufacture.RibCoeffsRoot2Tip(ribii));
    datafilename = sprintf('%sPrandtlRib%d.dat',outputfolder,ribii);
    %assert(~exist(datafilename,'file'));
    fid = fopen(datafilename,'w');
    fprintf(fid,'PrandtlRib%d airfoil\r\n',ribii);
    
    for pointii = 1:length(x_coordinate)
        fprintf(fid, '%10.9f %10.9f\r\n',x_coordinate(pointii),result(pointii));
    end
    fclose(fid);
end
end

function newcurve = cutLeadingEdgeSlot(curve,edgetype,Thickness_mm)
global X;
newcurve = [];
assert( Thickness_mm > X.setting.manufacture.LeadingEdgeBeamThickness_mm - 2*X.TraceCompensation.LeadingEdgeBeam_mm);

LE_x = curve.x(1);
LE_y = curve.y(1);

yshift = ( Thickness_mm - (X.setting.manufacture.LeadingEdgeBeamThickness_mm - 2*X.TraceCompensation.LeadingEdgeBeam_mm))/2;

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

newcurve.x = [LE_x+X.setting.manufacture.LeadingEdgeBeamWidth_mm;newcurve.x];
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

function [ribs,ribMirror] = cutRibSlot(ribs,width,xposition,slotName)
global X;
ribMirror = cell(1,X.OneSideRibCnt);
X.ForceBearingBeamCnt = X.ForceBearingBeamCnt + 1;
assert(X.ForceBearingBeamCnt == 1,'currently we don''t handle the complexity causd by the second beam deployed at none AOA rotation center, curve bottom edge, dihedral connection, etc'); 
figure;
axis equal;
hold on;
for ii = 1:X.OneSideRibCnt
    ribMirror{ii}.top = cutslotIn1curve(ribs{ii}.btm,2,width,xposition,'cutBoth');
    [thickness,~,btm_mm] = getThickness(ribs{ii},xposition);
    [ribs{ii}.top.x,ribs{ii}.top.y] = cutnothing(ribs{ii}.top.x,ribs{ii}.top.y);
    cutDepth = thickness/2;
    if X.RibCntNeedCarbon == 0
        RibCntOneSideNeedCarbon = 0;
    else
        RibCntOneSideNeedCarbon = (X.RibCntNeedCarbon+1)/2;
    end
    if ii > RibCntOneSideNeedCarbon
        ribs{ii}.btm = cutslotIn1curve(ribs{ii}.btm,-cutDepth,width,xposition,'cutBoth');
    else
        % we attach a square carbon tube to strengthen the wood beam
        ribs{ii}.btm = cutslotIn1curve(ribs{ii}.btm,-cutDepth,width,xposition,'cutBothAndAddedCarbon');
    end
    ribs{ii}.ForceBearingBeamCutDepth = [ribs{ii}.ForceBearingBeamCutDepth;cutDepth];
    ribs{ii}.ForceBearingBeamRibThickness  = [ribs{ii}.ForceBearingBeamRibThickness;thickness];
    ribs{ii}.ForceBearingBeamRibYoffset = [ribs{ii}.ForceBearingBeamRibYoffset;btm_mm];
    plot([ribs{ii}.top.x; ribs{ii}.btm.x],[ribs{ii}.top.y; ribs{ii}.btm.y],'.');
    
    ribMirror{ii}.btm.x = [min(ribs{ii}.top.x);max(ribs{ii}.top.x)];
    ribMirror{ii}.btm.y = -ones(2,1)*X.setting.manufacture.RibSupportStructureHeight_mm;
    ribMirror{ii}.btm.PointType = ones(2,1)*X.PointType.fundamental;
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
neighbourX = basic_x(leftii:rightii);
neighbourY = basic_y(leftii:rightii);

% if length(unique(sign(diff(neighbourX)))) > 1
%     % neighbourX not monotonically increasing/decreasing
%     [~,closestII] = min(abs(neighbourX));
%     assert(closestII ~= 1 && closestII ~= length(neighbourX));
%     if closestII > length(neighbourX)/2
%         neighbourX = neighbourX(1:closestII);
%         neighbourY = neighbourY(1:closestII);
%     else
%     end
% end
val = interp1(neighbourX,neighbourY,newx,type);
end

function [thickness,top_mm, btm_mm] = getThickness(rib,x)
global X;
x1 = rib.top.x;
y1 = rib.top.y;
PointType1 = rib.top.PointType;
x2 = rib.btm.x;
y2 = rib.btm.y;
PointType2 = rib.btm.PointType;
top_mm = interp1nearby(x1,y1,PointType1,x,2,X.curvefitmethod);
btm_mm = interp1nearby(x2,y2,PointType2,x,2,X.curvefitmethod);
thickness = abs(top_mm-btm_mm);
end

function [cutx,cuty] = cutnothing(x,y)
cutx = x;
cuty = y;
end

function NewCurve = cutBeamSlot(Curve,depth,width,xposition,style)
assert(width > 0);
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
%   +: cut to Y-, downward, for example, top edge
%   -: cut to Y+, upward, for example, bottom edge
% width: width to be cut
% xposition: relative to xposition, cut width/2 to X+ and X-
% cutmethod:
%   cutLeft: don't rise the left side to original height after the cut 
%   cutRight: don't rise the Right side to original height after the cut 
%   cutBoth: both sides, rise to original height after the cut 
if ~exist('cutmethod','var')
    cutmethod = 'cutBoth';
end

global X;
NewCurve = [];
NewCurve.x = [];
NewCurve.y = [];
NewCurve.PointType = [];
beamLeadingEdgeX = xposition-width/2;
beamTrailingEdgeX = xposition+width/2;

switch cutmethod
    case 'cutLeft'
        % do nothing
    case {'cutRight','cutBoth','cutBothAndAddedCarbon'}
        indexBeforeLEcut = find(Curve.x<=beamLeadingEdgeX);
        NewCurve.x = Curve.x(indexBeforeLEcut);
        NewCurve.y = Curve.y(indexBeforeLEcut);
        NewCurve.PointType = Curve.PointType(indexBeforeLEcut);
        y_valueLE = interp1nearby(Curve.x,Curve.y,Curve.PointType,beamLeadingEdgeX,2,X.curvefitmethod);
        if ismember(beamLeadingEdgeX,Curve.x)
            % we don't need to insert the point and begin slot cut
        else
            NewCurve.x = [NewCurve.x;beamLeadingEdgeX];
            NewCurve.y = [NewCurve.y;y_valueLE];
            NewCurve.PointType = [NewCurve.PointType;X.PointType.fundamental];
        end
    otherwise
        assert(false);
end
y_valueSlotLeftEdge = interp1nearby(Curve.x,Curve.y,Curve.PointType,xposition-width/2,2,X.curvefitmethod);
y_valueSlotRightEdge = interp1nearby(Curve.x,Curve.y,Curve.PointType,xposition+width/2,2,X.curvefitmethod);

% depth: depth to be cut. 
%   +: cut to Y-, downward, for example, top edge
%   -: cut to Y+, upward, for example, bottom edge
if (depth > 0)
    SlotcutDestination = min([y_valueSlotLeftEdge y_valueSlotRightEdge]) - depth;
else
    SlotcutDestination = max([y_valueSlotLeftEdge y_valueSlotRightEdge]) - depth;
end

NewCurve.x = [NewCurve.x;beamLeadingEdgeX];
NewCurve.y = [NewCurve.y;SlotcutDestination];
NewCurve.PointType = [NewCurve.PointType;X.PointType.addedPoints];

NewCurve.x = [NewCurve.x;beamTrailingEdgeX];
NewCurve.y = [NewCurve.y;SlotcutDestination];
NewCurve.PointType = [NewCurve.PointType;X.PointType.addedPoints];

switch cutmethod
    case 'cutRight'
        % do nothing
    case {'cutLeft','cutBoth'}
        if ~ismember(beamTrailingEdgeX,Curve.x)
            y_valueTE = interp1nearby(Curve.x,Curve.y,Curve.PointType,beamTrailingEdgeX,2,X.curvefitmethod);
            NewCurve.x = [NewCurve.x;beamTrailingEdgeX];
            NewCurve.y = [NewCurve.y;y_valueTE];
            NewCurve.PointType = [NewCurve.PointType;X.PointType.fundamental];
        end
    case 'cutBothAndAddedCarbon'
        carbinTubeCutDestination = SlotcutDestination + depth + X.beamSquareCarbonTubeSize_mm;
        NewCurve.x = [NewCurve.x;beamTrailingEdgeX];
        NewCurve.y = [NewCurve.y;carbinTubeCutDestination];
        NewCurve.PointType = [NewCurve.PointType;X.PointType.addedPoints];
        TE_X_withCarbon_mm = beamTrailingEdgeX + X.beamSquareCarbonTubeSize_mm;
        NewCurve.x = [NewCurve.x;TE_X_withCarbon_mm];
        NewCurve.y = [NewCurve.y;carbinTubeCutDestination];
        NewCurve.PointType = [NewCurve.PointType;X.PointType.addedPoints];
        
        if ~ismember(TE_X_withCarbon_mm,Curve.x)
            y_valueTE = interp1nearby(Curve.x,Curve.y,Curve.PointType,TE_X_withCarbon_mm,2,X.curvefitmethod);
            NewCurve.x = [NewCurve.x;TE_X_withCarbon_mm];
            NewCurve.y = [NewCurve.y;y_valueTE];
            NewCurve.PointType = [NewCurve.PointType;X.PointType.fundamental];
        end
    otherwise
        assert(false);
end

switch cutmethod
    case {'cutLeft','cutRight','cutBoth'}
        indexAfterTEcut = find(Curve.x>=beamTrailingEdgeX);
    case 'cutBothAndAddedCarbon'
        indexAfterTEcut = find(Curve.x>=beamTrailingEdgeX+X.beamSquareCarbonTubeSize_mm);
    otherwise
        assert(false);
end

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

function [top_x,top_y,btm_x,btm_y] = airfoilProfile2twoPieces(x,y)
end_of_center_top_curve = (length(x)+1)/2;
top_x = x(1:end_of_center_top_curve);
top_y = y(1:end_of_center_top_curve);
btm_x = x(end_of_center_top_curve:end);
btm_y = y(end_of_center_top_curve:end);
end

function [x,y] = twoPieces2airfoilProfile(top_x,top_y,btm_x,btm_y)
x = [top_x; btm_x(2:end)];
y = [top_y; btm_y(2:end)];
end

function printXFLR5info(ribs_lengthmm,twist_ribsDeg)
global X;
Ylocation_mm = X.setting.geometry.wingSpan_mm/2*(0:X.OneSideRibCnt-1)/(X.OneSideRibCnt-1); 
LEoffset = X.setting.geometry.wingSpan_mm/2*tand(X.setting.geometry.LE_sweepbackDeg)*(0:X.OneSideRibCnt-1)/(X.OneSideRibCnt-1); 
fprintf(1,'index Y(mm) cordlength(mm) leadingEdgeOffset(mm) Twist(Deg)\n');
for ii = 1:length(ribs_lengthmm)
    fprintf(1,'%3d %10.3f %10.3f %10.3f %10.2f\n',ii,Ylocation_mm(ii),ribs_lengthmm(ii),LEoffset(ii),twist_ribsDeg(ii));
    fprintf(1,'----------------------------------------------------------\n');
end
end
