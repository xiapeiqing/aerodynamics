aircraft = '8mCarbonTubeSpar';
geometry = [];
structcfg = [];
switch aircraft
    case '3m'
        geometry.scalingfactor=0.8;
        geometry.LE_sweepbackDeg=24;
        geometry.dihedralDeg=2.5;
        structcfg.buildingmethod = 'woodspar+2carbonRod';
        structcfg.ribCnt=61;
        structcfg.RibCntCarbonSupportedSpar=41;
    case '1.7m'
        geometry.scalingfactor=0.45;
        geometry.LE_sweepbackDeg=24;
        geometry.dihedralDeg=2.5;
        structcfg.buildingmethod = 'woodspar+2carbonRod';
        structcfg.ribCnt=39;
        structcfg.RibCntCarbonSupportedSpar=25;
    case '1.7meter0dihedral'
        geometry.scalingfactor=0.45;
        geometry.LE_sweepbackDeg=24;
        geometry.dihedralDeg=0;
        structcfg.buildingmethod = 'woodspar+2carbonRod';
        structcfg.ribCnt=39;
        structcfg.RibCntCarbonSupportedSpar=25;
    case '1.7meter0dihedral0sweepback'
        geometry.scalingfactor=0.45;
        geometry.LE_sweepbackDeg=0;
        geometry.dihedralDeg=0;
        structcfg.buildingmethod = 'woodspar+2carbonRod';
        structcfg.ribCnt=39;
        structcfg.RibCntCarbonSupportedSpar=25;
    case '3mCarbonTubeSpar'
        geometry.scalingfactor=0.8;
        geometry.LE_sweepbackDeg=24;
        geometry.dihedralDeg=2.5;
        structcfg.buildingmethod = 'CarbonTubeSpar';
        structcfg.ribCnt=61;
        structcfg.RibCntCarbonSupportedSpar=41;
    case '8mCarbonTubeSpar'
        geometry.scalingfactor=8e3/3750;
        geometry.LE_sweepbackDeg=24;
        geometry.dihedralDeg=0;
        structcfg.buildingmethod = 'CarbonTubeSpar';
        structcfg.ribCnt=161;
        structcfg.RibCntCarbonSupportedSpar=121;
end
usrctrl = [];
usrctrl.sanityTest = true;
draw_airfoil(geometry,structcfg,usrctrl);