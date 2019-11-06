module Vehicles

open System

type SLS() as this =

    let mutable maxSeenQ:float = -1.0
    let mutable kickHeight:float = 0.0
    let mutable timeAtBurnout:float = 0.0

    let mutable Mo1:float = 200780.0  // dry mass in kg of stage 1
    let mutable Mo2:float = 90275.0  // dry mass in kg of stage 2
    let mutable Mo3:float = 4354.0  // dry mass in kg of Stage 3

    // Mass of each stage in kg
    let mutable M1:float = 1463770.0  // wet mass in kg of stage 1
    let mutable M2:float = 1091452.0  // wet mass in kg of stage 2
    let mutable M3:float = 31207.0  // wet mass in kg of stage 3

    // thrust of each stage in  KN
    let mutable T1:float = 32000000.0  // thrust in N of stage 1
    // T2 = 9116000 //- (16.5408339*P) // 7440000 N SL 9116000 N vac thrust in kn of stage 2
    let mutable T3:float = 110100.0  // thrust in N of stage 3

    let mutable FairingMass:float = 4000.0  // fairing mass

    // Burn time of each stage in seconds
    let mutable Bt1:float = 126.0
    let mutable Bt2:float = 476.0
    let mutable Bt3:float = 1125.0

    // Drag coeficiant of each stage
    let mutable CD1:float = 0.394356099087654  // (for each booster)
    let mutable CD2:float = 0.0
    let mutable CD3:float = 0.0

    // Reference area of each stage
    let mutable RefA1:float = 43.2411
    let mutable RefA2:float = 0.0
    let mutable RefA3:float = 0.0

    let mutable PayloadM:float = 28600.0 //  ERV Earth Return Vehcicle mass at laucnh


    member this.getT2 (p:float) =
        9116000.0 - (16.5408339 * p)


    member this.getLaunchMass (stageNo:int) =
        M1 + M2 + M3 + PayloadM + FairingMass;  // Current mass in kg


    member this.MassAndStageChanger (t:float) (timeStep:float) (stageNo:int) (localAirDensity:float) (localPressure:float) (V:float) (M:float) machNo =
      // Mass and Stage changer
      let vT2 = this.getT2 localPressure          // 7440000 N SL 9116000 N vac thrust in kn of stage 2

      let mutable nextM:float = M
      let mutable thrustN:float = 0.0
      let mutable forceOfDrag:float = 0.0
      let mutable nextStageNo:int = 0

      if t < Bt1 then
        nextStageNo <- 1
        thrustN <- T1 + vT2
        forceOfDrag <- (2.0 * (0.5 * localAirDensity * (V ** 2.0) * CD1 * RefA1)) + (0.5 * localAirDensity * (V ** 2.0) * CD2 * RefA2)
        nextM <- (M - ((M1 - Mo1) / Bt1) * timeStep)
        nextM <- (M - ((M2 - Mo2) / Bt2) * timeStep)
      elif t < Bt2 then
        nextM <- (M - ((M2 - Mo2) / Bt2) * timeStep)
        forceOfDrag <- (0.5 * localAirDensity * (V ** 2.0) * CD2 * RefA2)
        thrustN <- vT2
        if stageNo = 1 then
          nextM <- M - Mo1
          nextStageNo <- 2
      elif t < (Bt3 + Bt2) then
        thrustN <- T3
        nextM <- (M-((M3-Mo3) / Bt3)*timeStep)
        forceOfDrag <- (0.5 * localAirDensity * (V ** 2.0) * CD3 * RefA3)
        if (stageNo = 2) then
          nextStageNo <- 3
          nextM <- M - (Mo2 + FairingMass)
      else // if (t > Bt3 + Bt2)
        thrustN <- 0.0
        forceOfDrag <- (0.5 * localAirDensity * (V ** 2.0) * CD3 * RefA3)
        if timeAtBurnout = 0.0 then
            timeAtBurnout <- t
            printfn "Burnout at %f seconds" t
        if stageNo = 3 then
          nextStageNo <- 4
          nextM <- M - Mo3
      M, thrustN, forceOfDrag, nextStageNo


    member this.PitchControl (t:float) (timeStep:float) (h:float) (FlightPathDeg:float) (HeadingDeg:float) (PolarCoordDeg:float) (dynamicQ:float) =
        // Pitch Control Commands
        //kickHeight = 0
        //kickTime = 60
        //pitchEaseWindow = 50
        //kickAngleDeg = 36.4
        //pitch = 0

        //if (t >= kickTime) and (t <= kickTime + pitchEaseWindow):
        //    pitch = (kickAngleDeg / pitchEaseWindow) * (t - kickTime)
        //else:
        //    if FlightPathDeg < kickAngleDeg:
        //        pitch = kickAngleDeg
        //    else:
        //        pitch = FlightPathDeg

        if (dynamicQ > this.maxSeenQ) then
            maxSeenQ <- dynamicQ
        elif this.kickHeight = 0 then
            printfn "Max Seen Q was %s , chosen kick height was %s" maxSeenQ h
            this.kickHeight = h
        kickHeight = this.kickHeight
        kickAngleDeg = 20
        kickTransitionWindow = 20
        kickWindow = 100
        let mutable heading:float = 0.0

        if kickHeight > 0.0 then
            if h < kickHeight then
                heading <- 0.0
            elif h < kickHeight + kickTransitionWindow then
                heading <- (h - kickHeight) * (kickAngleDeg / kickTransitionWindow)
            else
                if FlightPathDeg < kickAngleDeg then
                    heading <- kickAngleDeg
                else
                    heading <- FlightPathDeg
        // if (h > 100000):
        //    if heading > 90 + PolarCoordDeg:
        //        heading <- 90 + PolarCoordDeg
        // else:
        //    if heading > 85 + PolarCoordDeg:
        //        heading <- 85 + PolarCoordDeg

        while heading > 360.0 do
            heading <- heading - 360.0
        while heading < 0 do
            heading <- heading + 360.0

        heading, kickAngleDeg, kickHeight


type SLSBlock1() as this =
    inherit SLS()

    member this.__init__(this):
        // SLS Block 1 (ish)

        this.maxSeenQ = -1
        this.kickHeight = 0
        this.timeAtBurnout = 0

        this.Mo1 = 200780  // dry mass in kg of stage 1
        this.Mo2 = 90275  // dry mass in kg of stage 2
        this.Mo3 = 4354  // dry mass in kg of Stage 3

        // Mass of each stage in kg
        this.M1 = 1463770  // wet mass in kg of stage 1
        this.M2 = 1091452  // wet mass in kg of stage 2
        this.M3 = 31207  // wet mass in kg of stage 3

        // thrust of each stage in  KN
        this.T1 = 32000000  // thrust in N of stage 1
        // T2 = 9116000 //- (16.5408339*P) // 7440000 N SL 9116000 N vac thrust in kn of stage 2
        this.T3 = 110100  // thrust in N of stage 3

        this.FairingMass = 4000  // fairing mass

        // Burn time of each stage in seconds
        this.Bt1 = 126
        this.Bt2 = 476
        this.Bt3 = 1125

        // Drag coeficiant of each stage
        this.CD1 = 0.394356099087654  // (for each booster)
        this.CD2 = None
        this.CD3 = None

        // Reference area of each stage
        this.RefA1 = 43.2411
        this.RefA2 = None
        this.RefA3 = None



class SLSBlock1B(SLS):

    member this.__init__(this):
        // SLS Block 1B (ish)

        this.maxSeenQ = -1
        this.kickHeight = 0
        this.timeAtBurnout = 0

        this.Mo1 = 200780 // dry mass in kg of stage 1
        this.Mo2 = 85275  // dry mass in kg of stage 2
        this.Mo3 = 13092  // dry mass in kg of Stage 3 (interpolated)

        this.M1 = 1463770 // wet mass in kg of stage 1
        this.M2 = 1091452 // wet mass in kg of stage 2
        this.M3 = 142000  // wet mass in kg of stage 3 (interpolated)

        this.T1 = 32000000// thrust in N of stage 1
        //T2 = 9116000 //- (16.5408339*P) // 7440000 N SL 9116000 N vac thrust in kn of stage 2
        this.T3 = 440000  // thrust in N of stage 3

        this.FairingMass = 8000  // fairing mass

        //Burn time of each stage in seconds
        this.Bt1 = 126
        this.Bt2 = 476
        this.Bt3 = 5352

        // Drag coeficiant of each stage
        this.CD1 = 0.394356099087654 // (for each booster)
        this.CD2 = 0.46
        this.CD3 = 0.8

        // Reference area of each stage
        this.RefA1 = 43.2411
        this.RefA2 = 221.6705904
        this.RefA3 = 221.6705904

        this.PayloadM = 28600 //  ERV Earth Return Vehcicle mass at laucnh
        //this.PayloadM = 25200 //  Hab Habitiation/Lander Module mass at laucnh


class SLSBlock2(SLS):

    member this.__init__(this):
        // SLS Block 2 (ish)

        this.maxSeenQ = -1
        this.kickHeight = 0
        this.timeAtBurnout = 0

        this.Mo1 = 168000 // dry mass in kg of stage 1
        this.Mo2 = 112000 // dry mass in kg of stage 2
        this.Mo3 = 13092  // dry mass in kg of Stage 3 (interpolated)

        this.M1 = 1586000 // wet mass in kg of stage 1
        this.M2 = 1091452 // wet mass in kg of stage 2
        this.M3 = 142000  // wet mass in kg of stage 3 (interpolated)

        this.T1 = 40000000// thrust in N of stage 1
        this.T3 = 440000  // thrust in N of stage 3

        this.FairingMass = 8000 //  fairing mass

        //Burn time of each stage in seconds
        this.Bt1 = 110
        this.Bt2 = 476
        this.Bt3 = 5352

        // Drag coeficiant of each stage
        this.CD1 = 0.394356099087654 // (for each booster)
        this.CD2 = 0.46
        this.CD3 = 0.8

        // Reference area of each stage
        this.RefA1 = 43.2411
        this.RefA2 = 221.6705904
        this.RefA3 = 221.6705904

        this.PayloadM = 28600.0 //  ERV Earth Return Vehcicle mass at laucnh
        //this.PayloadM = 25200 //  Hab Habitiation/Lander Module mass at laucnh


class SaturnV():

    member this.__init__(this):
        // Drag coefficients for every 0.5 Mach (starting at 0)
        this.dragCoefTable = [0.3, 0.26, 0.4, 0.55, 0.47, 0.36, 0.277, 0.23, 0.21, 0.205,
                         0.2, 0.205, 0.21, 0.22, 0.23, 0.24, 0.25, 0.257, 0.26, 0.26, 0.26]

        // Saturn V with  (ish)

        this.payload_LESmass = 4173 //Launch escape system mass in kg on front of payload

        this.maxSeenQ = -1
        this.kickHeight = 0
        this.timeAtBurnout = 0

        this.pl_commMod = 5806   // Command module mass
        this.pl_serv = 24523  // Service module mass
        this.pl_lmAdapter = 1800   // LM adapter mass
        this.pl_LMtotalMass = 15200  // LM total mass
        this.payload_Apollo = this.pl_commMod + this.pl_serv + this.pl_lmAdapter + this.pl_LMtotalMass + this.payload_LESmass
        this.stage2_AftInterstageMass = 5195
        this.stage3_AftInterstageMass = 3650

        this.Mo1 = 130570  // dry mass in kg of stage 1
        this.Mo2 = 85275   // dry mass in kg of stage 2
        this.Mo3 = 13092   // dry mass in kg of Stage 3
        this.M1 = 2149500 + this.Mo1        // wet mass in kg of stage 1
        this.M2 = 451650 + this.Mo2 + 5195 // wet mass in kg of stage 2
        this.M3 = 106940 + this.Mo3 + 3650  // wet mass in kg of stage 3

        this.stageOneStartThrust = 34354772
        this.stageOneMaxThrust = 40064144
        this.stageOneOriginalStartLocalPressure = 165.075263457
        this.stageOneOriginalMaxLocalPressure = 100610

        //Math for determining thrust of first stage at a given pressure/altitude-+
        //ThrustN = stageOneMaxThrust - (((stageOneMaxThrust - stageOneStartThrust) /
        //                                (stageOneOriginalMaxLocalPressure - stageOneOriginalStartLocalPressure))
        //                               * localPressure)

        this.T2 = 5004000   // thrust in N of stage 2
        this.T3 = 1001000   // thrust in N of stage 3

        //Burn time and other event times of each stage in seconds
        this.F1RocketBurnRate = 2735.70738 //Burn rate of F-1 rocket engine in kg/s, inferred from burn times and fuel tank capacity
        this.J2RocketBurnRate = 306.59834 //Burn rate of J-2 rocket engine in kg/s, inferred from burn times and fuel tank capacity
        this.stage1_CenterEngCutoff = 135.2  // Center cuts off early at 135.5 reducing thrust and fuel consumption by 1/5
        this.stage1_OutboardEngCutoff = 162.63 // Outboard engine cutoff
        this.stage2_Ignition = 166
        this.stage2_AftInterstageJettisoned = 192.3
        this.stage2_LauchEscapeTowerJettison = 197.9
        this.stage2_CenterEngCutoff = 460.62
        this.stage2_OutboardEngCutoff = 548.22
        this.stage3_1stBurnStart = 560.2
        this.stage3_1stBurnCutoff = 699.33
        //this.stage3_2ndBurnStart = 9856.2
        //this.stage3_2ndBurnCutoff = 10203.03
        this.stage3_2ndBurnStart = 699.33       // Customized for out flightplan
        this.stage3_2ndBurnCutoff = 346.83+699.33

        // Drag coeficiant of each stage
        this.CD1 = 0.394356099087654
        this.CD2 = 0.46
        this.CD3 = 0.8

        // Reference area of each stage
        this.RefA1 = 112.97009664
        this.RefA2 = 79.48277735
        this.RefA3 = 34.2119151
  

    member this.getLaunchMass(this, stageNo):
        return this.M1 + this.M2 + this.M3 + this.payload_Apollo;  // Starting mass in kg


    member this.getT2(this, p):
        return 5004000


    member this.MassAndStageChanger(this, t, timeStep, stageNo, localAirDensity, localPressure, V, M, machNo):

      // Mass and Stage changer
      vT2 = this.getT2(localPressure)          // 7440000 N SL 9116000 N vac thrust in kn of stage 2
      if (t < this.stage1_CenterEngCutoff):
        StageNo = 1
        ThrustN = this.stageOneMaxThrust -
                      (
                          (this.stageOneMaxThrust - this.stageOneStartThrust) /
                          (this.stageOneOriginalMaxLocalPressure - this.stageOneOriginalStartLocalPressure)
                       ) * localPressure
        M = this.M1 + this.M2 + this.M3 + this.payload_Apollo //- (this.F1RocketBurnRate * 5 * t)
        forceOfDrag = (.5 * localAirDensity * math.pow(V, 2) * this.dragCoefTable[int(min(machNo, 10) * 2)] * this.RefA1)
      elif (t < this.stage1_OutboardEngCutoff):
        StageNo = 1
        ThrustN = (this.stageOneMaxThrust - (((this.stageOneMaxThrust - this.stageOneStartThrust) /
                                        (this.stageOneOriginalMaxLocalPressure - this.stageOneOriginalStartLocalPressure))
                                       * localPressure)) * 4/5
        forceOfDrag = (.5 * localAirDensity * math.pow(V, 2) * this.dragCoefTable[int(min(machNo, 10) * 2)] * this.RefA1)
        M = this.M1 + this.M2 + this.M3 + this.payload_Apollo -\
            (this.F1RocketBurnRate * 4 * (t - this.stage1_CenterEngCutoff) +
                (this.F1RocketBurnRate * 5 * this.stage1_CenterEngCutoff))
      elif (t < this.stage2_Ignition):
        M = this.M2 + this.M3 + this.payload_Apollo
        forceOfDrag = (.5 * localAirDensity * math.pow(V, 2) * this.dragCoefTable[int(min(machNo, 10) * 2)] * this.RefA1)
        ThrustN = 0
        StageNo = 2
      elif (t < (this.stage2_AftInterstageJettisoned)):
        M = this.M2 + this.M3 + this.payload_Apollo - (this.J2RocketBurnRate * 5 * (t - this.stage2_Ignition))
        ThrustN = this.T2
        forceOfDrag = (.5 * localAirDensity * math.pow(V, 2) * this.dragCoefTable[int(min(machNo, 10) * 2)] * this.RefA2)
        StageNo = 2
      elif  (t < this.stage2_CenterEngCutoff):
        ThrustN = this.T2
        M = (this.M2 + this.M3 + this.payload_LESmass - this.stage2_AftInterstageMass) - (this.J2RocketBurnRate * 5 * (t - this.stage2_Ignition))
        forceOfDrag = (.5 * localAirDensity * math.pow(V, 2) * this.dragCoefTable[int(min(machNo, 10) * 2)] * this.RefA2)
      elif  (t < this.stage2_OutboardEngCutoff):
        ThrustN = this.T2 * 4 / 5
        M = (M-((this.M2-this.Mo2) / this.stage2_OutboardEngCutoff)*timeStep)  //need calc
        forceOfDrag = (.5 * localAirDensity * math.pow(V, 2) * this.dragCoefTable[int(min(machNo, 10) * 2)] * this.RefA2)
        StageNo = 2
      elif  (t < this.stage3_1stBurnStart):
        M = this.M3 + this.payload_Apollo - (this.stage3_AftInterstageMass + this.payload_LESmass)
        forceOfDrag = (.5 * localAirDensity * math.pow(V, 2) * this.dragCoefTable[int(min(machNo, 10) * 2)] * this.RefA3)
        ThrustN = this.T2
        StageNo = 3
      elif (t < this.stage3_1stBurnCutoff):
        M = this.M3 + this.payload_Apollo - (this.stage3_AftInterstageMass + this.payload_LESmass) - (this.J2RocketBurnRate * (t - this.stage3_1stBurnStart))
        forceOfDrag = (.5 * localAirDensity * math.pow(V, 2) * this.dragCoefTable[int(min(machNo, 10) * 2)] * this.RefA3)
        ThrustN = this.T3
        StageNo = 3
      elif (t < this.stage3_2ndBurnStart):
        M = M = this.M3 + this.payload_Apollo - (this.stage3_AftInterstageMass + this.payload_LESmass) - (this.J2RocketBurnRate * (this.stage3_1stBurnCutoff - this.stage3_1stBurnStart))
        forceOfDrag = (.5 * localAirDensity * math.pow(V, 2) * this.CD2 * this.RefA2)
        ThrustN = 0
        StageNo = 3
      elif (t < this.stage3_2ndBurnCutoff):
        M = this.M3 + this.payload_Apollo - (this.stage3_AftInterstageMass + this.payload_LESmass) - ( (this.J2RocketBurnRate * (this.stage3_1stBurnCutoff - this.stage3_1stBurnStart)) + (t - this.stage3_2ndBurnStart))
        forceOfDrag = (.5 * localAirDensity * math.pow(V, 2) * this.dragCoefTable[int(min(machNo, 10) * 2)] * this.RefA3)
        ThrustN = this.T3
        StageNo = 3
      else:
        if this.timeAtBurnout = 0:
            this.timeAtBurnout = t
            print ('Burnout at %s seconds' % t)
        M = this.M3 + this.payload_Apollo - (this.stage3_AftInterstageMass + this.payload_LESmass) - ((this.J2RocketBurnRate * (this.stage3_1stBurnCutoff - this.stage3_1stBurnStart)) + (this.stage3_2ndBurnCutoff - this.stage3_2ndBurnStart))
        ThrustN = 0
        StageNo = 3
        forceOfDrag = (.5 * localAirDensity * math.pow(V, 2) * this.dragCoefTable[int(min(machNo, 10) * 2)] * this.RefA3)
      return M, ThrustN, forceOfDrag, StageNo


    member this.PitchControl(this, t, timeStep, h, FlightPathDeg, HeadingDeg, PolarCoordDeg, dynamicQ):
        // Pitch Control Commands
        //if (t < 0.3):
        //    ap = 0
        //elif (t < 30):
        //    ap = 0
        //elif (t < 80):
        //    ap = 0.728 * (t - 30)
        //elif (t < 135):
        //    ap = 36.40 + 0.469364 * (t - 80)
        //elif (t < 165):
        //    ap = 62.23 + 0.297 * (t - 135)
        //elif (t < 185):
        //    ap = 71.14 - 0.5285000 * (t - 165)
        //elif (t < 320):
        //    ap = 60.57 + 0.030963 * (t - 185)
        //elif (t < 460):
        //    ap = 64.75 + 0.09 * (t - 320)
        //elif (t < 480):
        //    ap = 77.35 - 0.138 * (t - 460)
        //elif (t < 550):
        //    ap = 74.59 + 0.0971429 * (t - 480)
        //elif (t < 570):
        //    ap = 81.39 - 0.207 * (t - 550)
        //elif (t < 640):
        //    ap = 77.25 + 0.1117143 * (t - 570)
        //elif (t < 705):
        //    ap = 85.07 + 0.0486154 * (t - 640)
        //else:
        //    ap = 88.23
        //return (ap - PolarCoordDeg)*0.55
        //k = .6
        //targetPerigee = 120000
        //pitch = 90 * math.pow(1 - (( h - 0) / (targetPerigee - 0)), (-k*h/targetPerigee)) - (90 + PolarCoordDeg)
        if (dynamicQ > this.maxSeenQ):
            this.maxSeenQ = dynamicQ
        elif this.kickHeight = 0:
            printfn "Max Seen Q was %s , chosen kick height was %s" maxSeenQ h
            this.kickHeight = h
        kickHeight = this.kickHeight
        kickAngleDeg = 1.4
        kickTransitionWindow = 200
        kickWindow = 100
        pitch = 0

        if kickHeight > 0:
            if h < kickHeight:
                pitch = 0
            elif h < kickHeight + kickTransitionWindow:
                pitch = (h - kickHeight) * (kickAngleDeg / kickTransitionWindow)
            else:
                if FlightPathDeg < kickAngleDeg:
                    pitch = kickAngleDeg
                else:
                    pitch = FlightPathDeg
        //if (h > 100000):
        //    if pitch > 90 + PolarCoordDeg:
        //        pitch = 90 + PolarCoordDeg
        //else:
        //    if pitch > 85 + PolarCoordDeg:
        //        pitch = 85 + PolarCoordDeg

        while pitch > 360:
            pitch = pitch - 360
        while pitch < 0:
            pitch = pitch + 360
        HeadingDeg = pitch
        return HeadingDeg, kickAngleDeg, kickHeight